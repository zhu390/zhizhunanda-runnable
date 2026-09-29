import argparse
import json
import time
from pathlib import Path

import cv2

from .camera_utils import apply_flip, open_camera
from .vision.facial_candidate import (
    FaceLocator,
    FacialCandidateDetector,
    FacialCandidateParams,
)
from .vision.mole_detect import DetectParams
from .vision.scan_candidate_fusion import (
    ScanCandidateFusion,
    ScanReferenceBoxSmoother,
)
from .vision.target_stability import CandidateStabilityTracker


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_FACE_MODEL = ROOT / "models" / "arm_face_yolo" / "best.pt"


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Reference-free facial dark-spot candidate preview"
    )
    parser.add_argument("--camera", default="1" if __import__("sys").platform == "win32" else "0")
    parser.add_argument("--image", default="", help="Process one image instead of a camera")
    parser.add_argument("--width", type=int, default=1920)
    parser.add_argument("--height", type=int, default=1080)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--flip", choices=["none", "0", "1", "-1"], default="none")
    parser.add_argument("--face-model", default=str(DEFAULT_FACE_MODEL))
    parser.add_argument("--face-conf", type=float, default=0.35)
    parser.add_argument("--face-device", default="")
    parser.add_argument("--min-area", type=int, default=8)
    parser.add_argument("--max-area", type=int, default=1200)
    parser.add_argument("--min-contrast", type=int, default=22)
    parser.add_argument("--min-circularity", type=float, default=0.55)
    parser.add_argument("--max-candidates", type=int, default=12)
    parser.add_argument("--stable-frames", type=int, default=4, help="连续稳定出现多少帧后才显示")
    parser.add_argument("--stable-radius", type=float, default=6.0, help="多帧中心允许的最大漂移像素")
    parser.add_argument(
        "--scan-mode",
        action="store_true",
        help=(
            "横向扫描发现模式：平滑人脸参考框、容忍短暂运动模糊，"
            "退出时只输出脸部相对候选摘要，不输出驱动坐标"
        ),
    )
    parser.add_argument(
        "--scan-reference-alpha",
        type=float,
        default=0.10,
        help="横向扫描人脸参考框低通系数，越小越抗框抖动",
    )
    parser.add_argument("--view", choices=["privacy", "algorithm", "mask"], default="privacy")
    parser.add_argument("--background", choices=["black", "blur"], default="black")
    parser.add_argument("--save", default="", help="Save the latest shown result with S")
    parser.add_argument(
        "--record-dir",
        default="",
        help=(
            "Optional diagnostic directory. Records untouched BGR frames to "
            "raw_bgr_mjpg.avi and per-frame detector/tracker data to frames.jsonl."
        ),
    )
    return parser.parse_args(argv)


def _candidate_payload(candidate, include_tracking=False):
    payload = {
        "center": [int(candidate.center[0]), int(candidate.center[1])],
        "radius": int(candidate.radius),
        "area": float(candidate.area),
        "circularity": float(candidate.circularity),
        "contrast": float(candidate.contrast),
    }
    if include_tracking:
        payload.update(
            {
                "track_id": int(getattr(candidate, "track_id", 0)),
                "stable_count": int(getattr(candidate, "stable_count", 0)),
                "required_count": int(getattr(candidate, "required_count", 0)),
                "track_age_s": float(getattr(candidate, "track_age_s", 0.0)),
            }
        )
    return payload


def serialize_diagnostic_frame(
    frame_index,
    monotonic_s,
    result,
    raw_candidates,
    stable_candidates,
    tracking_box=None,
    detected_tracking_box=None,
):
    """Build one JSON-safe record without modifying detector results."""

    face = getattr(result, "face", None)
    face_payload = None
    if face is not None:
        face_payload = {
            "x": int(face.x),
            "y": int(face.y),
            "width": int(face.width),
            "height": int(face.height),
            "confidence": float(face.confidence),
            "source": str(face.source),
        }
    return {
        "schema_version": 1,
        "monotonic_s": float(monotonic_s),
        "frame_index": int(frame_index),
        "face": face_payload,
        "tracking_box": (
            [int(value) for value in tracking_box]
            if tracking_box is not None
            else None
        ),
        "detected_tracking_box": (
            [int(value) for value in detected_tracking_box]
            if detected_tracking_box is not None
            else None
        ),
        "raw_candidates": [
            _candidate_payload(candidate) for candidate in raw_candidates
        ],
        "stable_candidates": [
            _candidate_payload(candidate, include_tracking=True)
            for candidate in stable_candidates
        ],
        "message": str(getattr(result, "message", "") or ""),
    }


class PreviewDiagnosticRecorder:
    """Write raw preview frames and matching JSONL diagnostics in one process."""

    VIDEO_FILENAME = "raw_bgr_mjpg.avi"
    JSONL_FILENAME = "frames.jsonl"

    def __init__(self, record_dir, requested_fps):
        self.record_dir = Path(record_dir).expanduser()
        self.record_dir.mkdir(parents=True, exist_ok=True)
        self.requested_fps = max(1.0, float(requested_fps))
        self.video_path = self.record_dir / self.VIDEO_FILENAME
        self.jsonl_path = self.record_dir / self.JSONL_FILENAME
        self._writer = None
        self._frame_size = None
        self._jsonl = self.jsonl_path.open("w", encoding="utf-8")
        self._closed = False

    def _ensure_writer(self, frame):
        if frame is None or frame.ndim != 3 or frame.shape[2] != 3:
            raise ValueError("diagnostic recorder requires a BGR frame")
        frame_size = (int(frame.shape[1]), int(frame.shape[0]))
        if self._writer is not None:
            if frame_size != self._frame_size:
                raise RuntimeError(
                    "camera frame size changed from {} to {}".format(
                        self._frame_size,
                        frame_size,
                    )
                )
            return

        fourcc = cv2.VideoWriter_fourcc(*"MJPG")
        writer = cv2.VideoWriter(
            str(self.video_path),
            fourcc,
            self.requested_fps,
            frame_size,
        )
        if not writer.isOpened():
            writer.release()
            raise RuntimeError(
                "cannot open diagnostic MJPG writer: {}".format(self.video_path)
            )
        self._writer = writer
        self._frame_size = frame_size

    def record(
        self,
        frame,
        frame_index,
        monotonic_s,
        result,
        raw_candidates,
        stable_candidates,
        tracking_box=None,
        detected_tracking_box=None,
    ):
        if self._closed:
            raise RuntimeError("diagnostic recorder is already closed")
        self._ensure_writer(frame)
        payload = serialize_diagnostic_frame(
            frame_index,
            monotonic_s,
            result,
            raw_candidates,
            stable_candidates,
            tracking_box=tracking_box,
            detected_tracking_box=detected_tracking_box,
        )
        self._writer.write(frame)
        self._jsonl.write(
            json.dumps(payload, ensure_ascii=False, separators=(",", ":")) + "\n"
        )
        self._jsonl.flush()

    def close(self):
        if self._closed:
            return
        self._closed = True
        try:
            if self._writer is not None:
                self._writer.release()
        finally:
            self._jsonl.close()


def build_detector(args):
    model_path = Path(args.face_model).expanduser() if args.face_model else None
    locator = FaceLocator(
        model_path=model_path,
        confidence=args.face_conf,
        imgsz=640,
        device=args.face_device,
    )
    candidate_params = DetectParams(
        min_area=max(1, args.min_area),
        max_area=max(args.min_area, args.max_area),
        min_circularity=max(0.0, min(1.0, args.min_circularity)),
        min_solidity=0.70,
        max_aspect_ratio=2.4,
        blackhat_ksize=31,
        min_contrast=max(0, args.min_contrast),
        skin_dilate=5,
        use_skin_mask=True,
    )
    params = FacialCandidateParams(
        max_candidates=max(1, args.max_candidates),
        background_mode=args.background,
        detection_exclusion_mode="eyes_only",
    )
    return FacialCandidateDetector(locator, candidate_params, params)


def select_view(result, view):
    if view == "algorithm":
        return result.annotated
    if view == "mask":
        return cv2.cvtColor(result.valid_mask, cv2.COLOR_GRAY2BGR)
    return result.privacy_view


def fit_for_display(frame, max_width=1280, max_height=720):
    height, width = frame.shape[:2]
    scale = min(max_width / width, max_height / height, 1.0)
    if scale >= 1.0:
        return frame
    return cv2.resize(
        frame,
        (int(round(width * scale)), int(round(height * scale))),
        interpolation=cv2.INTER_AREA,
    )


def main():
    args = parse_args()
    detector = build_detector(args)
    scan_mode = bool(getattr(args, "scan_mode", False))
    scan_reference_alpha = float(
        getattr(args, "scan_reference_alpha", 0.10)
    )
    if scan_mode:
        tracker_options = {
            "match_radius": 40.0,
            "max_missed": 45,
            "max_gap_s": 10.0,
            "fast_reacquire_max_gap_s": 2.0,
        }
    else:
        tracker_options = {
            "fast_reacquire_max_gap_s": 0.45,
        }
    tracker = CandidateStabilityTracker(
        required_hits=max(1, args.stable_frames),
        history_size=max(7, args.stable_frames + 2),
        stable_radius=max(0.5, args.stable_radius),
        fast_reacquire_hits=2,
        fast_reacquire_min_duration_s=0.10,
        **tracker_options,
    )
    scan_reference_smoother = None
    scan_fusion = None
    if scan_mode:
        scan_reference_smoother = ScanReferenceBoxSmoother(
            alpha=scan_reference_alpha,
        )
        scan_fusion = ScanCandidateFusion(
            min_hits=5,
            min_view_span_face_widths=0.30,
            max_missed_frames=8,
            max_gap_s=2.0,
            reference_box_alpha=scan_reference_alpha,
            stitch_max_gap_s=12.0,
        )
    window = "Facial Candidate (1 privacy / 2 algorithm / 3 mask, S save, Q quit)"
    current_view = args.view
    save_path = Path(args.save).expanduser() if args.save else Path("facial_candidate_result.jpg")

    if args.image:
        frame = cv2.imread(str(Path(args.image).expanduser()))
        if frame is None:
            raise RuntimeError("cannot read image: {}".format(args.image))
        result = detector.detect(frame)
        shown = select_view(result, current_view)
        cv2.imwrite(str(save_path), shown)
        print("{}; saved {}".format(result.message, save_path), flush=True)
        cv2.imshow(window, fit_for_display(shown))
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        return 0

    camera, label = open_camera(args.camera, args.width, args.height, args.fps)
    print("camera opened: {}".format(label), flush=True)
    recorder = None
    last_result = None
    last_shown = None
    frame_count = 0
    started = time.monotonic()
    try:
        if args.record_dir:
            recorder = PreviewDiagnosticRecorder(args.record_dir, args.fps)
            print(
                "diagnostic recording: video={} jsonl={}".format(
                    recorder.video_path,
                    recorder.jsonl_path,
                ),
                flush=True,
            )
        while True:
            ok, frame = camera.read()
            if not ok or frame is None:
                time.sleep(0.01)
                continue
            frame = apply_flip(frame, args.flip)
            sample_time = time.monotonic()
            last_result = detector.detect(frame)
            raw_candidates = list(last_result.candidates)
            raw_count = len(raw_candidates)
            detected_tracking_box = None
            if last_result.face_mask is not None:
                mask_points = cv2.findNonZero(last_result.face_mask)
                if mask_points is not None:
                    detected_tracking_box = cv2.boundingRect(mask_points)
            tracking_box = detected_tracking_box
            if scan_reference_smoother is not None:
                tracking_box = scan_reference_smoother.update(
                    detected_tracking_box
                )
            stable_candidates = tracker.update(
                raw_candidates,
                sample_time=sample_time,
                reference_box=tracking_box,
            )
            if scan_fusion is not None:
                scan_fusion.update(
                    raw_candidates,
                    reference_box=detected_tracking_box,
                    sample_time=sample_time,
                )
            if recorder is not None:
                recorder.record(
                    frame,
                    frame_count,
                    sample_time,
                    last_result,
                    raw_candidates,
                    stable_candidates,
                    tracking_box=tracking_box,
                    detected_tracking_box=detected_tracking_box,
                )
            last_result = detector.render_candidates(
                frame,
                last_result,
                stable_candidates,
            )
            last_result.message = "{} stable / {} raw".format(
                len(stable_candidates),
                raw_count,
            )
            last_shown = select_view(last_result, current_view)
            frame_count += 1
            fps = frame_count / max(0.001, time.monotonic() - started)
            cv2.putText(
                last_shown,
                "FPS {:.1f} | {}".format(fps, last_result.message),
                (12, 28),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (60, 240, 60),
                2,
                cv2.LINE_AA,
            )
            cv2.imshow(window, fit_for_display(last_shown))
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q"), ord("Q")):
                break
            if key == ord("1"):
                current_view = "privacy"
            elif key == ord("2"):
                current_view = "algorithm"
            elif key == ord("3"):
                current_view = "mask"
            elif key in (ord("s"), ord("S")) and last_shown is not None:
                cv2.imwrite(str(save_path), last_shown)
                print("saved {}".format(save_path), flush=True)
    except KeyboardInterrupt:
        print("preview interrupted", flush=True)
    finally:
        try:
            camera.release()
        finally:
            try:
                if recorder is not None:
                    recorder.close()
            finally:
                cv2.destroyAllWindows()
    if scan_fusion is not None:
        scan_result = scan_fusion.finish()
        if scan_result.selected is None:
            print(
                "[SCAN_RESULT] INVALID reason={} candidates={}".format(
                    scan_result.reason,
                    len(scan_result.candidates),
                ),
                flush=True,
            )
        else:
            selected = scan_result.selected
            print(
                (
                    "[SCAN_RESULT] UNIQUE track_id={} relative=({:.4f},{:.4f}) "
                    "hits={} hit_ratio={:.3f} view_span={:.3f} confidence={:.3f}"
                ).format(
                    selected.track_id,
                    selected.relative_center[0],
                    selected.relative_center[1],
                    selected.hits,
                    selected.hit_ratio,
                    selected.view_span_face_widths,
                    selected.confidence,
                ),
                flush=True,
            )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
