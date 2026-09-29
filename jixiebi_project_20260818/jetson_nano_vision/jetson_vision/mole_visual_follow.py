"""Isolated legacy visual-follow bench tool.

Do not run this script against the same serial port as the formal Qt
application.  Its optional serial path deliberately uses the archived legacy
AA/FF protocol and is retained only for isolated hardware bring-up.
"""

import argparse
import json
import time
from collections import deque
from pathlib import Path
from typing import Deque, List, Optional, Tuple

import cv2

from .camera_utils import apply_flip, open_camera
from .serial_sender import SerialSender
from .undistort import Undistorter
from .vision.mole_detect import DetectParams, DetectedMole, detect_moles
from .vision.reference_mole_detect import (
    ReferenceDetectParams,
    ReferenceMoleDetector,
)
from .vision.target_stability import stable_center
from .vision_protocol import (
    RA6_TO_JETSON_ERROR,
    RA6_TO_JETSON_READY,
    RA6_TO_JETSON_TARGET_CTRL,
)


STATE_PREVIEW = "PREVIEW"
STATE_WAIT_READY = "WAIT_READY"
STATE_FOLLOW = "FOLLOW"
STATE_ALIGNED = "ALIGNED"
STATE_NO_TARGET = "NO_TARGET"
STATE_UNSTABLE = "UNSTABLE"
STATE_RA6_ERROR = "RA6_ERROR"
STATE_ERROR_TOO_LARGE = "ERROR_TOO_LARGE"
STATE_VISUAL_ONLY = "VISUAL_ONLY"
STATE_ALIGN_FAILED = "ALIGN_FAILED"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Isolated legacy black-mole visual-follow test. The formal Qt "
            "application must not be running on the same serial port."
        )
    )
    parser.add_argument("--camera", default="0", help="Camera source: index, /dev/videoX, csi, or GStreamer pipeline")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--display", type=int, choices=[0, 1], default=1)
    parser.add_argument("--flip", choices=["none", "0", "1", "-1"], default="none")
    parser.add_argument("--calib-params", default="", help="Camera calibration .npz path")
    parser.add_argument("--params-json", default="", help="Detection parameters saved by mole_image_test.py")
    parser.add_argument("--detector-mode", choices=["spot", "reference"], default="spot",
                        help="spot=blackhat detector; reference=detect new dark marks against --reference.")
    parser.add_argument("--reference", default="",
                        help="Clean-face reference image. Non-empty enables reference detector.")
    parser.add_argument("--min-area", type=int, default=None,
                        help="Override detector min_area. Larger values reject tiny noise.")
    parser.add_argument("--max-area", type=int, default=None,
                        help="Override detector max_area. Smaller values reject eyes/base blobs.")
    parser.add_argument("--min-circularity", type=float, default=None,
                        help="Override detector min_circularity. Larger values prefer round moles.")
    parser.add_argument("--min-solidity", type=float, default=None,
                        help="Override detector min_solidity.")
    parser.add_argument("--max-aspect-ratio", type=float, default=None,
                        help="Override detector max_aspect_ratio. Smaller values reject long eyebrow/eye shapes.")
    parser.add_argument("--blackhat-ksize", type=int, default=None,
                        help="Override detector blackhat kernel size.")
    parser.add_argument("--min-contrast", type=int, default=None,
                        help="Override detector min_contrast. Larger values reject weak shadows.")
    parser.add_argument("--skin-dilate", type=int, default=None,
                        help="Override detector skin_dilate. Larger values make skin mask stricter.")
    parser.add_argument("--use-skin-mask", type=int, choices=[0, 1], default=None,
                        help="Override detector skin mask switch.")
    parser.add_argument("--ref-min-darkening", type=int, default=None,
                        help="Reference detector darkening threshold.")
    parser.add_argument("--ref-min-mean-darkening", type=float, default=None,
                        help="Reference detector mean darkening threshold.")
    parser.add_argument("--ref-min-confidence", type=float, default=None,
                        help="Reference detector confidence threshold.")
    parser.add_argument("--ref-mask-erode", type=int, default=None,
                        help="Reference detector skin mask erosion size.")
    parser.add_argument("--ref-max-translation-px", type=float, default=None,
                        help="Max reference alignment translation.")
    parser.add_argument("--ref-max-rotation-deg", type=float, default=None,
                        help="Max reference alignment rotation.")
    parser.add_argument("--ref-min-alignment-score", type=float, default=None,
                        help="Min reference alignment ECC score.")

    parser.add_argument(
        "--serial-port",
        default="",
        help="Isolated legacy bench serial port; never share it with the formal Qt process.",
    )
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--send-serial", type=int, choices=[0, 1], default=1,
                        help="Set 0 to only print centers without sending VISION_ERROR frames.")
    parser.add_argument("--start-target-ctrl", type=int, choices=[0, 1], default=1,
                        help="Send AA 01 01 BB before following when serial is connected")
    parser.add_argument("--stop-target-ctrl-on-exit", type=int, choices=[0, 1], default=1,
                        help="Send AA 01 00 BB before exit when target ctrl was started")
    parser.add_argument("--wait-ready", type=int, choices=[0, 1], default=1,
                        help="Wait for RA6M5 READY before sending follow errors")
    parser.add_argument("--ready-timeout-ms", type=int, default=30000,
                        help="Timeout for waiting READY. 0 disables timeout.")

    parser.add_argument("--send-rate-hz", type=float, default=8.0,
                        help="Max VISION_ERROR send rate while following")
    parser.add_argument("--deadband-px", type=int, default=5,
                        help="Send zero error when abs(dcx) and abs(dcy) are within this range")
    parser.add_argument("--zero-in-deadband", type=int, choices=[0, 1], default=1,
                        help="When aligned, send dcx=0 dcy=0 at the limited send rate")
    parser.add_argument("--max-error-px", type=int, default=400,
                        help="Do not send if abs(dcx) or abs(dcy) exceeds this value. 0 disables.")

    parser.add_argument("--stable-window", type=int, default=5,
                        help="Number of recent frames kept for stability check")
    parser.add_argument("--stable-required", type=int, default=3,
                        help="Required valid target count in the stability window")
    parser.add_argument("--stable-radius", type=float, default=8.0,
                        help="Max spread in pixels for stable target center")

    parser.add_argument("--roi-x-min-ratio", type=float, default=0.0,
                        help="Detection ROI left ratio. 0.0 means image left edge.")
    parser.add_argument("--roi-x-max-ratio", type=float, default=1.0,
                        help="Detection ROI right ratio. 1.0 means image right edge.")
    parser.add_argument("--roi-y-min-ratio", type=float, default=0.52,
                        help="Detection ROI top ratio. Default ignores eye/eyebrow area.")
    parser.add_argument("--roi-y-max-ratio", type=float, default=0.82,
                        help="Detection ROI bottom ratio. Default excludes the black model base.")
    parser.add_argument("--lock-target", type=int, choices=[0, 1], default=1,
                        help="Keep following the nearest target around the previous target.")
    parser.add_argument("--lock-max-distance-px", type=float, default=45.0,
                        help="Max distance for target lock matching.")
    parser.add_argument("--lock-lost-frames", type=int, default=5,
                        help="Clear target lock after this many unmatched frames.")
    parser.add_argument("--manual-lock", type=int, choices=[0, 1], default=1,
                        help="Enable left-click target lock in display window.")
    parser.add_argument("--manual-lock-radius-px", type=float, default=30.0,
                        help="Search radius after manual target lock.")
    parser.add_argument("--show-debug-text", type=int, choices=[0, 1], default=0,
                        help="Show full debug text overlay.")
    parser.add_argument("--show-roi", type=int, choices=[0, 1], default=0,
                        help="Show detection ROI rectangle.")
    parser.add_argument("--show-mole-labels", type=int, choices=[0, 1], default=0,
                        help="Show text labels beside detected moles.")
    parser.add_argument("--overlay-scale", type=float, default=0.35,
                        help="OpenCV overlay text scale.")
    parser.add_argument("--overlay-thickness", type=int, default=1,
                        help="OpenCV overlay text thickness.")

    parser.add_argument("--laser-offset-x-px", type=int, default=0,
                        help="Deprecated in this script; raw frame center is used.")
    parser.add_argument("--laser-offset-y-px", type=int, default=0,
                        help="Deprecated in this script; raw frame center is used.")
    parser.add_argument("--print-interval", type=float, default=0.5)
    return parser.parse_args()


def load_params(path: str) -> DetectParams:
    if not path:
        return DetectParams()
    params_path = Path(path)
    if not params_path.exists():
        print("params file not found, using defaults: {}".format(path), flush=True)
        return DetectParams()

    data = json.loads(params_path.read_text(encoding="utf-8"))
    return DetectParams(
        min_area=int(data.get("min_area", 8)),
        max_area=int(data.get("max_area", 600)),
        min_circularity=float(data.get("min_circularity", 0.55)),
        min_solidity=float(data.get("min_solidity", 0.80)),
        max_aspect_ratio=float(data.get("max_aspect_ratio", 2.2)),
        blackhat_ksize=int(data.get("blackhat_ksize", 21)),
        min_contrast=int(data.get("min_contrast", 18)),
        skin_dilate=int(data.get("skin_dilate", 9)),
        use_skin_mask=bool(data.get("use_skin_mask", True)),
    )


def apply_param_overrides(params: DetectParams, args: argparse.Namespace) -> DetectParams:
    return DetectParams(
        min_area=params.min_area if args.min_area is None else int(args.min_area),
        max_area=params.max_area if args.max_area is None else int(args.max_area),
        min_circularity=(
            params.min_circularity
            if args.min_circularity is None
            else float(args.min_circularity)
        ),
        min_solidity=(
            params.min_solidity
            if args.min_solidity is None
            else float(args.min_solidity)
        ),
        max_aspect_ratio=(
            params.max_aspect_ratio
            if args.max_aspect_ratio is None
            else float(args.max_aspect_ratio)
        ),
        blackhat_ksize=(
            params.blackhat_ksize
            if args.blackhat_ksize is None
            else int(args.blackhat_ksize)
        ),
        min_contrast=(
            params.min_contrast
            if args.min_contrast is None
            else int(args.min_contrast)
        ),
        skin_dilate=(
            params.skin_dilate
            if args.skin_dilate is None
            else int(args.skin_dilate)
        ),
        use_skin_mask=(
            params.use_skin_mask
            if args.use_skin_mask is None
            else bool(args.use_skin_mask)
        ),
    )


def reference_mode_enabled(args: argparse.Namespace) -> bool:
    return args.detector_mode == "reference" or bool(args.reference)


def build_reference_params(args: argparse.Namespace) -> ReferenceDetectParams:
    defaults = ReferenceDetectParams()
    min_darkening = defaults.min_darkening
    if args.min_contrast is not None:
        min_darkening = int(args.min_contrast)
    if args.ref_min_darkening is not None:
        min_darkening = int(args.ref_min_darkening)

    min_mean_darkening = defaults.min_mean_darkening
    if args.min_contrast is not None:
        min_mean_darkening = float(args.min_contrast)
    if args.ref_min_mean_darkening is not None:
        min_mean_darkening = float(args.ref_min_mean_darkening)

    return ReferenceDetectParams(
        min_area=defaults.min_area if args.min_area is None else int(args.min_area),
        max_area=defaults.max_area if args.max_area is None else int(args.max_area),
        min_darkening=min_darkening,
        min_mean_darkening=min_mean_darkening,
        min_circularity=(
            defaults.min_circularity
            if args.min_circularity is None
            else float(args.min_circularity)
        ),
        min_solidity=(
            defaults.min_solidity
            if args.min_solidity is None
            else float(args.min_solidity)
        ),
        min_confidence=(
            defaults.min_confidence
            if args.ref_min_confidence is None
            else float(args.ref_min_confidence)
        ),
        max_aspect_ratio=(
            defaults.max_aspect_ratio
            if args.max_aspect_ratio is None
            else float(args.max_aspect_ratio)
        ),
        mask_erode=(
            defaults.mask_erode
            if args.ref_mask_erode is None
            else int(args.ref_mask_erode)
        ),
        max_translation_px=(
            defaults.max_translation_px
            if args.ref_max_translation_px is None
            else float(args.ref_max_translation_px)
        ),
        max_rotation_deg=(
            defaults.max_rotation_deg
            if args.ref_max_rotation_deg is None
            else float(args.ref_max_rotation_deg)
        ),
        min_alignment_score=(
            defaults.min_alignment_score
            if args.ref_min_alignment_score is None
            else float(args.ref_min_alignment_score)
        ),
    )


def load_reference_detector(
    args: argparse.Namespace,
) -> Optional[ReferenceMoleDetector]:
    if not reference_mode_enabled(args):
        return None

    reference_path = args.reference or "reference/clean_face.png"
    reference = cv2.imread(reference_path)
    if reference is None:
        raise RuntimeError("reference image not found or unreadable: {}".format(reference_path))

    ref_h, ref_w = reference.shape[:2]
    if ref_w != args.width or ref_h != args.height:
        raise RuntimeError(
            "reference image size {}x{} does not match runtime size {}x{}".format(
                ref_w,
                ref_h,
                args.width,
                args.height,
            )
        )

    ref_params = build_reference_params(args)
    print(
        (
            "reference detector: image={} area={}..{} darkening>={} "
            "mean_darkening>={:.1f} confidence>={:.2f}"
        ).format(
            reference_path,
            ref_params.min_area,
            ref_params.max_area,
            ref_params.min_darkening,
            ref_params.min_mean_darkening,
            ref_params.min_confidence,
        ),
        flush=True,
    )
    try:
        return ReferenceMoleDetector(reference, ref_params)
    except ValueError as exc:
        raise RuntimeError("reference detector init failed: {}".format(exc))


def open_serial(args: argparse.Namespace) -> Optional[SerialSender]:
    if not args.serial_port:
        print("serial disabled: detect-only visual follow preview", flush=True)
        return None
    try:
        sender = SerialSender(
            args.serial_port,
            args.baudrate,
            protocol="legacy",
            auto_heartbeat=False,
        )
    except Exception as exc:
        print("serial open failed: {}".format(exc), flush=True)
        return None

    print(
        "WARNING: isolated LEGACY serial debug only; do not run the formal Qt serial session",
        flush=True,
    )
    print(
        "serial opened: {} @ {} protocol=legacy".format(args.serial_port, args.baudrate),
        flush=True,
    )
    return sender


def wait_period(send_rate_hz: float) -> float:
    if send_rate_hz <= 0:
        return 0.0
    return 1.0 / send_rate_hz


def target_error(
    center: Tuple[int, int],
    frame_size: Tuple[int, int],
) -> Tuple[int, int]:
    width, height = frame_size
    dcx = int(center[0]) - width // 2
    dcy = int(center[1]) - height // 2
    return dcx, dcy


def frame_center(frame_size: Tuple[int, int]) -> Tuple[int, int]:
    width, height = frame_size
    return int(width) // 2, int(height) // 2


def _clamp_ratio(value: float) -> float:
    return min(1.0, max(0.0, float(value)))


def detection_roi(frame_shape: Tuple[int, ...], args: argparse.Namespace) -> Optional[Tuple[int, int, int, int]]:
    h, w = frame_shape[:2]
    x_min = _clamp_ratio(args.roi_x_min_ratio)
    x_max = _clamp_ratio(args.roi_x_max_ratio)
    y_min = _clamp_ratio(args.roi_y_min_ratio)
    y_max = _clamp_ratio(args.roi_y_max_ratio)

    x0 = int(round(w * min(x_min, x_max)))
    x1 = int(round(w * max(x_min, x_max)))
    y0 = int(round(h * min(y_min, y_max)))
    y1 = int(round(h * max(y_min, y_max)))

    x0 = max(0, min(w - 1, x0))
    y0 = max(0, min(h - 1, y0))
    x1 = max(x0 + 1, min(w, x1))
    y1 = max(y0 + 1, min(h, y1))
    if x1 - x0 < 4 or y1 - y0 < 4:
        return None
    return x0, y0, x1 - x0, y1 - y0


def center_roi(
    frame_shape: Tuple[int, ...],
    center: Tuple[int, int],
    radius_px: float,
) -> Optional[Tuple[int, int, int, int]]:
    h, w = frame_shape[:2]
    radius = max(4, int(round(float(radius_px))))
    cx, cy = int(center[0]), int(center[1])
    x0 = max(0, cx - radius)
    y0 = max(0, cy - radius)
    x1 = min(w, cx + radius + 1)
    y1 = min(h, cy + radius + 1)
    if x1 - x0 < 4 or y1 - y0 < 4:
        return None
    return x0, y0, x1 - x0, y1 - y0


def _point_in_roi(
    center: Tuple[int, int],
    roi: Optional[Tuple[int, int, int, int]],
) -> bool:
    if roi is None:
        return True
    rx, ry, rw, rh = roi
    return rx <= center[0] < rx + rw and ry <= center[1] < ry + rh


def filter_moles_by_roi(moles, roi: Optional[Tuple[int, int, int, int]]):
    if roi is None:
        return moles
    return [mole for mole in moles if _point_in_roi(mole.center, roi)]


def _distance_sq(a: Tuple[int, int], b: Tuple[int, int]) -> float:
    dx = float(a[0] - b[0])
    dy = float(a[1] - b[1])
    return dx * dx + dy * dy


def select_target(
    moles: List[DetectedMole],
    locked_center: Optional[Tuple[int, int]],
    lock_enabled: bool,
    max_distance_px: float,
) -> Optional[DetectedMole]:
    if not moles:
        return None
    if not lock_enabled or locked_center is None:
        return moles[0]

    closest = min(moles, key=lambda mole: _distance_sq(mole.center, locked_center))
    if _distance_sq(closest.center, locked_center) <= max_distance_px * max_distance_px:
        return closest
    return None


def draw_moles(
    frame,
    moles: List[DetectedMole],
    selected: Optional[DetectedMole],
    args: argparse.Namespace,
) -> None:
    selected_center = selected.center if selected is not None else None
    for index, mole in enumerate(moles[:8], start=1):
        cx, cy = mole.center
        is_selected = selected_center == mole.center
        color = (0, 255, 255) if is_selected else (0, 170, 255)
        thickness = 2 if is_selected else 1
        cv2.circle(frame, (cx, cy), max(mole.radius + 3, 5), color, thickness)
        cv2.circle(frame, (cx, cy), 2, (0, 0, 255), -1)
        if args.show_mole_labels:
            cv2.putText(
                frame,
                "#{} ({},{})".format(index, cx, cy),
                (cx + 6, max(14, cy - 5)),
                cv2.FONT_HERSHEY_SIMPLEX,
                max(0.25, float(args.overlay_scale)),
                color,
                max(1, int(args.overlay_thickness)),
            )


def draw_follow_overlay(
    frame,
    state: str,
    fps: float,
    moles: List[DetectedMole],
    selected: Optional[DetectedMole],
    roi: Optional[Tuple[int, int, int, int]],
    locked_center: Optional[Tuple[int, int]],
    stable: Optional[Tuple[int, int]],
    dcx: Optional[int],
    dcy: Optional[int],
    sent_count: int,
    args: argparse.Namespace,
) -> None:
    h, w = frame.shape[:2]
    center = frame_center((w, h))
    target_center = stable if stable is not None else (
        selected.center if selected is not None else None
    )

    cv2.drawMarker(frame, center, (255, 255, 255), cv2.MARKER_CROSS, 22, 2)
    cv2.circle(frame, center, max(args.deadband_px, 3), (255, 160, 0), 1)
    cv2.drawMarker(frame, center, (255, 160, 0), cv2.MARKER_TILTED_CROSS, 18, 2)

    if roi is not None and args.show_roi:
        rx, ry, rw, rh = roi
        cv2.rectangle(frame, (rx, ry), (rx + rw - 1, ry + rh - 1), (255, 160, 0), 1)

    if locked_center is not None:
        cv2.circle(frame, locked_center, 8, (255, 0, 255), 1)
        cv2.drawMarker(frame, locked_center, (255, 0, 255), cv2.MARKER_CROSS, 10, 1)

    if stable is not None:
        cv2.circle(frame, stable, 10, (0, 255, 255), 2)
        cv2.line(frame, center, stable, (0, 255, 255), 1)

    draw_moles(frame, moles, selected, args)

    err_text = "--"
    if dcx is not None and dcy is not None:
        err_text = "dcx={} dcy={}".format(dcx, dcy)
    if args.show_debug_text:
        lines = [
            "STATE {} | FPS {:.1f} | MOLES {}".format(state, fps, len(moles)),
            "{} | sent={} | rate={:.1f}Hz".format(err_text, sent_count, args.send_rate_hz),
            "frame_center={} | target_center={}".format(center, target_center),
            "lock_center={}".format(locked_center),
            "deadband={}px | raw image center mode".format(args.deadband_px),
        ]
    else:
        lines = [
            "{} FPS {:.1f} MOLES {} {}".format(state, fps, len(moles), err_text),
        ]

    y = 16
    scale = max(0.2, float(args.overlay_scale))
    text_thickness = max(1, int(args.overlay_thickness))
    outline_thickness = text_thickness + 1
    for line in lines:
        cv2.putText(frame, line, (8, y), cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), outline_thickness)
        cv2.putText(frame, line, (8, y), cv2.FONT_HERSHEY_SIMPLEX, scale, (80, 255, 80), text_thickness)
        y += int(18 * scale + 8)


def main() -> int:
    args = parse_args()
    params = apply_param_overrides(load_params(args.params_json), args)
    try:
        reference_detector = load_reference_detector(args)
    except RuntimeError as exc:
        print("ERROR: {}".format(exc), flush=True)
        return 2

    if reference_detector is None:
        print(
            (
                "spot detector: area={}..{} circularity>={:.2f} "
                "aspect<={:.2f} contrast>={} skin_mask={}"
            ).format(
                params.min_area,
                params.max_area,
                params.min_circularity,
                params.max_aspect_ratio,
                params.min_contrast,
                int(params.use_skin_mask),
            ),
            flush=True,
        )

    cap, label = open_camera(args.camera, args.width, args.height, args.fps)
    print("camera opened: {}".format(label), flush=True)

    undistorter = None
    if args.calib_params:
        undistorter = Undistorter.load(args.calib_params, (args.width, args.height))
        if undistorter is not None:
            print("undistort enabled: {}".format(args.calib_params), flush=True)

    sender = open_serial(args)
    serial_send_enabled = sender is not None and bool(args.send_serial)
    target_ctrl_started = False
    if serial_send_enabled and args.start_target_ctrl:
        target_ctrl_started = sender.send_target_ctrl(True)
        if not target_ctrl_started:
            print("target ctrl start failed: {}".format(sender.last_error), flush=True)

    visual_only = not serial_send_enabled
    follow_enabled = serial_send_enabled and not bool(args.wait_ready)
    waiting_ready = serial_send_enabled and bool(args.wait_ready)
    ready_started = time.monotonic() if waiting_ready else None
    state = STATE_VISUAL_ONLY if visual_only else (
        STATE_WAIT_READY if waiting_ready else STATE_FOLLOW
    )

    if waiting_ready:
        print("waiting RA6M5 READY before visual follow", flush=True)
    elif visual_only:
        print("visual only: not sending AA 01 01 BB or VISION_ERROR", flush=True)
    elif sender is not None:
        print("visual follow starts without READY wait", flush=True)

    history = deque(maxlen=max(1, args.stable_window))  # type: Deque[Optional[Tuple[int, int]]]
    win_name = "Mole Visual Follow"
    mouse_lock = {"pending": None, "clear": False, "active": False}
    if args.display:
        cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)
        if args.manual_lock:
            def on_mouse(event, x, y, _flags, _param) -> None:
                if event == cv2.EVENT_LBUTTONDOWN:
                    mouse_lock["pending"] = (int(x), int(y))
                    print("manual target lock requested: ({}, {})".format(x, y), flush=True)
                elif event == cv2.EVENT_RBUTTONDOWN:
                    mouse_lock["clear"] = True
                    print("manual target lock cleared", flush=True)

            cv2.setMouseCallback(win_name, on_mouse)

    count = 0
    sent_count = 0
    start_time = time.monotonic()
    last_print = 0.0
    last_send = 0.0
    last_serial_error_print = 0.0
    min_send_period = wait_period(float(args.send_rate_hz))
    last_dcx = None  # type: Optional[int]
    last_dcy = None  # type: Optional[int]
    locked_center = None  # type: Optional[Tuple[int, int]]
    lock_lost_count = 0

    try:
        while True:
            ok, frame = cap.read()
            if not ok or frame is None:
                time.sleep(0.01)
                continue

            frame = apply_flip(frame, args.flip)
            if undistorter is not None:
                frame = undistorter.apply(frame)

            now = time.monotonic()
            if sender is not None:
                for status in sender.poll_statuses():
                    print(
                        "RA6M5 status {} func=0x{:02X} value=0x{:02X}".format(
                            status.label,
                            status.func,
                            status.value,
                        ),
                        flush=True,
                    )
                    if not serial_send_enabled:
                        continue
                    if status.func == RA6_TO_JETSON_READY and status.value == 0x01:
                        follow_enabled = True
                        waiting_ready = False
                        state = STATE_FOLLOW
                        print("READY received: visual follow enabled", flush=True)
                    elif status.func == RA6_TO_JETSON_ERROR and status.value == 0x01:
                        follow_enabled = False
                        waiting_ready = False
                        state = STATE_RA6_ERROR
                        print("RA6M5 ERROR received: visual follow disabled", flush=True)
                    elif status.func == RA6_TO_JETSON_TARGET_CTRL and status.value == 0x01:
                        if waiting_ready:
                            state = STATE_WAIT_READY

            if waiting_ready and args.ready_timeout_ms > 0 and ready_started is not None:
                elapsed_ms = (now - ready_started) * 1000.0
                if elapsed_ms >= args.ready_timeout_ms:
                    waiting_ready = False
                    follow_enabled = False
                    state = STATE_PREVIEW
                    print("READY timeout: visual follow disabled", flush=True)

            if mouse_lock["clear"]:
                locked_center = None
                lock_lost_count = 0
                history.clear()
                mouse_lock["active"] = False
                mouse_lock["clear"] = False
            if mouse_lock["pending"] is not None:
                locked_center = mouse_lock["pending"]  # type: ignore[assignment]
                lock_lost_count = 0
                history.clear()
                mouse_lock["active"] = True
                mouse_lock["pending"] = None

            roi = detection_roi(frame.shape, args)
            if mouse_lock["active"] and locked_center is not None:
                roi = center_roi(frame.shape, locked_center, args.manual_lock_radius_px)
            alignment = None
            alignment_failed = False
            if reference_detector is not None:
                try:
                    moles, alignment, reference_debug = reference_detector.detect(
                        frame,
                        return_debug=True,
                    )
                except ValueError as exc:
                    print("ERROR: reference detector failed: {}".format(exc), flush=True)
                    return 2

                frame = reference_debug["aligned"]
                alignment_failed = not alignment.ok
                if alignment_failed:
                    moles = []
                else:
                    moles = filter_moles_by_roi(moles, roi)
            else:
                moles = detect_moles(frame, params=params, roi=roi)
            lock_active = bool(args.lock_target) or bool(mouse_lock["active"])
            lock_distance = (
                float(args.manual_lock_radius_px)
                if mouse_lock["active"]
                else float(args.lock_max_distance_px)
            )
            selected = select_target(
                moles,
                locked_center,
                lock_active,
                max(1.0, lock_distance),
            )
            primary = selected.center if selected is not None else None
            current_target_center = primary  # type: Optional[Tuple[int, int]]
            if lock_active:
                if selected is not None:
                    locked_center = selected.center
                    lock_lost_count = 0
                elif locked_center is not None:
                    lock_lost_count += 1
                    if (
                        lock_lost_count >= max(1, int(args.lock_lost_frames))
                        and not mouse_lock["active"]
                    ):
                        locked_center = None
                        lock_lost_count = 0

            history.append(primary)
            stable = stable_center(history, args.stable_required, args.stable_radius)
            if stable is not None:
                current_target_center = stable

            count += 1
            fps = count / max(1e-6, now - start_time)
            dcx = None  # type: Optional[int]
            dcy = None  # type: Optional[int]

            if alignment_failed:
                state = STATE_ALIGN_FAILED if follow_enabled else (
                    STATE_VISUAL_ONLY if visual_only else state
                )
                last_dcx = None
                last_dcy = None
            elif not moles:
                state = STATE_NO_TARGET if follow_enabled else (
                    STATE_VISUAL_ONLY if visual_only else state
                )
                last_dcx = None
                last_dcy = None
            elif selected is None:
                state = STATE_UNSTABLE if follow_enabled else (
                    STATE_VISUAL_ONLY if visual_only else state
                )
                last_dcx = None
                last_dcy = None
            elif stable is None:
                state = STATE_UNSTABLE if follow_enabled else (
                    STATE_VISUAL_ONLY if visual_only else state
                )
                last_dcx = None
                last_dcy = None
            else:
                dcx, dcy = target_error(
                    stable,
                    (frame.shape[1], frame.shape[0]),
                )
                last_dcx, last_dcy = dcx, dcy

                in_deadband = abs(dcx) <= args.deadband_px and abs(dcy) <= args.deadband_px
                too_large = (
                    args.max_error_px > 0
                    and (abs(dcx) > args.max_error_px or abs(dcy) > args.max_error_px)
                )

                if follow_enabled:
                    if too_large:
                        state = STATE_ERROR_TOO_LARGE
                    elif in_deadband:
                        state = STATE_ALIGNED
                    else:
                        state = STATE_FOLLOW

                can_send = (
                    serial_send_enabled
                    and follow_enabled
                    and not too_large
                    and now - last_send >= min_send_period
                )
                if can_send:
                    send_dcx, send_dcy = dcx, dcy
                    if in_deadband:
                        if not args.zero_in_deadband:
                            send_dcx = None
                            send_dcy = None
                        else:
                            send_dcx = 0
                            send_dcy = 0

                    if send_dcx is not None and send_dcy is not None:
                        if sender.send_vision_error(send_dcx, send_dcy):
                            last_send = now
                            sent_count += 1
                        elif now - last_serial_error_print >= args.print_interval:
                            last_serial_error_print = now
                            print("serial send failed: {}".format(sender.last_error), flush=True)

            if now - last_print >= args.print_interval:
                last_print = now
                err_text = "--"
                if last_dcx is not None and last_dcy is not None:
                    err_text = "dcx={} dcy={}".format(last_dcx, last_dcy)
                center = frame_center((frame.shape[1], frame.shape[0]))
                align_text = ""
                if alignment_failed and alignment is not None:
                    align_text = " align={}".format(alignment.reason)
                print(
                    (
                        "fps={:.1f} state={} moles={} frame_center={} "
                        "target_center={} stable={} {} sent={}{}"
                    ).format(
                        fps,
                        state,
                        len(moles),
                        center,
                        current_target_center,
                        stable,
                        err_text,
                        sent_count,
                        align_text,
                    ),
                    flush=True,
                )

            if args.display:
                draw_follow_overlay(
                    frame,
                    state,
                    fps,
                    moles,
                    selected,
                    roi,
                    locked_center,
                    stable,
                    dcx,
                    dcy,
                    sent_count,
                    args,
                )
                cv2.imshow(win_name, frame)
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q") or key == 27:
                    break

    except KeyboardInterrupt:
        print("\nstopped by user", flush=True)
    finally:
        cap.release()
        if sender is not None:
            if target_ctrl_started and args.stop_target_ctrl_on_exit:
                sender.send_target_ctrl(False)
            sender.close()
        if args.display:
            cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
