import argparse
import json
import math
import os
import sys
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from threading import Lock
from typing import Deque, Dict, List, Optional, Tuple

from PyQt5.QtCore import QDateTime, QProcess, QSize, QThread, QTimer, Qt, pyqtSignal
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import (
    QApplication,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QListWidget,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QComboBox,
    QScrollArea,
    QScroller,
    QSizePolicy,
    QStackedWidget,
    QVBoxLayout,
    QWidget,
)

from .arm_controller import ArmController, ArmTarget


RA6_READY_FUNC = 0x01
RA6_ALIGN_DONE_FUNC = 0x02
RA6_OUTPUT_FUNC = 0x03
RA6_TARGET_CTRL_FUNC = 0x04
RA6_HEARTBEAT_FUNC = 0x05
RA6_SAFE_DISTANCE_FUNC = 0x06
RA6_VISION_STATE_FUNC = 0x07
RA6_ERROR_FUNC = 0xFE
RA6_CAPTURE_POINT_FUNC = 0x10
RA6_CAPTURE_HOME_FUNC = 0x11
RA6_TARGET_PRESTART_FUNC = 0x12
RA6_WORKFLOW_FUNC = 0x20
RA6_COMMAND_ACK_FUNC = 0x21
RA6_VALUE_ON = 0x01
RA6_VALUE_OFF = 0x00

WORKFLOW_IDLE = 0x00
WORKFLOW_START_ACCEPTED = 0x01
WORKFLOW_MEASURE_POSITION_READY = 0x02
WORKFLOW_DISTANCE_SAFE_LATCHED = 0x03
WORKFLOW_RETREAT_DONE_WAIT_RESTART = 0x04
WORKFLOW_RETURN_HOME_DONE = 0x05
WORKFLOW_ABORTED_HOLD = 0x06
WORKFLOW_FAULT_HOLD = 0x07

ERROR_SAFE_DISTANCE_TOO_CLOSE = 0x09
ERROR_VISION_LOST = 0x0A
ERROR_SEQ_CONFLICT = 0x0D
ERROR_SAFE_DISTANCE_TIMEOUT = 0x10

UNSUPPORTED_CONTINUOUS_SAFE_DISTANCE_MESSAGE = (
    "启动已阻止：--continuous-safe-distance 是尚未纳入正式协议的旧提案，"
    "它会请求未定义的 WORKFLOW_CTRL action=0x04。请删除该参数；"
    "当前 Qt 正式流程固定发送 action=0x01，并在安全距离锁存后停止发送 SAFE_DISTANCE。"
)

ROOT = Path(__file__).resolve().parents[2]
VISION_ROOT = ROOT / "jetson_nano_vision"
if VISION_ROOT.exists():
    sys.path.insert(0, str(VISION_ROOT))
PYORBBEC_LIB = ROOT / "pyorbbecsdk" / "install" / "lib"
if PYORBBEC_LIB.exists():
    sys.path.insert(0, str(PYORBBEC_LIB))

from jetson_vision.scene_guard import FaceSceneGuard
from jetson_vision.target_lock import TargetIdentityLock

try:
    import cv2
    import numpy as np
except Exception as exc:  # pragma: no cover - shown in UI on target machine
    cv2 = None
    np = None
    CV_IMPORT_ERROR = exc
else:
    CV_IMPORT_ERROR = None

try:
    if CV_IMPORT_ERROR is not None:
        raise CV_IMPORT_ERROR

    try:
        cv2.setNumThreads(1)
    except Exception:
        pass

    from jetson_vision.camera_utils import apply_flip, open_camera
    from jetson_vision.rgbd_distance import RgbdDistanceEstimator
    from jetson_vision.vision.facial_candidate import (
        FaceLocator,
        FacialCandidateDetector,
        FacialCandidateParams,
    )
    from jetson_vision.vision.baseline_face_library import BaselineFaceLibrary
    from jetson_vision.vision.mole_detect import (
        DetectParams,
        DetectedMole,
        build_skin_mask,
        detect_moles,
    )
    from jetson_vision.vision.target_stability import CandidateStabilityTracker
except Exception as exc:  # pragma: no cover - shown in UI on target machine
    apply_flip = None
    open_camera = None
    build_skin_mask = None
    DetectParams = None
    DetectedMole = None
    detect_moles = None
    FaceLocator = None
    FacialCandidateDetector = None
    FacialCandidateParams = None
    BaselineFaceLibrary = None
    CandidateStabilityTracker = None
    RgbdDistanceEstimator = None
    IMPORT_ERROR = exc
else:
    IMPORT_ERROR = None

if IMPORT_ERROR is None:
    try:
        from jetson_vision.vision.reference_mole_detect import (
            ReferenceDetectParams,
            ReferenceMoleDetector,
        )
    except Exception as exc:  # pragma: no cover - shown in UI on target machine
        ReferenceDetectParams = None
        ReferenceMoleDetector = None
        REFERENCE_IMPORT_ERROR = exc
    else:
        REFERENCE_IMPORT_ERROR = None
else:
    ReferenceDetectParams = None
    ReferenceMoleDetector = None
    REFERENCE_IMPORT_ERROR = IMPORT_ERROR


DEFAULT_REFERENCE_PATH = VISION_ROOT / "reference" / "clean_face.png"
DEFAULT_SAFETY_IMAGE_PATH = VISION_ROOT / "reference" / "safety_distance.png"
DEFAULT_SAFETY_REFERENCE_PATH = VISION_ROOT / "reference" / "safety_reference.json"
DEFAULT_ASTRA_YOLO_MODEL = ROOT / "models" / "arm_face_yolo" / "best.pt"
DEFAULT_FACIAL_FACE_MODEL = DEFAULT_ASTRA_YOLO_MODEL
DEFAULT_ASTRA_RGB_CALIBRATION = ROOT / "calibration" / "astra_rgb" / "rgb_calib_params.npz"
DEFAULT_FACE_CAPTURE_DIR = ROOT / "captures" / "end_camera"
MODE_PREVIEW = "预览"
MODE_DETECT_ONCE = "单次识别"
MODE_DETECTING = "连续识别"
MODE_REFERENCE_CAPTURE = "采集基准图"
MODE_SAFETY_CAPTURE = "采集安全距离图"
VISION_TRANSIENT_LOSS_GRACE_S = 0.35
VISION_TRANSIENT_INVALID_REASONS = frozenset(
    (
        "RAW_CANDIDATE_0",
        "STABILITY_PENDING",
        "TARGET_LOCK_LOST",
        "TARGET_LOCK_PENDING",
        "LOCK_SAMPLE_MISMATCH",
    )
)
SAFETY_SAFE = "safe"
SAFETY_WARN = "warn"
SAFETY_DANGER = "danger"
PAGE_WORKBENCH = 0
PAGE_BINOCULAR = 1
PAGE_END_CAMERA = 2
PAGE_TRIPTYCH = 3
PAGE_DEVICE = 4
PAGE_LOGS = 5


@dataclass
class DetectionTarget:
    x: int
    y: int
    radius: int
    area: float
    contrast: float
    circularity: float
    track_id: int = 0
    stable_count: int = 0
    required_count: int = 0
    track_age_s: float = 0.0


@dataclass
class DetectionResult:
    detected: bool
    targets: List[DetectionTarget]
    frame_width: int
    frame_height: int
    fps: float
    mode: str
    message: str = ""
    safety_level: str = SAFETY_SAFE
    safety_message: str = "安全距离正常"
    face_area_ratio: float = 0.0
    face_bbox_ratio: float = 0.0
    sample_time: float = 0.0
    # Structured end-camera diagnostics.  These fields describe the current
    # pixels before the session identity lock and never change wire data.
    raw_candidate_count: int = 0
    detector_reason: str = ""


def _classify_vision_invalid_reason(
    *,
    target_valid: bool,
    gate_reason: str,
    result: Optional[DetectionResult],
    aim_reference_configured: bool,
) -> str:
    """Return a stable terminal category for one VISION_ERROR decision."""

    if target_valid:
        return "NONE"
    if not aim_reference_configured:
        return "AIM_REFERENCE_UNCALIBRATED"

    reason = str(gate_reason or "")
    if "结果过期" in reason:
        return "RESULT_STALE"

    detector_reason = str(getattr(result, "detector_reason", "") or "")
    if detector_reason.startswith("PARTIAL_FACE"):
        return "PARTIAL_FACE"
    if detector_reason.startswith("NO_FACE"):
        return "NO_FACE"
    if detector_reason.startswith("FACE_BASELINE_PENDING"):
        return "FACE_BASELINE_PENDING"
    if detector_reason == "RAW_CANDIDATE_0":
        return "RAW_CANDIDATE_0"
    if detector_reason == "STABILITY_PENDING":
        return "STABILITY_PENDING"
    if detector_reason == "BASELINE_STATIC_FILTER":
        return "STATIC_FACE_FEATURE"
    if detector_reason == "DETECTOR_UNINITIALIZED":
        return "DETECTOR_UNINITIALIZED"
    if detector_reason == "DETECTOR_ERROR":
        return "DETECTOR_ERROR"

    if "锁定" in reason and "丢失" in reason:
        return "TARGET_LOCK_LOST"
    if any(token in reason for token in ("稳定确认", "重绑定确认", "候选尚未通过")):
        return "TARGET_LOCK_PENDING"
    if "锁定目标与最新帧不一致" in reason:
        return "LOCK_SAMPLE_MISMATCH"
    if "早于选中视图到位" in reason:
        return "RESULT_BEFORE_VIEW_READY"
    if "尚无末端识别结果" in reason:
        return "NO_RESULT"
    if "末端连续识别未运行" in reason:
        return "DETECTION_NOT_RUNNING"
    if "末端摄像头线程未运行" in reason:
        return "CAMERA_WORKER_NOT_RUNNING"
    if "未标定" in reason:
        return "AIM_REFERENCE_UNCALIBRATED"
    return "OTHER_GATE"


@dataclass
class ImageSample:
    image: QImage
    sample_time: float
    raw_image: Optional[QImage] = None


@dataclass
class TriptychDetectionResult:
    point_id: int
    image: QImage
    targets: List[DetectionTarget]
    message: str
    frame_width: int
    frame_height: int


@dataclass
class CameraPreview:
    layout: str
    primary: str
    raw_width: int
    raw_height: int
    left_width: int
    left_height: int
    right_width: int
    right_height: int
    left_image: Optional[QImage]
    right_image: Optional[QImage]
    message: str
    sample_time: float = 0.0


@dataclass
class DepthPreview:
    available: bool
    image: Optional[QImage]
    width: int
    height: int
    fps: float
    center_depth_mm: float
    valid_ratio: float
    message: str
    depth_mm: Optional[object] = None
    intrinsics: Optional[Tuple[float, float, float, float]] = None
    rgb_intrinsics: Optional[Tuple[float, float, float, float]] = None
    depth_to_rgb_rotation: Optional[object] = None
    depth_to_rgb_translation: Optional[object] = None
    sample_time: float = 0.0
    depth_calibration_size: Optional[Tuple[int, int]] = None
    rgb_calibration_size: Optional[Tuple[int, int]] = None
    depth_distortion: Optional[Tuple[float, ...]] = None
    rgb_distortion: Optional[Tuple[float, ...]] = None
    depth_distortion_mode: str = "unknown"


@dataclass
class AstraRgbPreview:
    available: bool
    image: Optional[QImage]
    width: int
    height: int
    fps: float
    message: str
    yolo_message: str = "YOLO 未运行"
    yolo_arm_detected: bool = False
    yolo_face_detected: bool = False
    yolo_detections: Optional[object] = None
    sample_time: float = 0.0


@dataclass(frozen=True)
class SafeFaceSnapshot:
    box: Tuple[float, float, float, float]
    frame_size: Tuple[int, int]
    rgb_sample_time: float
    depth_sample_time: float


@dataclass(frozen=True)
class CameraCalibration:
    intrinsics: Tuple[float, float, float, float]
    distortion: Tuple[float, ...]
    image_size: Tuple[int, int]
    rms_px: float


class VideoWorker(QThread):
    frame_ready = pyqtSignal(object)
    camera_preview_ready = pyqtSignal(object)
    result_ready = pyqtSignal(object)
    status_changed = pyqtSignal(str)
    reference_changed = pyqtSignal(bool, str)
    safety_changed = pyqtSignal(bool, str)

    def __init__(
        self,
        camera: str,
        width: int,
        height: int,
        fps: int,
        camera_brightness: float,
        camera_gain: float,
        camera_backlight_compensation: float,
        ui_fps: float,
        flip: str,
        rotate: str,
        camera_layout: str,
        stereo_primary: str,
        demo: bool,
        detector_mode: str,
        reference_path: Path,
        reference_frames: int,
        reference_warmup: int,
        reference_params,
        spot_params,
        spot_roi: Tuple[float, float, float, float],
        facial_face_model: Path,
        facial_face_conf: float,
        facial_face_imgsz: int,
        facial_face_device: str,
        facial_face_refresh_frames: int,
        facial_max_candidates: int,
        facial_stable_frames: int,
        facial_stable_seconds: float,
        facial_stable_radius: float,
        facial_enhanced_filters: bool,
        baseline_face_enabled: bool,
        baseline_face_dir: str,
        safety_enabled: bool,
        safety_image_path: Path,
        safety_reference_path: Path,
        safety_warn_face_ratio: float,
        safety_danger_face_ratio: float,
        safety_warn_scale: float,
        safety_danger_scale: float,
    ) -> None:
        super().__init__()
        self.camera = camera
        self.width = width
        self.height = height
        self.fps = fps
        self.camera_brightness = float(camera_brightness)
        self.camera_gain = float(camera_gain)
        self.camera_backlight_compensation = float(
            camera_backlight_compensation
        )
        self.ui_interval = 1.0 / max(1.0, min(60.0, float(ui_fps)))
        self.flip = flip
        self.rotate = rotate
        self.camera_layout = camera_layout
        self.stereo_primary = stereo_primary
        self.vision_width = self._expected_vision_width()
        self.vision_height = self._expected_vision_height()
        self.demo = demo
        self.detector_mode = detector_mode
        self.reference_path = reference_path
        self.reference_frames = max(3, int(reference_frames))
        self.reference_warmup = max(0, int(reference_warmup))
        self.reference_params = reference_params
        self.spot_params = spot_params
        self.spot_roi = spot_roi
        self.facial_face_model = Path(facial_face_model)
        self.facial_face_conf = max(0.01, min(1.0, float(facial_face_conf)))
        self.facial_face_imgsz = max(160, int(facial_face_imgsz))
        self.facial_face_device = str(facial_face_device)
        self.facial_face_refresh_frames = max(1, int(facial_face_refresh_frames))
        self.facial_max_candidates = max(1, int(facial_max_candidates))
        self.facial_stable_frames = max(1, int(facial_stable_frames))
        self.facial_stable_seconds = max(0.0, float(facial_stable_seconds))
        self.facial_stable_radius = max(0.5, float(facial_stable_radius))
        self.facial_enhanced_filters = bool(facial_enhanced_filters)
        self.baseline_face_enabled = bool(baseline_face_enabled)
        self.baseline_face_dir = str(baseline_face_dir)
        self.baseline_face_view = "front"
        self._baseline_face_library = None
        if self.baseline_face_enabled and BaselineFaceLibrary is not None:
            try:
                self._baseline_face_library = BaselineFaceLibrary(
                    self.baseline_face_dir,
                    enabled=True,
                )
            except Exception:
                self._baseline_face_library = None
        self.safety_enabled = bool(safety_enabled)
        self.safety_image_path = safety_image_path
        self.safety_reference_path = safety_reference_path
        self.safety_warn_face_ratio = max(0.0, float(safety_warn_face_ratio))
        self.safety_danger_face_ratio = max(
            self.safety_warn_face_ratio,
            float(safety_danger_face_ratio),
        )
        self.safety_danger_scale = max(0.1, float(safety_danger_scale))
        self.safety_warn_scale = max(self.safety_danger_scale, float(safety_warn_scale))
        self._safety_reference_area_ratio = 0.0
        self._safety_reference_bbox_ratio = 0.0
        self._safety_reference_loaded = False
        self._reference_detector = None
        self._facial_detector = None
        self._last_raw_candidate_count = 0
        self._last_detector_reason = ""
        self._facial_tracker = (
            CandidateStabilityTracker(
                required_hits=self.facial_stable_frames,
                history_size=max(7, self.facial_stable_frames + 2),
                stable_radius=self.facial_stable_radius,
                min_stable_duration_s=self.facial_stable_seconds,
                # The end camera can miss a small mole for several inference
                # frames while the arm is moving.  Retain the identity for at
                # most 1.5 seconds (45 frames is only a secondary hard cap),
                # but still clear its stability history so VISION_ERROR stays
                # valid=0 until 4 frames + the elapsed-time gate reconfirm it.
                max_missed=45,
                max_gap_s=1.5,
                max_radius_ratio=1.5,
                # A fully confirmed target that is absent for less than
                # 450 ms may re-lock after two fresh, face-relative matches.
                # Missing frames themselves still publish no coordinate.
                fast_reacquire_hits=2,
                fast_reacquire_min_duration_s=0.10,
                fast_reacquire_max_gap_s=0.45,
            )
            if CandidateStabilityTracker is not None
            else None
        )
        self._reference_message = "未采集基准图"
        self._reference_capture_active = False
        self._reference_capture_warmup_left = 0
        self._reference_capture_frames = []
        self._safety_capture_active = False
        self._safety_capture_warmup_left = 0
        self._safety_capture_frames = []
        self._running = True
        self._continuous = False
        self._detect_once = False
        self._detect_once_frames_left = 0
        # Once the doctor-selected treatment target has acquired a stable
        # tracker id, keep the public preview focused on that one identity.
        # Other candidates still reach the session identity lock so nearby
        # ambiguity remains fail-closed, but they must never look like newly
        # selectable treatment targets while the arm is approaching.
        self._display_track_id = 0
        self._lock = Lock()
        self._load_safety_reference()

    def set_baseline_face_view(self, view: str) -> None:
        self.baseline_face_view = str(view or "front")

    def set_display_track_id(self, track_id: int) -> None:
        """Limit the rendered treatment overlay to one confirmed track id."""

        with self._lock:
            self._display_track_id = max(0, int(track_id))

    def _candidates_for_display(self, candidates):
        items = list(candidates)
        lock = self.__dict__.get("_lock")
        if lock is None:
            display_track_id = int(self.__dict__.get("_display_track_id", 0))
        else:
            with lock:
                display_track_id = int(
                    self.__dict__.get("_display_track_id", 0)
                )
        if display_track_id <= 0:
            return items
        return [
            item
            for item in items
            if int(getattr(item, "track_id", 0)) == display_track_id
        ]

    def _is_stereo_enabled(self) -> bool:
        return self.camera_layout == "side-by-side"

    def _expected_vision_width(self) -> int:
        base_width = int(self.width)
        if self.camera_layout == "side-by-side":
            base_width = max(1, int(self.width) // 2)
        if self.rotate in ("90cw", "90ccw"):
            return int(self.height)
        return base_width

    def _expected_vision_height(self) -> int:
        base_height = int(self.height)
        if self.rotate in ("90cw", "90ccw"):
            if self.camera_layout == "side-by-side":
                return max(1, int(self.width) // 2)
            return int(self.width)
        return base_height

    def stop(self) -> None:
        self._running = False

    def start_detection(self) -> None:
        with self._lock:
            already_continuous = self._continuous
            self._continuous = True
            self._detect_once = False
            self._detect_once_frames_left = 0
            if not already_continuous:
                self._display_track_id = 0
                if self._facial_detector is not None:
                    self._facial_detector.reset()
                if self._facial_tracker is not None:
                    self._facial_tracker.reset()

    def stop_detection(self) -> None:
        with self._lock:
            self._continuous = False
            self._detect_once = False
            self._detect_once_frames_left = 0

    def request_detect_once(self) -> None:
        with self._lock:
            self._detect_once = True
            expected_interval = max(
                self.ui_interval,
                1.0 / max(1.0, float(self.fps)),
            )
            duration_frames = int(
                math.ceil(self.facial_stable_seconds / expected_interval)
            ) + 1
            self._detect_once_frames_left = max(
                self.facial_stable_frames,
                duration_frames,
            )
            if self._facial_detector is not None:
                self._facial_detector.reset()
            if self._facial_tracker is not None:
                self._facial_tracker.reset()

    def request_reference_capture(self) -> None:
        with self._lock:
            self._continuous = False
            self._detect_once = False
            self._detect_once_frames_left = 0
            self._reference_capture_active = True
            self._reference_capture_warmup_left = self.reference_warmup
            self._reference_capture_frames = []

    def cancel_reference_capture(self) -> None:
        with self._lock:
            self._reference_capture_active = False
            self._reference_capture_warmup_left = 0
            self._reference_capture_frames = []

    def request_safety_capture(self) -> None:
        with self._lock:
            self._continuous = False
            self._detect_once = False
            self._detect_once_frames_left = 0
            self._safety_capture_active = True
            self._safety_capture_warmup_left = self.reference_warmup
            self._safety_capture_frames = []

    def cancel_safety_capture(self) -> None:
        with self._lock:
            self._safety_capture_active = False
            self._safety_capture_warmup_left = 0
            self._safety_capture_frames = []

    def _has_forced_frame_work(self) -> bool:
        with self._lock:
            return (
                self._detect_once
                or self._reference_capture_active
                or self._safety_capture_active
            )

    def _read_due_camera_frame(self, cap, forced_work: bool, next_emit: float):
        """Read one fresh frame without consuming an extra frame at a deadline.

        ``VideoCapture.read()`` performs both ``grab`` and ``retrieve``.  The
        old throttling path first called ``grab()`` while waiting and then
        called ``read()`` once the UI deadline was due.  At 30 FPS with a
        20 FPS UI limit that consumed roughly three camera frames for each
        published frame and reduced the observed rate to about 10 FPS.

        Grab exactly one incoming frame, re-check the deadline after the
        blocking grab, and retrieve that same frame when it is due.  Forced
        capture operations still use ``read()`` on every iteration.
        """

        if forced_work or next_emit <= 0.0:
            ok, frame = cap.read()
            return bool(ok), frame, False

        try:
            grabbed = bool(cap.grab())
        except Exception:
            grabbed = False
        if not grabbed:
            return False, None, False
        if time.monotonic() < float(next_emit):
            return True, None, True
        try:
            ok, frame = cap.retrieve()
        except Exception:
            return False, None, False
        return bool(ok), frame, False

    def _advance_ui_deadline(self, current_deadline: float, sample_time: float) -> float:
        """Advance a fixed-rate UI schedule without locking to camera frames.

        A 30 FPS camera cannot provide frames exactly every 50 ms.  Moving the
        deadline to ``last_emit + 50 ms`` after each published frame therefore
        selects every second input frame and caps a requested 20 FPS preview at
        roughly 15 FPS.  Keep the deadline on its own fixed timeline instead;
        the selected 30 FPS frames then alternate as needed and average close
        to the requested UI rate.
        """

        now = float(sample_time)
        interval = max(1e-3, float(self.ui_interval))
        if current_deadline <= 0.0:
            return now + interval

        deadline = float(current_deadline) + interval
        if deadline <= now:
            missed = math.floor((now - deadline) / interval) + 1
            deadline += float(missed) * interval
        return deadline

    def _build_image_sample(self, shown, raw_frame, sample_time: float) -> ImageSample:
        shown_image = self._to_qimage(shown)
        # In plain preview ``shown`` is the untouched camera frame.  QImage is
        # implicitly shared and the receiver never mutates it, so reuse the
        # conversion for the raw capture cache.  A privacy mask or overlay
        # still receives a separate raw QImage.
        raw_image = (
            shown_image
            if shown is raw_frame
            else self._to_qimage(raw_frame)
        )
        return ImageSample(
            image=shown_image,
            sample_time=float(sample_time),
            raw_image=raw_image,
        )

    def run(self) -> None:
        if IMPORT_ERROR is not None and not self.demo:
            self.status_changed.emit(f"视觉依赖加载失败：{IMPORT_ERROR}")
            return

        cap = None
        label = "模拟画面"
        try:
            if self.demo:
                self.status_changed.emit("演示模式：未打开真实摄像头")
                self._load_reference_detector()
            else:
                cap, label = open_camera(
                    self.camera,
                    self.width,
                    self.height,
                    self.fps,
                    brightness=self.camera_brightness,
                    gain=self.camera_gain,
                    backlight_compensation=self.camera_backlight_compensation,
                )
                print("[END_CAMERA] {}".format(label), flush=True)
                self.status_changed.emit(f"摄像头已打开：{label}")
                self._load_reference_detector()

            fps_meter = 0.0
            last_ui_emit = 0.0
            next_ui_emit = 0.0
            last_result_emit = 0.0
            while self._running:
                now = time.monotonic()
                forced_work = self._has_forced_frame_work()
                if self.demo:
                    frame = self._demo_frame()
                    ok = True
                    time.sleep(max(0.005, 1.0 / max(self.fps, 1)))
                else:
                    ok, frame, skipped = self._read_due_camera_frame(
                        cap,
                        forced_work,
                        next_ui_emit,
                    )
                    if skipped:
                        time.sleep(0.001)
                        continue

                if not ok or frame is None:
                    time.sleep(0.03)
                    continue

                now = time.monotonic()
                if not forced_work and next_ui_emit > 0.0 and now < next_ui_emit:
                    time.sleep(0.002)
                    continue

                frame, left_frame, right_frame, camera_message = self._prepare_camera_views(frame)

                safety_level = SAFETY_SAFE
                safety_message = "安全检测未启用"
                face_area_ratio = 0.0
                face_bbox_ratio = 0.0
                if self.safety_enabled or forced_work:
                    safety_level, safety_message, face_area_ratio, face_bbox_ratio = (
                        self._evaluate_safety(frame)
                    )
                display_frame = frame
                targets_already_drawn = False
                targets: List[DetectionTarget] = []
                message = ""
                raw_candidate_count = 0
                detector_reason = ""
                safety_capture_message = self._consume_safety_capture(frame)
                if safety_capture_message is not None:
                    mode = MODE_SAFETY_CAPTURE
                    message = safety_capture_message
                else:
                    reference_message = self._consume_reference_capture(frame)
                    if reference_message is not None:
                        mode = MODE_REFERENCE_CAPTURE
                        message = reference_message
                    else:
                        should_detect, mode = self._consume_detection_mode()
                        if should_detect:
                            targets, message, detected_frame = self._detect(
                                frame,
                                sample_time=now,
                            )
                            raw_candidate_count = int(
                                getattr(self, "_last_raw_candidate_count", 0)
                            )
                            detector_reason = str(
                                getattr(self, "_last_detector_reason", "") or ""
                            )
                            if detected_frame is not None:
                                display_frame = detected_frame
                                targets_already_drawn = (
                                    self.detector_mode == "facial_candidate"
                                )

                force_emit = mode in (
                    MODE_DETECT_ONCE,
                    MODE_REFERENCE_CAPTURE,
                    MODE_SAFETY_CAPTURE,
                )
                if not force_emit and next_ui_emit > 0.0 and now < next_ui_emit:
                    continue
                emit_dt = now - last_ui_emit if last_ui_emit > 0.0 else self.ui_interval
                last_ui_emit = now
                if force_emit:
                    next_ui_emit = now + self.ui_interval
                else:
                    next_ui_emit = self._advance_ui_deadline(next_ui_emit, now)
                fps_sample = 1.0 / max(emit_dt, 1e-3)
                fps_limit = 1.0 / max(self.ui_interval, 1e-3)
                fps_sample = min(fps_sample, fps_limit)
                fps_meter = 0.8 * fps_meter + 0.2 * fps_sample if fps_meter else fps_sample

                if mode == MODE_PREVIEW and safety_level == SAFETY_SAFE and not targets:
                    shown = display_frame
                else:
                    shown = self._draw_overlay(
                        display_frame,
                        targets,
                        mode,
                        safety_level,
                        safety_message,
                        draw_targets=not targets_already_drawn,
                    )
                self._emit_camera_preview(left_frame, right_frame, camera_message, now)
                self.frame_ready.emit(
                    self._build_image_sample(shown, frame, now)
                )
                if (
                    mode != MODE_PREVIEW
                    or self.safety_enabled
                    or now - last_result_emit >= 0.5
                ):
                    last_result_emit = now
                    self.result_ready.emit(
                        DetectionResult(
                            detected=bool(targets),
                            targets=targets,
                            frame_width=display_frame.shape[1],
                            frame_height=display_frame.shape[0],
                            fps=fps_meter,
                            mode=mode,
                            message=message,
                            safety_level=safety_level,
                            safety_message=safety_message,
                            sample_time=now,
                            face_area_ratio=face_area_ratio,
                            face_bbox_ratio=face_bbox_ratio,
                            raw_candidate_count=raw_candidate_count,
                            detector_reason=detector_reason,
                        )
                    )
        except Exception as exc:
            self.status_changed.emit(f"运行错误：{exc}")
        finally:
            if cap is not None:
                cap.release()

    def _prepare_camera_views(self, raw_frame):
        if self._is_stereo_enabled():
            left_frame, right_frame = self._split_side_by_side(raw_frame)
            left_frame = self._apply_configured_flip(left_frame)
            right_frame = self._apply_configured_flip(right_frame)
            left_frame = self._apply_configured_rotation(left_frame)
            right_frame = self._apply_configured_rotation(right_frame)
            if self.stereo_primary == "right":
                frame = right_frame
                primary_text = "右目"
            else:
                frame = left_frame
                primary_text = "左目"
            message = "双目左右拼接，主视图：{}".format(primary_text)
            return frame, left_frame, right_frame, message

        frame = self._apply_configured_flip(raw_frame)
        frame = self._apply_configured_rotation(frame)
        if self.camera_layout == "astra-rgbd":
            return frame, frame, None, "Astra Pro Plus RGB 主视图"
        return frame, frame, None, "单目主视图"

    def _apply_configured_flip(self, frame):
        if self.flip != "none" and apply_flip is not None:
            return apply_flip(frame, self.flip)
        return frame

    def _apply_configured_rotation(self, frame):
        if self.rotate == "none":
            return frame
        if cv2 is None:
            return frame
        if self.rotate == "90cw":
            return cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
        if self.rotate == "90ccw":
            return cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
        if self.rotate == "180":
            return cv2.rotate(frame, cv2.ROTATE_180)
        return frame

    def _split_side_by_side(self, frame):
        height, width = frame.shape[:2]
        half = max(1, width // 2)
        left = frame[:, :half].copy()
        right = frame[:, half:].copy()
        if right.size == 0:
            right = left.copy()
        del height
        return left, right

    def _emit_camera_preview(
        self,
        left_frame,
        right_frame,
        message: str,
        sample_time: float,
    ) -> None:
        left_h, left_w = left_frame.shape[:2]
        include_preview_images = self._is_stereo_enabled()
        if right_frame is None:
            right_h, right_w = 0, 0
            right_image = None
        else:
            right_h, right_w = right_frame.shape[:2]
            right_image = self._to_qimage(right_frame) if include_preview_images else None

        preview = CameraPreview(
            layout=self.camera_layout,
            primary=self.stereo_primary if self._is_stereo_enabled() else "mono",
            raw_width=left_w + right_w,
            raw_height=max(left_h, right_h),
            left_width=left_w,
            left_height=left_h,
            right_width=right_w,
            right_height=right_h,
            left_image=self._to_qimage(left_frame) if include_preview_images else None,
            right_image=right_image,
            message=message,
            sample_time=sample_time,
        )
        self.camera_preview_ready.emit(preview)

    def _consume_detection_mode(self) -> Tuple[bool, str]:
        with self._lock:
            if self._detect_once:
                self._detect_once_frames_left -= 1
                if self._detect_once_frames_left <= 0:
                    self._detect_once = False
                    self._detect_once_frames_left = 0
                return True, MODE_DETECT_ONCE
            if self._continuous:
                return True, MODE_DETECTING
            return False, MODE_PREVIEW

    def _evaluate_safety(self, frame) -> Tuple[str, str, float, float]:
        with self._lock:
            reference_capture_active = self._reference_capture_active
            safety_capture_active = self._safety_capture_active
        if reference_capture_active:
            return SAFETY_SAFE, "正在采集基准图", 0.0, 0.0
        if safety_capture_active:
            return SAFETY_SAFE, "正在采集安全距离图", 0.0, 0.0

        if not self.safety_enabled:
            return SAFETY_SAFE, "安全检测未启用", 0.0, 0.0
        measured, face_area_ratio, face_bbox_ratio, reason = self._measure_face_ratio(frame)
        if not measured:
            level = SAFETY_WARN if reason.startswith("安全检测异常") else SAFETY_SAFE
            return level, reason, face_area_ratio, face_bbox_ratio

        if self._safety_reference_loaded:
            warn_area_ratio = self._safety_reference_area_ratio * self.safety_warn_scale
            danger_area_ratio = self._safety_reference_area_ratio * self.safety_danger_scale
            warn_bbox_ratio = self._safety_reference_bbox_ratio * self.safety_warn_scale
            danger_bbox_ratio = self._safety_reference_bbox_ratio * self.safety_danger_scale
            if face_area_ratio < danger_area_ratio or face_bbox_ratio < danger_bbox_ratio:
                return (
                    SAFETY_DANGER,
                    "危险：人脸距离过近，已禁用预定位和点痣操作",
                    face_area_ratio,
                    face_bbox_ratio,
                )

            if face_area_ratio <= warn_area_ratio or face_bbox_ratio <= warn_bbox_ratio:
                return (
                    SAFETY_WARN,
                    "警告：正在接近安全距离图标定位置",
                    face_area_ratio,
                    face_bbox_ratio,
                )

            return SAFETY_SAFE, "安全：已远离安全距离图标定位置", face_area_ratio, face_bbox_ratio

        return SAFETY_SAFE, "安全距离未标定，请先采集安全距离图", face_area_ratio, face_bbox_ratio

    def _measure_face_ratio(self, frame) -> Tuple[bool, float, float, str]:
        if build_skin_mask is None or cv2 is None or np is None:
            return False, 0.0, 0.0, "安全检测不可用"

        try:
            mask = build_skin_mask(frame)
            count, _labels, stats, _centroids = cv2.connectedComponentsWithStats(
                mask,
                connectivity=8,
            )
        except Exception as exc:
            return False, 0.0, 0.0, "安全检测异常：{}".format(exc)

        frame_h, frame_w = frame.shape[:2]
        frame_area = max(1, int(frame_w) * int(frame_h))
        if count <= 1:
            return False, 0.0, 0.0, "未检测到人脸区域"

        largest_index = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
        largest_area = float(stats[largest_index, cv2.CC_STAT_AREA])
        width = float(stats[largest_index, cv2.CC_STAT_WIDTH])
        height = float(stats[largest_index, cv2.CC_STAT_HEIGHT])
        face_area_ratio = largest_area / float(frame_area)
        face_bbox_ratio = max(
            width / max(1.0, float(frame_w)),
            height / max(1.0, float(frame_h)),
        )
        return True, face_area_ratio, face_bbox_ratio, ""

    def _load_safety_reference(self) -> bool:
        if not self.safety_reference_path.exists():
            self._safety_reference_loaded = False
            return False

        try:
            data = json.loads(self.safety_reference_path.read_text(encoding="utf-8"))
            image_size = data.get("image_size")
            area_ratio = float(data.get("face_area_ratio", 0.0))
            bbox_ratio = float(data.get("face_bbox_ratio", 0.0))
        except Exception:
            self._safety_reference_loaded = False
            return False

        expected_size = [self.vision_width, self.vision_height]
        swapped_size = [self.vision_height, self.vision_width]
        accepts_rotated_size = (
            self.rotate in ("90cw", "90ccw")
            and image_size == swapped_size
        )
        if image_size != expected_size and not accepts_rotated_size:
            self._safety_reference_loaded = False
            return False
        if area_ratio <= 0.0 or bbox_ratio <= 0.0:
            self._safety_reference_loaded = False
            return False

        self._safety_reference_area_ratio = area_ratio
        self._safety_reference_bbox_ratio = bbox_ratio
        self._safety_reference_loaded = True
        return True

    def _consume_reference_capture(self, frame) -> Optional[str]:
        frames_to_save = None
        with self._lock:
            if not self._reference_capture_active:
                return None

            if self._reference_capture_warmup_left > 0:
                self._reference_capture_warmup_left -= 1
                return "基准图预热中，剩余 {} 帧".format(
                    self._reference_capture_warmup_left
                )

            self._reference_capture_frames.append(frame.copy())
            captured = len(self._reference_capture_frames)
            if captured >= self.reference_frames:
                frames_to_save = list(self._reference_capture_frames)
                self._reference_capture_frames = []
                self._reference_capture_active = False
            else:
                return "正在采集基准图 {}/{}".format(captured, self.reference_frames)

        try:
            safety_message = self._save_reference(frames_to_save)
        except Exception as exc:
            message = "基准图保存失败：{}".format(exc)
            self._reference_message = message
            self.reference_changed.emit(False, message)
            return message

        message = "基准图已保存并启用：{}".format(self.reference_path)
        if safety_message:
            message = "{}；{}".format(message, safety_message)
        self._reference_message = message
        self.reference_changed.emit(True, message)
        return message

    def _consume_safety_capture(self, frame) -> Optional[str]:
        frames_to_save = None
        with self._lock:
            if not self._safety_capture_active:
                return None

            if self._safety_capture_warmup_left > 0:
                self._safety_capture_warmup_left -= 1
                return "安全距离图预热中，剩余 {} 帧".format(
                    self._safety_capture_warmup_left
                )

            self._safety_capture_frames.append(frame.copy())
            captured = len(self._safety_capture_frames)
            if captured >= self.reference_frames:
                frames_to_save = list(self._safety_capture_frames)
                self._safety_capture_frames = []
                self._safety_capture_active = False
            else:
                return "正在采集安全距离图 {}/{}".format(captured, self.reference_frames)

        try:
            message = self._save_safety_distance(frames_to_save)
        except Exception as exc:
            message = "安全距离图保存失败：{}".format(exc)
            self.safety_changed.emit(False, message)
            return message

        self.safety_changed.emit(self._safety_reference_loaded, message)
        return message

    def _save_reference(self, frames) -> str:
        stack = np.stack(frames, axis=0)
        reference = np.median(stack, axis=0).astype(np.uint8)
        self.reference_path.parent.mkdir(parents=True, exist_ok=True)
        if not cv2.imwrite(str(self.reference_path), reference):
            raise RuntimeError("无法写入 {}".format(self.reference_path))
        self._load_reference_detector()
        return ""

    def _save_safety_distance(self, frames) -> str:
        stack = np.stack(frames, axis=0)
        safety_reference = np.median(stack, axis=0).astype(np.uint8)
        self.safety_image_path.parent.mkdir(parents=True, exist_ok=True)
        if not cv2.imwrite(str(self.safety_image_path), safety_reference):
            raise RuntimeError("无法写入 {}".format(self.safety_image_path))
        message = self._save_safety_reference(
            safety_reference,
            source_path=self.safety_image_path,
        )
        self.safety_enabled = self._safety_reference_loaded
        return "{}：{}".format(message, self.safety_image_path)

    def _save_safety_reference(self, reference, source_path: Path) -> str:
        measured, area_ratio, bbox_ratio, reason = self._measure_face_ratio(reference)
        if not measured:
            self._safety_reference_loaded = False
            return "安全距离未标定：{}".format(reason)

        metadata = {
            "schema_version": 1,
            "created_at_unix": time.time(),
            "image_size": [self.vision_width, self.vision_height],
            "safety_image": str(source_path),
            "face_area_ratio": area_ratio,
            "face_bbox_ratio": bbox_ratio,
            "warn_scale": self.safety_warn_scale,
            "danger_scale": self.safety_danger_scale,
            "direction": "smaller_face_ratio_is_danger",
        }
        self.safety_reference_path.parent.mkdir(parents=True, exist_ok=True)
        self.safety_reference_path.write_text(
            json.dumps(metadata, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        self._safety_reference_area_ratio = area_ratio
        self._safety_reference_bbox_ratio = bbox_ratio
        self._safety_reference_loaded = True
        return "安全距离图已保存并启用"

    def _load_reference_detector(self) -> bool:
        if self.detector_mode == "facial_candidate":
            self._reference_detector = None
            if FacialCandidateDetector is None or FaceLocator is None:
                self._facial_detector = None
                self._reference_message = "人脸候选检测器不可用"
                self.reference_changed.emit(False, self._reference_message)
                return False
            try:
                self._facial_detector = FacialCandidateDetector(
                    FaceLocator(
                        model_path=self.facial_face_model,
                        confidence=self.facial_face_conf,
                        imgsz=self.facial_face_imgsz,
                        device=self.facial_face_device,
                    ),
                    candidate_params=self.spot_params,
                    params=FacialCandidateParams(
                        max_candidates=self.facial_max_candidates,
                        face_refresh_frames=self.facial_face_refresh_frames,
                        partial_face_guard=True,
                        # The face silhouette still rejects all background.
                        # Only the eyes remain a hard anatomical safety mask;
                        # cheek/temple/nose/mouth pixels may produce candidates
                        # and are filtered by the dark-spot shape gates.
                        detection_exclusion_mode="eyes_only",
                        # Establish the selected-view full-face scale from two
                        # adjacent live frames before publishing any target.
                        face_baseline_frames=2,
                        enhanced_candidate_filters=self.facial_enhanced_filters,
                    ),
                )
            except Exception as exc:
                self._facial_detector = None
                self._reference_message = "人脸候选检测器初始化失败：{}".format(exc)
                self.reference_changed.emit(False, self._reference_message)
                return False
            self._reference_message = "人脸候选检测已启用（不需要无黑痣基准图）"
            self.reference_changed.emit(True, self._reference_message)
            return False

        if self.detector_mode == "spot":
            self._reference_detector = None
            self._facial_detector = None
            self._reference_message = "当前使用普通黑点检测"
            self.reference_changed.emit(False, self._reference_message)
            return False

        if ReferenceMoleDetector is None:
            self._reference_detector = None
            self._reference_message = "基准检测不可用：{}".format(REFERENCE_IMPORT_ERROR)
            self.reference_changed.emit(False, self._reference_message)
            return False

        if not self.reference_path.exists():
            self._reference_detector = None
            self._reference_message = "未采集基准图：{}".format(self.reference_path)
            self.reference_changed.emit(False, self._reference_message)
            return False

        reference = cv2.imread(str(self.reference_path))
        if reference is None:
            self._reference_detector = None
            self._reference_message = "基准图无法读取：{}".format(self.reference_path)
            self.reference_changed.emit(False, self._reference_message)
            return False

        reference, adapt_message = self._adapt_reference_to_current_view(reference)
        if reference.shape[1] != self.vision_width or reference.shape[0] != self.vision_height:
            self._reference_detector = None
            self._reference_message = (
                "基准图尺寸 {}x{} 与当前 {}x{} 不一致"
            ).format(reference.shape[1], reference.shape[0], self.vision_width, self.vision_height)
            self.reference_changed.emit(False, self._reference_message)
            return False

        try:
            self._reference_detector = ReferenceMoleDetector(
                reference,
                self.reference_params,
            )
        except Exception as exc:
            self._reference_detector = None
            self._reference_message = "基准检测初始化失败：{}".format(exc)
            self.reference_changed.emit(False, self._reference_message)
            return False

        self._reference_message = "基准图已启用{}：{}".format(adapt_message, self.reference_path)
        self.reference_changed.emit(True, self._reference_message)
        return True

    def _adapt_reference_to_current_view(self, reference):
        if reference.shape[1] == self.vision_width and reference.shape[0] == self.vision_height:
            return reference, ""
        if self.rotate not in ("90cw", "90ccw"):
            return reference, ""
        if reference.shape[1] != self.vision_height or reference.shape[0] != self.vision_width:
            return reference, ""

        rotated = self._apply_configured_rotation(reference)
        if rotated.shape[1] == self.vision_width and rotated.shape[0] == self.vision_height:
            return rotated, "（已按当前旋转方向适配）"
        return reference, ""

    def _detect(
        self,
        frame,
        sample_time: Optional[float] = None,
    ) -> Tuple[List[DetectionTarget], str, Optional[object]]:
        self._last_raw_candidate_count = 0
        self._last_detector_reason = ""
        if self.detector_mode == "facial_candidate":
            if self._facial_detector is None:
                self._last_detector_reason = "DETECTOR_UNINITIALIZED"
                return [], "人脸候选检测器未初始化", None
            try:
                result = self._facial_detector.detect(
                    frame,
                    render=self._facial_tracker is None,
                )
            except Exception as exc:
                self._last_detector_reason = "DETECTOR_ERROR"
                return [], "人脸候选检测失败：{}".format(exc), None
            raw_count = len(result.candidates)
            self._last_raw_candidate_count = raw_count
            worker_state = object.__getattribute__(self, "__dict__")
            baseline_library = worker_state.get("_baseline_face_library")
            if (
                raw_count > 0
                and baseline_library is not None
                and result.face is not None
            ):
                ranked_candidates = baseline_library.score_candidates(
                    frame,
                    result.candidates,
                    view=worker_state.get("baseline_face_view", "front"),
                    reject_static_dark=True,
                )
                if len(ranked_candidates) != len(result.candidates):
                    self._last_detector_reason = "BASELINE_STATIC_FILTER"
                result.candidates = ranked_candidates
                raw_count = len(result.candidates)
                self._last_raw_candidate_count = raw_count
                result = self._facial_detector.render_candidates(
                    frame,
                    result,
                    result.candidates,
                )
            if (
                raw_count == 0
                and baseline_library is not None
                and result.face is not None
            ):
                baseline_candidates = baseline_library.detect_darkening_candidates(
                    frame,
                    view=worker_state.get("baseline_face_view", "front"),
                    valid_mask=result.valid_mask,
                    min_darkening=max(18.0, float(self.spot_params.min_contrast) * 0.75),
                    min_area=max(6.0, float(self.spot_params.min_area) * 0.75),
                    max_area=float(self.spot_params.max_area),
                    max_candidates=self.facial_max_candidates,
                )
                if baseline_candidates:
                    result.candidates = baseline_candidates
                    raw_count = len(result.candidates)
                    self._last_raw_candidate_count = raw_count
                    self._last_detector_reason = "BASELINE_FALLBACK"
                    result = self._facial_detector.render_candidates(
                        frame,
                        result,
                        result.candidates,
                    )
            published_candidates = list(result.candidates)
            if self._facial_tracker is not None:
                # Associate candidates in the current face-mask coordinate
                # system as well as raw pixels.  The camera moves with the
                # arm, so the whole face can translate by more than the old
                # fixed 28 px gate while the same physical mole keeps nearly
                # the same position inside the face silhouette.
                tracking_box = None
                if result.face_mask is not None:
                    mask_points = cv2.findNonZero(result.face_mask)
                    if mask_points is not None:
                        tracking_box = cv2.boundingRect(mask_points)
                stable_candidates = self._facial_tracker.update(
                    result.candidates,
                    sample_time=sample_time,
                    reference_box=tracking_box,
                )
                published_candidates = list(stable_candidates)
                result = self._facial_detector.render_candidates(
                    frame,
                    result,
                    self._candidates_for_display(stable_candidates),
                )
            targets = [
                DetectionTarget(
                    x=int(item.center[0]),
                    y=int(item.center[1]),
                    radius=int(item.radius),
                    area=float(item.area),
                    contrast=float(item.contrast),
                    circularity=float(item.circularity),
                    track_id=int(getattr(item, "track_id", 0)),
                    stable_count=int(getattr(item, "stable_count", 0)),
                    required_count=int(getattr(item, "required_count", 0)),
                    track_age_s=float(getattr(item, "track_age_s", 0.0)),
                )
                for item in published_candidates[: self.facial_max_candidates]
            ]
            if result.face is None:
                detector_reason = str(result.message or "NO_FACE")
                if detector_reason.startswith("PARTIAL_FACE"):
                    self._last_detector_reason = "PARTIAL_FACE"
                elif detector_reason.startswith("FACE_BASELINE_PENDING"):
                    self._last_detector_reason = "FACE_BASELINE_PENDING"
                else:
                    self._last_detector_reason = "NO_FACE"
                if detector_reason.startswith("PARTIAL_FACE"):
                    message = "人脸区域不完整，已拒绝该帧并重新定位（{}）".format(
                        detector_reason
                    )
                elif detector_reason.startswith("FACE_BASELINE_PENDING"):
                    message = "正在确认选中视图的完整人脸基线（{}）".format(
                        detector_reason
                    )
                else:
                    message = "未检测到有效人脸，已禁止全图黑点识别（{}）".format(
                        detector_reason
                    )
                return [], message, result.privacy_view
            if raw_count <= 0:
                self._last_detector_reason = "RAW_CANDIDATE_0"
            elif not targets:
                self._last_detector_reason = "STABILITY_PENDING"
            return (
                targets,
                "稳定候选 {} 个（当前单帧 {} 个）".format(len(targets), raw_count),
                result.privacy_view,
            )

        if self._reference_detector is not None:
            try:
                raw_targets, alignment, debug = self._reference_detector.detect(
                    frame,
                    return_debug=True,
                )
            except Exception as exc:
                return [], "基准检测失败：{}".format(exc), None

            aligned = debug.get("aligned")
            if not alignment.ok:
                return [], "基准对齐失败：{}".format(alignment.reason), aligned

            targets = [
                DetectionTarget(
                    x=int(item.center[0]),
                    y=int(item.center[1]),
                    radius=int(item.radius),
                    area=float(item.area),
                    contrast=float(item.mean_darkening),
                    circularity=float(item.circularity),
                )
                for item in raw_targets[:8]
            ]
            if not targets:
                return [], "基准差分未发现目标；请确认基准图与当前距离/位置一致", aligned
            return targets, "基准差分检测", aligned

        if self.detector_mode == "reference":
            return [], "请先采集无黑痣基准图", None

        if detect_moles is None:
            return [], "检测函数不可用", None
        try:
            raw_targets = detect_moles(
                frame,
                params=self.spot_params,
                roi=self._spot_roi_for_frame(frame),
            )
            raw_targets = self._filter_eye_like_targets(frame, raw_targets)
        except Exception as exc:
            return [], f"检测失败：{exc}", None

        targets = [
            DetectionTarget(
                x=int(item.center[0]),
                y=int(item.center[1]),
                radius=int(item.radius),
                area=float(item.area),
                contrast=float(item.contrast),
                circularity=float(item.circularity),
            )
            for item in raw_targets[:8]
        ]
        return targets, "普通黑点检测", None

    def _spot_roi_for_frame(self, frame) -> Optional[Tuple[int, int, int, int]]:
        h, w = frame.shape[:2]
        x_min, x_max, y_min, y_max = self.spot_roi
        x0 = max(0, min(w - 1, int(round(w * x_min))))
        x1 = max(1, min(w, int(round(w * x_max))))
        y0 = max(0, min(h - 1, int(round(h * y_min))))
        y1 = max(1, min(h, int(round(h * y_max))))
        if x1 <= x0 or y1 <= y0:
            return None
        return x0, y0, x1 - x0, y1 - y0

    def _filter_eye_like_targets(self, frame, targets):
        if not targets or cv2 is None or np is None:
            return targets

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        dark = cv2.inRange(gray, 0, 75)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 5))
        dark = cv2.morphologyEx(dark, cv2.MORPH_CLOSE, kernel, iterations=1)
        contours = cv2.findContours(dark, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

        frame_h, frame_w = frame.shape[:2]
        max_eye_w = max(18, int(frame_w * 0.35))
        max_eye_h = max(8, int(frame_h * 0.20))
        eye_like_rects = []
        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < 60.0:
                continue
            x, y, w, h = cv2.boundingRect(contour)
            aspect = float(w) / max(1.0, float(h))
            if 12 <= w <= max_eye_w and 4 <= h <= max_eye_h and aspect >= 1.6:
                eye_like_rects.append((x, y, w, h))

        if not eye_like_rects:
            return targets

        filtered = []
        for target in targets:
            cx, cy = target.center
            radius = max(1, int(target.radius))
            reject = False
            for x, y, w, h in eye_like_rects:
                margin = max(6, radius * 2)
                if x - margin <= cx <= x + w + margin and y - margin <= cy <= y + h + margin:
                    reject = True
                    break
            if not reject:
                filtered.append(target)
        return filtered

    def _draw_overlay(
        self,
        frame,
        targets: List[DetectionTarget],
        mode: str,
        safety_level: str,
        safety_message: str,
        draw_targets: bool = True,
    ):
        shown = frame.copy()
        if cv2 is None:
            return shown

        if draw_targets:
            if targets:
                for index, target in enumerate(targets, start=1):
                    color = (0, 70, 255) if index == 1 else (0, 180, 255)
                    cv2.circle(shown, (target.x, target.y), max(target.radius + 7, 12), color, 2)
                    cv2.drawMarker(
                        shown,
                        (target.x, target.y),
                        color,
                        markerType=cv2.MARKER_CROSS,
                        markerSize=18,
                        thickness=2,
                    )
                    cv2.putText(
                        shown,
                        f"#{index} ({target.x},{target.y})",
                        (target.x + 12, max(22, target.y - 12)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.52,
                        color,
                        2,
                    )
            else:
                cv2.putText(
                    shown,
                    "NO TARGET" if mode != MODE_PREVIEW else "PREVIEW",
                    (18, 34),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (80, 220, 80),
                    2,
                )

        cv2.putText(
            shown,
            self._overlay_mode_text(mode),
            (18, shown.shape[0] - 18),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            (255, 255, 255),
            2,
        )
        if safety_level == SAFETY_DANGER:
            cv2.rectangle(shown, (2, 2), (shown.shape[1] - 3, shown.shape[0] - 3), (0, 0, 255), 4)
            cv2.putText(
                shown,
                "DANGER: TOO CLOSE",
                (18, 68),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.78,
                (0, 0, 255),
                2,
            )
        elif safety_level == SAFETY_WARN:
            cv2.rectangle(shown, (2, 2), (shown.shape[1] - 3, shown.shape[0] - 3), (0, 180, 255), 2)
            cv2.putText(
                shown,
                "WARNING: CLOSE RANGE",
                (18, 68),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.66,
                (0, 180, 255),
                2,
            )
        del safety_message
        return shown

    def _overlay_mode_text(self, mode: str) -> str:
        mapping = {
            MODE_PREVIEW: "PREVIEW",
            MODE_DETECT_ONCE: "DETECT ONCE",
            MODE_DETECTING: "DETECTING",
            MODE_REFERENCE_CAPTURE: "CAPTURING REFERENCE",
            MODE_SAFETY_CAPTURE: "CAPTURING SAFETY",
        }
        return mapping.get(mode, "PREVIEW")

    def _to_qimage(self, frame) -> QImage:
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb.shape
        return QImage(rgb.data, w, h, ch * w, QImage.Format_RGB888).copy()

    def _demo_frame(self):
        if np is None or cv2 is None:
            raise RuntimeError("演示模式需要 numpy 和 opencv")
        if self._is_stereo_enabled():
            left_w = max(1, self.width // 2)
            right_w = max(1, self.width - left_w)
            left = self._demo_eye_frame(left_w, self.height, x_shift=0, label="LEFT")
            right = self._demo_eye_frame(right_w, self.height, x_shift=-18, label="RIGHT")
            return np.hstack([left, right])
        return self._demo_eye_frame(self.width, self.height, x_shift=0, label="")

    def _demo_eye_frame(self, width: int, height: int, x_shift: int, label: str):
        t = time.monotonic()
        frame = np.full((height, width, 3), (224, 204, 185), dtype=np.uint8)
        cx = int(width * 0.5 + math.sin(t) * width * 0.12 + x_shift)
        cy = int(height * 0.5 + math.cos(t * 0.7) * height * 0.10)
        cx = max(12, min(width - 12, cx))
        cy = max(12, min(height - 12, cy))
        cv2.circle(frame, (cx, cy), 7, (24, 24, 26), -1)
        cv2.rectangle(frame, (0, 0), (width - 1, height - 1), (88, 96, 105), 2)
        if label:
            cv2.putText(
                frame,
                label,
                (12, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (40, 72, 100),
                2,
                cv2.LINE_AA,
            )
        return frame


class DepthWorker(QThread):
    depth_ready = pyqtSignal(object)
    status_changed = pyqtSignal(str)

    def __init__(
        self,
        enabled: bool,
        timeout_ms: int,
        rotate_180: bool,
        ui_fps: float,
        distortion_mode: str = "auto",
    ) -> None:
        super().__init__()
        self.enabled = bool(enabled)
        self.timeout_ms = max(100, int(timeout_ms))
        self.rotate_180 = bool(rotate_180)
        self.ui_interval = 1.0 / max(1.0, min(60.0, float(ui_fps)))
        self.distortion_mode = str(distortion_mode or "auto")
        self._running = True

    def stop(self) -> None:
        self._running = False

    def run(self) -> None:
        if not self.enabled:
            self.depth_ready.emit(
                DepthPreview(False, None, 0, 0, 0.0, 0.0, 0.0, "深度未启用")
            )
            return
        if cv2 is None or np is None:
            self.depth_ready.emit(
                DepthPreview(False, None, 0, 0, 0.0, 0.0, 0.0, "OpenCV / numpy 不可用")
            )
            return

        pipeline = None
        try:
            from pyorbbecsdk import Config, OBSensorType, Pipeline

            config = Config()
            pipeline = Pipeline()
            profile_list = pipeline.get_stream_profile_list(OBSensorType.DEPTH_SENSOR)
            if profile_list is None:
                raise RuntimeError("未发现 ORBBEC Depth Sensor")
            depth_profile = profile_list.get_default_video_stream_profile()
            if depth_profile is None:
                raise RuntimeError("未发现默认深度流配置")
            config.enable_stream(depth_profile)
            pipeline.start(config)
            (
                depth_intrinsics,
                rgb_intrinsics,
                depth_to_rgb_rotation,
                depth_to_rgb_translation,
                depth_calibration_size,
                rgb_calibration_size,
                depth_distortion,
                rgb_distortion,
                depth_distortion_mode,
            ) = self._get_rgbd_calibration(pipeline)
            self.status_changed.emit("Astra Pro Plus 深度流已启动")

            last_time = time.monotonic()
            fps_meter = 0.0
            last_ui_emit = 0.0
            while self._running:
                frames = pipeline.wait_for_frames(self.timeout_ms)
                if frames is None:
                    continue
                depth_frame = frames.get_depth_frame()
                if depth_frame is None:
                    continue

                depth_mm = self._read_depth_frame(depth_frame)
                sample_time = time.monotonic()
                display_depth_mm = depth_mm
                if self.rotate_180:
                    display_depth_mm = cv2.rotate(depth_mm, cv2.ROTATE_180)

                now = time.monotonic()
                dt = max(now - last_time, 1e-6)
                last_time = now
                fps_meter = 0.9 * fps_meter + 0.1 * (1.0 / dt) if fps_meter else 1.0 / dt
                if now - last_ui_emit < self.ui_interval:
                    continue
                last_ui_emit = now

                valid = (depth_mm > 20.0) & (depth_mm < 10000.0)
                valid_ratio = float(np.count_nonzero(valid)) / float(valid.size or 1)
                center_depth = self._center_depth(depth_mm, valid)
                shown = self._make_depth_image(display_depth_mm)
                self.depth_ready.emit(
                    DepthPreview(
                        True,
                        self._to_qimage(shown),
                        int(depth_mm.shape[1]),
                        int(depth_mm.shape[0]),
                        fps_meter,
                        center_depth,
                        valid_ratio,
                        "Depth 正常",
                        depth_mm.copy(),
                        depth_intrinsics,
                        rgb_intrinsics,
                        depth_to_rgb_rotation,
                        depth_to_rgb_translation,
                        sample_time,
                        depth_calibration_size,
                        rgb_calibration_size,
                        depth_distortion,
                        rgb_distortion,
                        depth_distortion_mode,
                    )
                )
        except Exception as exc:
            self.depth_ready.emit(
                DepthPreview(False, None, 0, 0, 0.0, 0.0, 0.0, "深度不可用：{}".format(exc))
            )
            self.status_changed.emit("Astra Pro Plus 深度不可用：{}".format(exc))
        finally:
            if pipeline is not None:
                try:
                    pipeline.stop()
                except Exception:
                    pass

    def _read_depth_frame(self, depth_frame):
        width = depth_frame.get_width()
        height = depth_frame.get_height()
        scale = depth_frame.get_depth_scale()
        depth = np.frombuffer(depth_frame.get_data(), dtype=np.uint16).reshape((height, width))
        return depth.astype(np.float32) * float(scale)

    def _get_depth_intrinsics(self, pipeline) -> Optional[Tuple[float, float, float, float]]:
        try:
            camera_param = pipeline.get_camera_param()
        except Exception:
            return None

        def attr_any(obj, names):
            if obj is None:
                return None
            for name in names:
                if not hasattr(obj, name):
                    continue
                value = getattr(obj, name)
                if callable(value):
                    try:
                        value = value()
                    except TypeError:
                        pass
                return value
            return None

        candidates = [
            camera_param,
            attr_any(camera_param, ("depth_intrinsic", "depthIntrinsic", "depth_intrinsics")),
            attr_any(camera_param, ("depth_intrinsic_param", "depthIntrinsicParam")),
        ]
        for candidate in candidates:
            fx = attr_any(candidate, ("fx", "focalLengthX", "focal_length_x"))
            fy = attr_any(candidate, ("fy", "focalLengthY", "focal_length_y"))
            cx = attr_any(candidate, ("cx", "principalPointX", "principal_point_x", "ppx"))
            cy = attr_any(candidate, ("cy", "principalPointY", "principal_point_y", "ppy"))
            if None not in (fx, fy, cx, cy):
                return float(fx), float(fy), float(cx), float(cy)
        return None

    def _get_rgbd_calibration(self, pipeline):
        depth_intrinsics = self._get_depth_intrinsics(pipeline)
        depth_calibration_size = None
        depth_distortion = None
        try:
            camera_param = pipeline.get_camera_param()
            depth = getattr(camera_param, "depth_intrinsic", None)
            rgb = getattr(camera_param, "rgb_intrinsic", None)
            transform = getattr(camera_param, "transform", None)
            sdk_depth_distortion = getattr(camera_param, "depth_distortion", None)
            sdk_rgb_distortion = getattr(camera_param, "rgb_distortion", None)
            depth_calibration_size = (int(depth.width), int(depth.height))
            rgb_calibration_size = (int(rgb.width), int(rgb.height))
            if min(*depth_calibration_size, *rgb_calibration_size) <= 0:
                raise ValueError("RGB-D calibration resolution is invalid")
            rgb_intrinsics = (
                float(rgb.fx),
                float(rgb.fy),
                float(rgb.cx),
                float(rgb.cy),
            )
            if rgb_intrinsics[0] <= 0.0 or rgb_intrinsics[1] <= 0.0:
                raise ValueError("RGB intrinsics are invalid")
            def distortion_values(value):
                # Match OpenCV's Brown/rational order used by the projection
                # path: k1, k2, p1, p2, k3, k4, k5, k6.
                result = tuple(
                    float(getattr(value, name))
                    for name in ("k1", "k2", "p1", "p2", "k3", "k4", "k5", "k6")
                )
                if not all(math.isfinite(item) for item in result):
                    raise ValueError("RGB-D distortion contains non-finite values")
                return result

            sdk_depth_distortion_values = distortion_values(sdk_depth_distortion)
            rgb_distortion = distortion_values(sdk_rgb_distortion)
            depth_distortion, depth_distortion_mode = self._resolve_depth_distortion(
                pipeline,
                sdk_depth_distortion_values,
            )
            rotation = np.asarray(transform.rot, dtype=np.float64).reshape(3, 3).copy()
            translation = np.asarray(transform.transform, dtype=np.float64).reshape(3).copy()
            if not np.all(np.isfinite(rotation)) or not np.all(np.isfinite(translation)):
                raise ValueError("RGB-D extrinsics contain non-finite values")
            if abs(float(np.linalg.det(rotation))) < 0.5:
                raise ValueError("RGB-D rotation matrix is invalid")
            return (
                depth_intrinsics,
                rgb_intrinsics,
                rotation,
                translation,
                depth_calibration_size,
                rgb_calibration_size,
                depth_distortion,
                rgb_distortion,
                depth_distortion_mode,
            )
        except Exception as exc:
            self.status_changed.emit("RGB-Depth 外参不可用：{}".format(exc))
            return (
                depth_intrinsics,
                None,
                None,
                None,
                depth_calibration_size,
                None,
                depth_distortion,
                None,
                "unknown",
            )

    def _resolve_depth_distortion(self, pipeline, sdk_distortion):
        if self.distortion_mode == "raw":
            return tuple(sdk_distortion), "raw-explicit"
        if self.distortion_mode == "rectified":
            return (0.0,) * 8, "rectified-explicit"
        try:
            from pyorbbecsdk import OBPermissionType, OBPropertyID

            device = pipeline.get_device()
            prop = OBPropertyID.OB_PROP_HARDWARE_DISTORTION_SWITCH_BOOL
            if not device.is_property_supported(prop, OBPermissionType.PERMISSION_READ):
                self.status_changed.emit(
                    "Depth硬件去畸变状态不可查询；正式RGB-D测距保持无效"
                )
                return None, "unknown"
            enabled = bool(device.get_bool_property(prop))
            if enabled:
                return (0.0,) * 8, "rectified-auto"
            return tuple(sdk_distortion), "raw-auto"
        except Exception as exc:
            self.status_changed.emit(
                "Depth硬件去畸变状态读取失败：{}；正式RGB-D测距保持无效".format(exc)
            )
            return None, "unknown"

    def _make_depth_image(self, depth_mm):
        shown_depth = np.where(
            (depth_mm > 20.0) & (depth_mm < 10000.0),
            depth_mm,
            0,
        ).astype(np.uint16)
        depth_u8 = cv2.normalize(shown_depth, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        return cv2.applyColorMap(depth_u8, cv2.COLORMAP_JET)

    def _center_depth(self, depth_mm, valid) -> float:
        height, width = depth_mm.shape[:2]
        cx = width // 2
        cy = height // 2
        radius = 8
        y0 = max(0, cy - radius)
        y1 = min(height, cy + radius + 1)
        x0 = max(0, cx - radius)
        x1 = min(width, cx + radius + 1)
        roi = depth_mm[y0:y1, x0:x1]
        roi_valid = valid[y0:y1, x0:x1]
        if not np.any(roi_valid):
            return 0.0
        return float(np.median(roi[roi_valid]))

    def _to_qimage(self, frame) -> QImage:
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb.shape
        return QImage(rgb.data, w, h, ch * w, QImage.Format_RGB888).copy()


class AstraRgbWorker(QThread):
    preview_ready = pyqtSignal(object)
    status_changed = pyqtSignal(str)

    def __init__(
        self,
        enabled: bool,
        camera: str,
        width: int,
        height: int,
        fps: int,
        ui_fps: float,
        flip: str,
        yolo_enabled: bool,
        yolo_model: str,
        yolo_conf: float,
        yolo_iou: float,
        yolo_imgsz: int,
        yolo_device: str,
        yolo_arm_name: str,
        yolo_face_name: str,
        yolo_interval: float,
    ) -> None:
        super().__init__()
        self.enabled = bool(enabled)
        self.camera = camera
        self.width = int(width)
        self.height = int(height)
        self.fps = int(fps)
        self.ui_interval = 1.0 / max(1.0, min(60.0, float(ui_fps)))
        self.flip = flip
        self.yolo_enabled = bool(yolo_enabled)
        self.yolo_model = yolo_model
        self.yolo_conf = max(0.01, min(0.99, float(yolo_conf)))
        self.yolo_iou = max(0.01, min(0.99, float(yolo_iou)))
        self.yolo_imgsz = max(64, int(yolo_imgsz))
        self.yolo_device = str(yolo_device or "").strip()
        self.yolo_arm_name = str(yolo_arm_name or "arm_tip")
        self.yolo_face_name = str(yolo_face_name or "face")
        self.yolo_interval = max(0.05, float(yolo_interval))
        self._yolo_model = None
        self._yolo_names = {}
        self._yolo_arm_id = None
        self._yolo_face_id = None
        self._yolo_unavailable = False
        self._yolo_message = "YOLO 等待识别" if self.yolo_enabled else "YOLO 未启用"
        self._running = True

    def stop(self) -> None:
        self._running = False

    def run(self) -> None:
        if not self.enabled:
            self.preview_ready.emit(AstraRgbPreview(False, None, 0, 0, 0.0, "Astra RGB 未启用"))
            return
        if IMPORT_ERROR is not None:
            self.preview_ready.emit(AstraRgbPreview(False, None, 0, 0, 0.0, "Astra RGB 依赖不可用"))
            self.status_changed.emit("Astra RGB 依赖不可用：{}".format(IMPORT_ERROR))
            return

        cap = None
        try:
            cap, label = open_camera(self.camera, self.width, self.height, self.fps)
            self.status_changed.emit("Astra RGB 已打开：{}".format(label))
            last_time = time.monotonic()
            fps_meter = 0.0
            last_ui_emit = 0.0
            last_yolo_run = 0.0
            last_yolo_detections = {}
            while self._running:
                ok, frame = cap.read()
                if not ok or frame is None:
                    time.sleep(0.02)
                    continue
                if self.flip != "none" and apply_flip is not None:
                    frame = apply_flip(frame, self.flip)
                now = time.monotonic()
                dt = max(now - last_time, 1e-6)
                last_time = now
                fps_meter = 0.9 * fps_meter + 0.1 * (1.0 / dt) if fps_meter else 1.0 / dt
                if now - last_ui_emit < self.ui_interval:
                    continue
                last_ui_emit = now
                display_frame = frame
                yolo_detections = last_yolo_detections
                if self.yolo_enabled and now - last_yolo_run >= self.yolo_interval:
                    last_yolo_run = now
                    display_frame, self._yolo_message, yolo_detections = self._run_yolo(frame)
                    last_yolo_detections = yolo_detections
                elif last_yolo_detections:
                    display_frame = self._draw_yolo_detections(frame.copy(), last_yolo_detections)
                image = self._to_qimage(display_frame)
                self.preview_ready.emit(
                    AstraRgbPreview(
                        True,
                        image,
                        int(frame.shape[1]),
                        int(frame.shape[0]),
                        fps_meter,
                        "Astra RGB 正常",
                        self._yolo_message,
                        "arm" in yolo_detections,
                        "face" in yolo_detections,
                        dict(yolo_detections),
                        last_yolo_run,
                    )
                )
        except Exception as exc:
            message = "Astra RGB 不可用：{}".format(exc)
            self.preview_ready.emit(AstraRgbPreview(False, None, 0, 0, 0.0, message))
            self.status_changed.emit(message)
        finally:
            if cap is not None:
                cap.release()

    def _run_yolo(self, frame):
        display_frame = frame.copy()
        try:
            model = self._ensure_yolo_model()
            if model is None:
                return display_frame, self._yolo_message, {}
            kwargs = {
                "conf": self.yolo_conf,
                "iou": self.yolo_iou,
                "imgsz": self.yolo_imgsz,
                # Filter during Ultralytics post-processing instead of first
                # returning every class and discarding them below.  Keep room
                # for duplicate candidates from both required classes: a
                # global max_det of 2 could let two high-confidence faces hide
                # the arm tip (or vice versa).
                "classes": sorted({int(self._yolo_arm_id), int(self._yolo_face_id)}),
                "max_det": 10,
                "verbose": False,
            }
            if self.yolo_device:
                kwargs["device"] = self.yolo_device
            results = model.predict(frame, **kwargs)
            result = results[0] if results else None
            detections = self._best_yolo_detections(result)
            display_frame = self._draw_yolo_detections(display_frame, detections)
            return display_frame, self._format_yolo_message(detections), detections
        except Exception as exc:
            self._yolo_message = "YOLO 识别错误：{}".format(exc)
            return display_frame, self._yolo_message, {}

    def _ensure_yolo_model(self):
        if self._yolo_model is not None:
            return self._yolo_model
        if self._yolo_unavailable:
            return None
        try:
            from ultralytics import YOLO
        except Exception as exc:
            self._yolo_unavailable = True
            self._yolo_message = "YOLO 依赖未安装：{}".format(exc)
            self.status_changed.emit(self._yolo_message)
            return None

        model_path = Path(self.yolo_model).expanduser()
        if not model_path.is_absolute():
            model_path = ROOT / model_path
        if not model_path.exists():
            self._yolo_unavailable = True
            self._yolo_message = "YOLO 模型不存在：{}".format(model_path)
            self.status_changed.emit(self._yolo_message)
            return None

        try:
            model = YOLO(str(model_path))
            names = getattr(model, "names", {})
            self._yolo_names = {int(k): str(v) for k, v in dict(names).items()}
            self._yolo_arm_id = self._class_id_by_name(self._yolo_names, self.yolo_arm_name)
            self._yolo_face_id = self._class_id_by_name(self._yolo_names, self.yolo_face_name)
            if self._yolo_arm_id is None or self._yolo_face_id is None:
                raise RuntimeError(
                    "模型类别不匹配：需要 {} / {}，当前 {}".format(
                        self.yolo_arm_name,
                        self.yolo_face_name,
                        self._yolo_names,
                    )
                )
            self._yolo_model = model
            self._yolo_message = "YOLO 已加载，等待目标"
            self.status_changed.emit("Astra YOLO 已加载：{}".format(model_path))
            return self._yolo_model
        except Exception as exc:
            self._yolo_unavailable = True
            self._yolo_message = "YOLO 加载失败：{}".format(exc)
            self.status_changed.emit(self._yolo_message)
            return None

    def _class_id_by_name(self, names, wanted: str):
        for class_id, name in names.items():
            if name == wanted:
                return class_id
        return None

    def _best_yolo_detections(self, result):
        detections = {}
        if result is None:
            return detections
        boxes = getattr(result, "boxes", None)
        if boxes is None or getattr(boxes, "xyxy", None) is None:
            return detections
        try:
            xyxy = boxes.xyxy.detach().cpu().numpy()
            cls = boxes.cls.detach().cpu().numpy().astype(np.int32)
            conf = boxes.conf.detach().cpu().numpy()
        except Exception:
            return detections
        for box, class_id, score in zip(xyxy, cls, conf):
            class_id = int(class_id)
            if class_id == self._yolo_arm_id:
                key = "arm"
            elif class_id == self._yolo_face_id:
                key = "face"
            else:
                continue
            prev = detections.get(key)
            if prev is None or float(score) > prev["conf"]:
                detections[key] = {
                    "box": [float(v) for v in box],
                    "conf": float(score),
                    "name": self._yolo_names.get(class_id, str(class_id)),
                }
        return detections

    def _draw_yolo_detections(self, frame, detections):
        if cv2 is None:
            return frame
        styles = {
            "arm": ((40, 245, 90), "机械臂末端"),
            "face": ((0, 210, 255), "人脸"),
        }
        height, width = frame.shape[:2]
        for key, (color, title) in styles.items():
            detection = detections.get(key)
            if detection is None:
                continue
            x0, y0, x1, y1 = [int(round(v)) for v in detection["box"]]
            x0 = max(0, min(width - 1, x0))
            x1 = max(0, min(width - 1, x1))
            y0 = max(0, min(height - 1, y0))
            y1 = max(0, min(height - 1, y1))
            cv2.rectangle(frame, (x0, y0), (x1, y1), color, 2)
            label = "{} {:.2f}".format(title, detection["conf"])
            self._draw_yolo_text(frame, label, (x0 + 4, max(20, y0 - 6)), color)
        return frame

    def _draw_yolo_text(self, frame, text: str, origin, color) -> None:
        font = cv2.FONT_HERSHEY_SIMPLEX
        scale = 0.55
        thickness = 1
        x, y = origin
        (text_w, text_h), baseline = cv2.getTextSize(text, font, scale, thickness)
        y0 = max(0, y - text_h - baseline - 4)
        x0 = max(0, x - 3)
        x1 = min(frame.shape[1] - 1, x + text_w + 5)
        y1 = min(frame.shape[0] - 1, y + baseline + 3)
        cv2.rectangle(frame, (x0, y0), (x1, y1), (12, 20, 24), -1)
        cv2.putText(frame, text, (x, y), font, scale, color, thickness, cv2.LINE_AA)

    def _format_yolo_message(self, detections) -> str:
        arm = detections.get("arm")
        face = detections.get("face")
        if arm is not None and face is not None:
            return "YOLO 已识别：机械臂末端 {:.2f} / 人脸 {:.2f}".format(
                arm["conf"],
                face["conf"],
            )
        missing = []
        if arm is None:
            missing.append("机械臂末端")
        if face is None:
            missing.append("人脸")
        return "YOLO 等待：{}".format("、".join(missing))

    def _to_qimage(self, frame) -> QImage:
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb.shape
        return QImage(rgb.data, w, h, ch * w, QImage.Format_RGB888).copy()


def _qimage_to_bgr(image: QImage):
    if cv2 is None or np is None:
        return None
    rgb_image = image.convertToFormat(QImage.Format_RGB888)
    width = rgb_image.width()
    height = rgb_image.height()
    ptr = rgb_image.bits()
    ptr.setsize(height * width * 3)
    rgb = np.frombuffer(ptr, dtype=np.uint8).reshape((height, width, 3)).copy()
    return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)


def _bgr_to_qimage(frame) -> QImage:
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    h, w, ch = rgb.shape
    return QImage(rgb.data, w, h, ch * w, QImage.Format_RGB888).copy()


def _spot_roi_for_frame(frame, spot_roi: Tuple[float, float, float, float]):
    h, w = frame.shape[:2]
    x_min, x_max, y_min, y_max = spot_roi
    x0 = max(0, min(w - 1, int(round(w * x_min))))
    x1 = max(1, min(w, int(round(w * x_max))))
    y0 = max(0, min(h - 1, int(round(h * y_min))))
    y1 = max(1, min(h, int(round(h * y_max))))
    if x1 <= x0 or y1 <= y0:
        return None
    return x0, y0, x1 - x0, y1 - y0


def _detect_moles_on_bgr(frame, spot_params, spot_roi: Tuple[float, float, float, float]):
    try:
        raw_targets = detect_moles(
            frame,
            params=spot_params,
            roi=_spot_roi_for_frame(frame, spot_roi),
        )
    except Exception as exc:
        return [], "识别失败：{}".format(exc), frame

    targets = [
        DetectionTarget(
            x=int(item.center[0]),
            y=int(item.center[1]),
            radius=int(item.radius),
            area=float(item.area),
            contrast=float(item.contrast),
            circularity=float(item.circularity),
        )
        for item in raw_targets[:8]
    ]
    shown = frame.copy()
    for index, target in enumerate(targets, start=1):
        color = (0, 70, 255) if index == 1 else (0, 180, 255)
        cv2.circle(shown, (target.x, target.y), max(target.radius + 6, 10), color, 2)
        cv2.drawMarker(
            shown,
            (target.x, target.y),
            color,
            markerType=cv2.MARKER_CROSS,
            markerSize=16,
            thickness=2,
        )
    if targets:
        return targets, "发现 {} 个候选黑痣".format(len(targets)), shown
    return [], "未识别到黑痣", shown


def _detect_candidates_on_bgr(
    frame,
    detector_mode: str,
    facial_detector,
    spot_params,
    spot_roi: Tuple[float, float, float, float],
):
    if detector_mode != "facial_candidate":
        return _detect_moles_on_bgr(frame, spot_params, spot_roi)
    if facial_detector is None:
        return [], "人脸候选检测器不可用", frame.copy()
    try:
        result = facial_detector.detect(frame)
    except Exception as exc:
        return [], "人脸候选检测失败：{}".format(exc), frame.copy()
    targets = [
        DetectionTarget(
            x=int(item.center[0]),
            y=int(item.center[1]),
            radius=int(item.radius),
            area=float(item.area),
            contrast=float(item.contrast),
            circularity=float(item.circularity),
        )
        for item in result.candidates
    ]
    if result.face is None:
        return [], "未检测到有效人脸", result.privacy_view
    return targets, "发现 {} 个候选暗斑".format(len(targets)), result.privacy_view


def _target_quality_score(target: DetectionTarget) -> float:
    return max(0.0, float(target.area)) * max(0.0, float(target.contrast))


def _select_primary_triptych_targets(targets: List[DetectionTarget]) -> List[DetectionTarget]:
    if not targets:
        return []
    return [max(targets, key=_target_quality_score)]


def _mole_candidates_to_targets(candidates) -> List[DetectionTarget]:
    return [
        DetectionTarget(
            x=int(item.center[0]),
            y=int(item.center[1]),
            radius=int(item.radius),
            area=float(item.area),
            contrast=float(item.contrast),
            circularity=float(item.circularity),
            track_id=int(getattr(item, "track_id", 0)),
            stable_count=int(getattr(item, "stable_count", 0)),
            required_count=int(getattr(item, "required_count", 0)),
            track_age_s=float(getattr(item, "track_age_s", 0.0)),
        )
        for item in candidates
    ]


def _draw_detection_targets_on_bgr(frame, targets: List[DetectionTarget]):
    shown = frame.copy()
    for index, target in enumerate(targets, start=1):
        color = (0, 70, 255) if index == 1 else (0, 180, 255)
        cv2.circle(
            shown,
            (int(target.x), int(target.y)),
            max(int(target.radius) + 7, 10),
            color,
            2,
        )
        cv2.drawMarker(
            shown,
            (int(target.x), int(target.y)),
            color,
            markerType=cv2.MARKER_CROSS,
            markerSize=16,
            thickness=2,
        )
        cv2.putText(
            shown,
            "#{} X={} Y={}".format(index, int(target.x), int(target.y)),
            (
                min(int(target.x) + 20, max(0, shown.shape[1] - 180)),
                max(22, int(target.y) - 4),
            ),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            color,
            1,
            cv2.LINE_AA,
        )
    return shown


def _triptych_view_name(point_id: int) -> str:
    return {1: "left", 2: "front", 3: "right"}.get(int(point_id), "front")


class TriptychDetectionWorker(QThread):
    detection_finished = pyqtSignal(object)

    def __init__(
        self,
        images,
        detector_mode: str,
        spot_params,
        spot_roi: Tuple[float, float, float, float],
        facial_face_model: Path,
        facial_face_conf: float,
        facial_face_device: str,
        facial_max_candidates: int,
        facial_face_imgsz: int = 416,
        facial_enhanced_filters: bool = True,
        triptych_top_candidate_only: bool = True,
        baseline_face_enabled: bool = True,
        baseline_face_dir: str = "",
    ) -> None:
        super().__init__()
        self.images = [(int(point_id), image.copy()) for point_id, image in images]
        self.detector_mode = str(detector_mode)
        self.spot_params = spot_params
        self.spot_roi = spot_roi
        self.facial_face_model = Path(facial_face_model)
        self.facial_face_conf = float(facial_face_conf)
        self.facial_face_device = str(facial_face_device)
        self.facial_max_candidates = max(1, int(facial_max_candidates))
        self.facial_face_imgsz = max(160, int(facial_face_imgsz))
        self.facial_enhanced_filters = bool(facial_enhanced_filters)
        self.triptych_top_candidate_only = bool(triptych_top_candidate_only)
        self.baseline_face_enabled = bool(baseline_face_enabled)
        self.baseline_face_dir = str(baseline_face_dir)
        self._facial_detector = None
        self._baseline_library = None

    def run(self) -> None:
        results = []
        if cv2 is None or np is None or detect_moles is None:
            self.detection_finished.emit(results)
            return

        if self.detector_mode == "facial_candidate":
            try:
                self._facial_detector = FacialCandidateDetector(
                    FaceLocator(
                        model_path=self.facial_face_model,
                        confidence=self.facial_face_conf,
                        imgsz=self.facial_face_imgsz,
                        device=self.facial_face_device,
                    ),
                    candidate_params=self.spot_params,
                    params=FacialCandidateParams(
                        max_candidates=self.facial_max_candidates,
                        detection_exclusion_mode="eyes_only",
                        enhanced_candidate_filters=self.facial_enhanced_filters,
                        # These are three unrelated arm poses, not adjacent
                        # video frames.  Never carry a face box from one view
                        # into the next still image.
                        face_refresh_frames=1,
                    ),
                )
            except Exception:
                self._facial_detector = None
            if self.baseline_face_enabled and BaselineFaceLibrary is not None:
                try:
                    self._baseline_library = BaselineFaceLibrary(
                        self.baseline_face_dir,
                        enabled=True,
                    )
                except Exception:
                    self._baseline_library = None

        for point_id, image in self.images:
            if self.isInterruptionRequested():
                return
            frame = _qimage_to_bgr(image)
            if frame is None:
                continue
            targets, message, shown = self._detect_one(frame)
            if self._baseline_library is not None:
                if targets:
                    targets = self._rank_targets_with_baseline(point_id, frame, targets)
                    shown = _draw_detection_targets_on_bgr(frame, targets)
                    message = "{}；已用无黑痣基准库排序".format(message)
                else:
                    fallback = self._baseline_library.detect_darkening_candidates(
                        frame,
                        view=_triptych_view_name(point_id),
                        min_darkening=max(16.0, float(self.spot_params.min_contrast) * 0.65),
                        min_area=max(5.0, float(self.spot_params.min_area) * 0.65),
                        max_area=float(self.spot_params.max_area),
                        max_candidates=self.facial_max_candidates,
                    )
                    if fallback:
                        targets = _mole_candidates_to_targets(fallback)
                        shown = _draw_detection_targets_on_bgr(frame, targets)
                        message = "{}；无黑痣基准库补充候选 {} 个".format(
                            message,
                            len(targets),
                        )
            if self.triptych_top_candidate_only and len(targets) > 1:
                targets = _select_primary_triptych_targets(targets)
                shown = _draw_detection_targets_on_bgr(frame, targets)
                message = "{}；已保留最可信候选".format(message)
            results.append(
                TriptychDetectionResult(
                    point_id=point_id,
                    image=_bgr_to_qimage(shown),
                    targets=targets,
                    message=message,
                    frame_width=int(frame.shape[1]),
                    frame_height=int(frame.shape[0]),
                )
            )
        if self.isInterruptionRequested():
            return
        self.detection_finished.emit(results)

    def _detect_one(self, frame):
        if self._facial_detector is not None:
            self._facial_detector.reset()
        return _detect_candidates_on_bgr(
            frame,
            self.detector_mode,
            self._facial_detector,
            self.spot_params,
            self.spot_roi,
        )

    def _rank_targets_with_baseline(self, point_id: int, frame, targets):
        if self._baseline_library is None or not targets:
            return list(targets)
        candidates = [
            DetectedMole(
                center=(int(target.x), int(target.y)),
                radius=int(target.radius),
                area=float(target.area),
                circularity=float(target.circularity),
                contrast=float(target.contrast),
            )
            for target in targets
        ]
        ranked = self._baseline_library.score_candidates(
            frame,
            candidates,
            view=_triptych_view_name(point_id),
            reject_static_dark=True,
        )
        if not ranked:
            return []
        target_by_key = {
            (
                int(target.x),
                int(target.y),
                int(target.radius),
                round(float(target.area), 3),
            ): target
            for target in targets
        }
        ranked_targets = []
        for candidate in ranked:
            key = (
                int(candidate.center[0]),
                int(candidate.center[1]),
                int(candidate.radius),
                round(float(candidate.area), 3),
            )
            target = target_by_key.get(key)
            if target is not None:
                ranked_targets.append(target)
        if ranked_targets:
            return ranked_targets
        return list(targets)

    def _spot_roi_for_frame(self, frame):
        return _spot_roi_for_frame(frame, self.spot_roi)


def _resolve_reference_path(path_text: str) -> Path:
    if path_text:
        path = Path(path_text).expanduser()
        if path.is_absolute():
            return path
        return (Path.cwd() / path).resolve()
    return DEFAULT_REFERENCE_PATH


def _resolve_safety_image_path(path_text: str) -> Path:
    if path_text:
        path = Path(path_text).expanduser()
        if path.is_absolute():
            return path
        return (Path.cwd() / path).resolve()
    return DEFAULT_SAFETY_IMAGE_PATH


def _resolve_safety_reference_path(path_text: str) -> Path:
    if path_text:
        path = Path(path_text).expanduser()
        if path.is_absolute():
            return path
        return (Path.cwd() / path).resolve()
    return DEFAULT_SAFETY_REFERENCE_PATH


def _load_camera_calibration(path_text: str) -> CameraCalibration:
    if np is None:
        raise RuntimeError("numpy不可用")
    path = Path(path_text).expanduser()
    if not path.is_absolute():
        path = (Path.cwd() / path).resolve()
    if not path.exists():
        raise FileNotFoundError(str(path))
    with np.load(str(path), allow_pickle=False) as payload:
        matrix = np.asarray(payload["camera_matrix"], dtype=np.float64).reshape(3, 3)
        coefficients = np.asarray(payload["dist_coeffs"], dtype=np.float64).reshape(-1)
        image_size_values = np.asarray(payload["image_size"], dtype=np.int64).reshape(-1)
        rms_values = np.asarray(payload.get("rms", np.asarray(float("nan")))).reshape(-1)
    if coefficients.size < 5:
        raise ValueError("畸变参数少于5项")
    if image_size_values.size != 2:
        raise ValueError("标定分辨率无效")
    if not np.all(np.isfinite(matrix)) or not np.all(np.isfinite(coefficients)):
        raise ValueError("标定参数包含非有限值")
    fx = float(matrix[0, 0])
    fy = float(matrix[1, 1])
    cx = float(matrix[0, 2])
    cy = float(matrix[1, 2])
    if min(fx, fy) <= 0.0:
        raise ValueError("焦距参数无效")
    # OpenCV allows 5, 8 or more coefficients.  The current projection path
    # intentionally consumes the Brown/rational first eight only.
    distortion = np.zeros(8, dtype=np.float64)
    distortion[: min(8, coefficients.size)] = coefficients[:8]
    width, height = [int(value) for value in image_size_values]
    if min(width, height) <= 0:
        raise ValueError("标定分辨率无效")
    rms_px = float(rms_values[0]) if rms_values.size else float("nan")
    return CameraCalibration(
        (fx, fy, cx, cy),
        tuple(float(value) for value in distortion),
        (width, height),
        rms_px,
    )


def _build_reference_params(args: argparse.Namespace):
    if ReferenceDetectParams is None:
        return None
    return ReferenceDetectParams(
        min_darkening=int(args.ref_min_darkening),
        min_mean_darkening=float(args.ref_min_mean_darkening),
        min_confidence=float(args.ref_min_confidence),
        mask_erode=int(args.ref_mask_erode),
        max_translation_px=float(args.ref_max_translation_px),
        max_rotation_deg=float(args.ref_max_rotation_deg),
        min_alignment_score=float(args.ref_min_alignment_score),
    )


def _build_target_identity_lock(args: argparse.Namespace) -> TargetIdentityLock:
    """Build the session target lock from the validated Qt CLI settings."""

    return TargetIdentityLock(
        max_step_ratio=args.target_lock_max_step_ratio,
        max_radius_ratio=args.target_lock_max_radius_ratio,
        max_step_radius_factor=args.target_lock_max_step_radius_factor,
        reacquire_radius_factor=args.target_lock_reacquire_radius_factor,
        ambiguity_radius_factor=args.target_lock_ambiguity_radius_factor,
        stable_frames=args.target_lock_stable_frames,
        track_id_rebind_timeout_s=args.target_lock_rebind_timeout,
    )


def _build_spot_params(args: argparse.Namespace):
    if DetectParams is None:
        return None
    return DetectParams(
        min_area=int(args.spot_min_area),
        max_area=int(args.spot_max_area),
        min_circularity=float(args.spot_min_circularity),
        min_solidity=float(args.spot_min_solidity),
        max_aspect_ratio=float(args.spot_max_aspect_ratio),
        blackhat_ksize=int(args.spot_blackhat_ksize),
        min_contrast=int(args.spot_min_contrast),
        skin_dilate=int(args.spot_skin_dilate),
        use_skin_mask=True,
    )


def _build_spot_roi(args: argparse.Namespace) -> Tuple[float, float, float, float]:
    x_min = max(0.0, min(1.0, float(args.spot_roi_x_min_ratio)))
    x_max = max(0.0, min(1.0, float(args.spot_roi_x_max_ratio)))
    y_min = max(0.0, min(1.0, float(args.spot_roi_y_min_ratio)))
    y_max = max(0.0, min(1.0, float(args.spot_roi_y_max_ratio)))
    if x_max <= x_min:
        x_min, x_max = 0.0, 1.0
    if y_max <= y_min:
        y_min, y_max = 0.0, 1.0
    return x_min, x_max, y_min, y_max


class MainWindow(QMainWindow):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__()
        self.args = args
        self._unsupported_continuous_safe_distance_requested = bool(
            getattr(args, "continuous_safe_distance", False)
        )
        self.setWindowTitle("机械臂点痣装置")
        self.resize(args.window_width, args.window_height)
        self.reference_path = _resolve_reference_path(args.reference)
        self.safety_image_path = _resolve_safety_image_path(args.safety_image)
        self.safety_reference_path = _resolve_safety_reference_path(args.safety_reference)
        self._astra_rgb_calibration = None
        self._astra_rgb_calibration_error = ""
        try:
            self._astra_rgb_calibration = _load_camera_calibration(
                args.astra_rgb_calibration
            )
        except Exception as exc:
            self._astra_rgb_calibration_error = str(exc)
        self.reference_params = _build_reference_params(args)
        self.spot_params = _build_spot_params(args)
        self.spot_roi = _build_spot_roi(args)
        self.worker: Optional[VideoWorker] = None
        self.astra_rgb_worker: Optional[AstraRgbWorker] = None
        self.depth_worker: Optional[DepthWorker] = None
        self.triptych_worker: Optional[TriptychDetectionWorker] = None
        self._retired_workers: List[QThread] = []
        self._closing_requested = False
        self._restart_requested = False
        self._restart_started = False
        self._protocol_session_generation = 0
        self.arm = ArmController(
            enabled=bool(args.serial_port),
            port=args.serial_port,
            baudrate=args.baudrate,
            serial_protocol=args.serial_protocol,
            laser_offset_mm=args.laser_offset_mm,
            mm_per_pixel_x=args.mm_per_pixel_x,
            laser_offset_x_px=args.laser_offset_x_px,
            laser_offset_y_px=args.laser_offset_y_px,
            heartbeat_interval=args.serial_heartbeat_interval,
            vision_interval=args.serial_vision_interval,
            vision_max_age=args.vision_target_max_age,
            # Chapter 19 fixes the current formal workflow to START_MEASURE
            # (action=0x01). Never let the retired CLI proposal select 0x04.
            continuous_safe_distance=False,
        )
        self.arm_ready_timeout_ms = max(0, int(args.arm_ready_timeout_ms))
        self.serial_heartbeat_interval = max(0.10, min(0.20, float(args.serial_heartbeat_interval)))
        self.serial_safe_distance_interval = max(
            0.10, min(0.20, float(args.serial_safe_distance_interval))
        )
        self.serial_vision_interval = max(0.10, min(0.20, float(args.serial_vision_interval)))
        self.serial_safe_distance_source = str(args.serial_safe_distance_source)
        self.serial_safe_distance_max_age = max(0.1, float(args.serial_safe_distance_max_age))
        self.serial_safe_distance_min_valid_ratio = max(
            0.0,
            min(1.0, float(args.serial_safe_distance_min_valid_ratio)),
        )
        self.rgbd_sync_max_delta = max(0.01, min(0.50, float(args.rgbd_sync_max_delta)))
        self.rgbd_distance_log_enabled = bool(args.rgbd_distance_log)
        self.rgbd_distance_log_interval = max(0.10, float(args.rgbd_distance_log_interval))
        self.rgbd_continuous_stop_mm = max(0.0, float(args.rgbd_continuous_stop_mm))
        self.rgbd_continuous_guard_interval = max(
            0.10,
            min(1.0, float(args.rgbd_continuous_guard_interval)),
        )
        self._last_rgbd_distance_log_time = 0.0
        self._last_rgbd_distance_log_state = None
        self.serial_safe_distance_mm = max(0, min(65535, int(args.serial_safe_distance_mm)))
        self.serial_safe_distance_valid = bool(args.serial_safe_distance_valid)
        self.vision_target_max_age = max(0.10, min(0.50, float(args.vision_target_max_age)))
        self.current_result: Optional[DetectionResult] = None
        self._latest_result_time = 0.0
        self._target_identity_lock = _build_target_identity_lock(args)
        self._locked_current_target: Optional[DetectionTarget] = None
        self._locked_target_sample_time = 0.0
        self._last_target_lock_reason = "等待选中视图后的稳定候选"
        self.history: Deque[str] = deque(maxlen=30)
        self._waiting_for_arm_ready = False
        self._detection_running = False
        self._reference_capturing = False
        self._safety_capturing = False
        self._safety_blocking = False
        self._preposition_ready = False
        self._waiting_for_safe_distance = False
        self._workflow_active = False
        self._measure_position_ready = False
        self._measure_position_ready_time = 0.0
        self._distance_safe_latched = False
        self._capture_home_phase = ""
        self._capture_complete = False
        self._selected_view_ready = False
        self._selected_view_ready_time = 0.0
        self._target_ctrl_on = False
        self._target_ready = False
        self._align_done = False
        self._output_on = False
        self._vision_valid = False
        self._recoverable_vision_loss_active = False
        self._fault_hold = False
        self._restart_blocked = False
        self._arm_ready_wait_started: Optional[float] = None
        self._ready_after_action = ""
        self._last_serial_heartbeat = 0.0
        self._last_serial_safe_distance = 0.0
        self._last_sent_safe_sample_time = 0.0
        self._last_serial_vision_error = 0.0
        self._last_sent_vision_sample_time = 0.0
        self._last_vision_gate_log_time = 0.0
        self._last_vision_gate_reason = ""
        self._last_vision_invalid_reason = ""
        self._vision_last_published_valid = False
        self._vision_invalid_transition_sent = False
        self._vision_transient_loss_started_at = 0.0
        self._latest_safe_depth_mm = 0.0
        self._latest_safe_depth_valid_ratio = 0.0
        self._latest_safe_depth_time = 0.0
        self._latest_safe_rgb_time = 0.0
        self._latest_safe_face_box = None
        self._latest_safe_face_size = (0, 0)
        self._latest_safe_measurement_valid = False
        self._latest_safe_invalid_reason = "等待机械臂伸展到公共测距位"
        self._safe_face_snapshots: Dict[int, SafeFaceSnapshot] = {}
        self._approved_safe_face_snapshot: Optional[SafeFaceSnapshot] = None
        self._latest_rgbd_detections = {}
        self._latest_rgbd_detection_time = 0.0
        self._latest_rgbd_rgb_size = (0, 0)
        self._last_rgbd_estimated_detection_time = 0.0
        self._last_rgbd_continuous_estimate_time = 0.0
        self._rgbd_continuous_stop_triggered = False
        self._latest_depth_preview: Optional[DepthPreview] = None
        self._scene_guard = FaceSceneGuard(
            center_shift_ratio=args.scene_center_shift_ratio,
            scale_ratio=args.scene_scale_ratio,
            missing_timeout_s=args.scene_missing_timeout,
        )
        self._scene_abort_triggered = False
        self._scene_monitor_degraded = False
        self._rgbd_distance_estimator = None
        if RgbdDistanceEstimator is not None:
            self._rgbd_distance_estimator = RgbdDistanceEstimator(
                arm_scale=args.rgbd_arm_bbox_scale,
                face_scale=args.rgbd_face_bbox_scale,
                arm_offset=(args.rgbd_arm_depth_offset_x, args.rgbd_arm_depth_offset_y),
                face_offset=(args.rgbd_face_depth_offset_x, args.rgbd_face_depth_offset_y),
                min_depth_mm=args.rgbd_min_depth_mm,
                max_depth_mm=args.rgbd_max_depth_mm,
                max_points=args.rgbd_max_points,
                measurement_noise_mm=args.rgbd_filter_measurement_noise_mm,
                process_noise_mm=args.rgbd_filter_process_noise_mm,
                jump_reset_mm=args.rgbd_filter_jump_reset_mm,
                max_sample_cover_radius_mm=args.rgbd_max_sample_cover_radius_mm,
            )
        self._latest_frame_image: Optional[QImage] = None
        self._latest_rgb_image: Optional[QImage] = None
        self._latest_frame_image_time = 0.0
        self._latest_rgb_image_time = 0.0
        self._capture_ready_frame_time = 0.0
        self._capture_images = {1: None, 2: None, 3: None}
        self._face_capture_root = Path(args.face_capture_dir).expanduser()
        self._capture_session_dir: Optional[Path] = None
        self._capture_file_paths: Dict[int, Path] = {}
        self._capture_persistence_errors: Dict[int, str] = {}
        self._triptych_detection_results = {}
        self._capture_sequence_active = False
        self._capture_sequence_manual = False
        self._capture_sequence_points: List[int] = []
        self._capture_sequence_index = 0
        self._capture_pending_point = 0
        self._capture_ready_frame_time = 0.0
        self._selected_capture_point = 0
        self._selected_triptych_target: Optional[DetectionTarget] = None
        self.face_image_labels = {}
        self.face_state_labels = {}
        self.face_select_buttons = {}
        self.face_candidate_combos = {}
        self._build_ui()
        self._connect_signals()
        self._switch_page(PAGE_WORKBENCH)
        self._update_fullscreen_button()
        if args.serial_port:
            self._set_status(self.arm.connection_text())
            self.serial_state.setText(self.arm.connection_text())
            self._add_history_text(self.arm.connection_text())
        self._refresh_controls()

        self.clock_timer = QTimer(self)
        self.clock_timer.timeout.connect(self._update_clock)
        self.clock_timer.start(1000)
        self._update_clock()

        self.arm_timer = QTimer(self)
        self.arm_timer.timeout.connect(self._poll_arm_status)
        self.arm_timer.start(50)

    def closeEvent(self, event) -> None:
        first_close = not self._closing_requested
        self._closing_requested = True
        if hasattr(self, "arm_timer"):
            self.arm_timer.stop()
        if hasattr(self, "clock_timer"):
            self.clock_timer.stop()
        if first_close:
            if self.arm.connected and (
                self._workflow_active
                or self.arm.control_busy
                or self._target_ctrl_on
                or self._target_ready
                or self._output_on
            ):
                ok, message = self.arm.abort_workflow()
                self._log_event("关闭界面前安全停止：{}".format(message))
                if not ok:
                    self._log_event("关闭界面前 ABORT_HOLD 发送失败，将立即停止心跳")
            # ABORT is written synchronously.  Closing the transport now also
            # stops the independent heartbeat, so a lost ABORT still reaches
            # RA6M5's watchdog instead of preserving a false healthy link.
            self.arm.close()
        self._stop_end_worker()
        self._stop_binocular_workers()
        if self.triptych_worker is not None and self.triptych_worker.isRunning():
            self.triptych_worker.requestInterruption()
            if not self.triptych_worker.wait(5000):
                self._set_status("仍在等待三视图识别线程安全退出")
        running_workers = [worker for worker in self._retired_workers if worker.isRunning()]
        if self.triptych_worker is not None and self.triptych_worker.isRunning():
            running_workers.append(self.triptych_worker)
        for worker in list(running_workers):
            worker.wait(5000)
        if any(worker.isRunning() for worker in running_workers):
            event.ignore()
            QTimer.singleShot(500, self.close)
            return
        if self._restart_requested and not self._restart_started:
            self._restart_started = True
            ok, message = self._launch_full_arm_restart()
            self._log_event(message)
        self.arm.close()
        super().closeEvent(event)

    def _launch_full_arm_restart(self) -> Tuple[bool, str]:
        script = ROOT / "arm_mole_qt" / "run_full_arm.sh"
        if not script.exists():
            return False, "刷新失败：未找到 {}".format(script)
        ok = QProcess.startDetached(
            "/usr/bin/env",
            ["DISPLAY=:0", "bash", str(script)],
            str(script.parent),
        )
        if ok:
            return True, "已重新启动 Qt：DISPLAY=:0 bash run_full_arm.sh"
        return False, "刷新失败：无法重新启动 run_full_arm.sh"

    def _request_application_restart(self) -> None:
        self._restart_requested = True
        self._set_status("正在刷新界面：关闭当前 Qt 后重新运行 run_full_arm.sh")
        self._log_event("用户请求刷新界面")
        self.close()

    def _retain_worker_until_finished(self, worker: QThread, label: str) -> None:
        if worker.wait(1500):
            worker.deleteLater()
            return
        if worker not in self._retired_workers:
            self._retired_workers.append(worker)
            worker.finished.connect(
                lambda finished_worker=worker: self._release_retired_worker(finished_worker)
            )
        self._log_event("{}仍在退出，已保留线程对象避免销毁崩溃".format(label))
        if not worker.isRunning():
            QTimer.singleShot(0, lambda: self._release_retired_worker(worker))

    def _release_retired_worker(self, worker: QThread) -> None:
        try:
            self._retired_workers.remove(worker)
        except ValueError:
            pass
        worker.deleteLater()

    @staticmethod
    def _disconnect_worker_signals(worker: QThread, signal_names) -> None:
        for name in signal_names:
            try:
                getattr(worker, name).disconnect()
            except (AttributeError, TypeError):
                pass

    def _create_end_worker(self) -> VideoWorker:
        args = self.args
        return VideoWorker(
            camera=args.camera,
            width=args.width,
            height=args.height,
            fps=args.fps,
            camera_brightness=args.camera_brightness,
            camera_gain=args.camera_gain,
            camera_backlight_compensation=args.camera_backlight_compensation,
            ui_fps=args.ui_fps,
            flip=args.flip,
            rotate=args.rotate,
            camera_layout=args.camera_layout,
            stereo_primary=args.stereo_primary,
            demo=args.demo,
            detector_mode=args.detector_mode,
            reference_path=self.reference_path,
            reference_frames=args.reference_frames,
            reference_warmup=args.reference_warmup,
            reference_params=self.reference_params,
            spot_params=self.spot_params,
            spot_roi=self.spot_roi,
            facial_face_model=Path(args.facial_face_model),
            facial_face_conf=args.facial_face_conf,
            facial_face_imgsz=args.facial_face_imgsz,
            facial_face_device=args.facial_face_device,
            facial_face_refresh_frames=args.facial_face_refresh_frames,
            facial_max_candidates=args.facial_max_candidates,
            facial_stable_frames=args.facial_stable_frames,
            facial_stable_seconds=args.facial_stable_seconds,
            facial_stable_radius=args.facial_stable_radius,
            facial_enhanced_filters=bool(args.facial_enhanced_filters),
            baseline_face_enabled=bool(args.baseline_face_enabled),
            baseline_face_dir=args.baseline_face_dir,
            safety_enabled=bool(args.safety_enabled),
            safety_image_path=self.safety_image_path,
            safety_reference_path=self.safety_reference_path,
            safety_warn_face_ratio=args.safety_warn_face_ratio,
            safety_danger_face_ratio=args.safety_danger_face_ratio,
            safety_warn_scale=args.safety_warn_scale,
            safety_danger_scale=args.safety_danger_scale,
        )

    def _start_end_worker(self) -> None:
        if self._closing_requested:
            return
        if self.worker is not None and self.worker.isRunning():
            return
        self.worker = self._create_end_worker()
        self.worker.frame_ready.connect(self._set_frame)
        self.worker.camera_preview_ready.connect(self._set_camera_preview)
        self.worker.result_ready.connect(self._set_result)
        self.worker.status_changed.connect(self._set_status)
        self.worker.reference_changed.connect(self._set_reference_state)
        self.worker.safety_changed.connect(self._set_safety_capture_state)
        self._latest_frame_image = None
        self._latest_rgb_image = None
        self._latest_frame_image_time = 0.0
        self._latest_rgb_image_time = 0.0
        self.current_result = None
        self._latest_result_time = 0.0
        self.video_label.clear()
        self.video_label.setText("正在打开末端视觉")
        self.worker.start()
        if self._detection_running:
            self.worker.start_detection()
        self._log_event("末端视觉已启动")

    def _stop_end_worker(self) -> None:
        if self.worker is None:
            return
        worker = self.worker
        worker.stop_detection()
        worker.cancel_reference_capture()
        worker.cancel_safety_capture()
        worker.stop()
        self._disconnect_worker_signals(
            worker,
            (
                "frame_ready",
                "camera_preview_ready",
                "result_ready",
                "status_changed",
                "reference_changed",
                "safety_changed",
            ),
        )
        self.worker = None
        self._retain_worker_until_finished(worker, "末端视觉")
        if hasattr(self, "video_label"):
            self.video_label.clear()
            self.video_label.setText("末端视觉未打开")
        self._latest_frame_image = None
        self._latest_rgb_image = None
        self._latest_frame_image_time = 0.0
        self._latest_rgb_image_time = 0.0
        self.current_result = None
        self._latest_result_time = 0.0
        self._log_event("末端视觉已关闭")

    def _start_binocular_workers(self) -> None:
        if self._closing_requested:
            return
        args = self.args
        started = False
        if self.astra_rgb_worker is None:
            self.astra_rgb_worker = AstraRgbWorker(
                enabled=bool(args.astra_rgb_enabled),
                camera=args.astra_rgb_camera,
                width=args.astra_rgb_width,
                height=args.astra_rgb_height,
                fps=args.astra_rgb_fps,
                ui_fps=args.ui_fps,
                flip=args.astra_rgb_flip,
                yolo_enabled=bool(args.astra_yolo_enabled),
                yolo_model=args.astra_yolo_model,
                yolo_conf=args.astra_yolo_conf,
                yolo_iou=args.astra_yolo_iou,
                yolo_imgsz=args.astra_yolo_imgsz,
                yolo_device=args.astra_yolo_device,
                yolo_arm_name=args.astra_yolo_arm_name,
                yolo_face_name=args.astra_yolo_face_name,
                yolo_interval=args.astra_yolo_interval,
            )
            self.astra_rgb_worker.preview_ready.connect(self._set_astra_rgb_preview)
            self.astra_rgb_worker.status_changed.connect(self._log_event)
            self.astra_rgb_worker.start()
            started = True
        if self.depth_worker is None:
            self.depth_worker = DepthWorker(
                enabled=bool(args.astra_depth_enabled),
                timeout_ms=args.depth_timeout_ms,
                rotate_180=args.depth_rotate_180,
                ui_fps=args.ui_fps,
                distortion_mode=args.astra_depth_distortion_mode,
            )
            self.depth_worker.depth_ready.connect(self._set_depth_preview)
            self.depth_worker.status_changed.connect(self._log_event)
            self.depth_worker.start()
            started = True
        if started:
            self._log_event("双目视觉已启动")

    def _stop_binocular_workers(self) -> None:
        if self.astra_rgb_worker is not None:
            worker = self.astra_rgb_worker
            worker.stop()
            self._disconnect_worker_signals(worker, ("preview_ready", "status_changed"))
            self.astra_rgb_worker = None
            self._retain_worker_until_finished(worker, "Astra RGB")
        if self.depth_worker is not None:
            worker = self.depth_worker
            worker.stop()
            self._disconnect_worker_signals(worker, ("depth_ready", "status_changed"))
            self.depth_worker = None
            self._retain_worker_until_finished(worker, "Astra Depth")
        self._latest_rgbd_detections = {}
        self._latest_rgbd_detection_time = 0.0
        self._latest_rgbd_rgb_size = (0, 0)
        self._latest_safe_depth_mm = 0.0
        self._latest_safe_depth_valid_ratio = 0.0
        self._latest_safe_depth_time = 0.0
        self._latest_safe_rgb_time = 0.0
        self._latest_safe_face_box = None
        self._latest_safe_face_size = (0, 0)
        self._latest_safe_measurement_valid = False
        self._latest_safe_invalid_reason = "双目视觉已关闭"
        self._last_rgbd_estimated_detection_time = 0.0
        self._latest_depth_preview = None
        self._last_sent_safe_sample_time = 0.0
        if self._rgbd_distance_estimator is not None:
            self._rgbd_distance_estimator.reset()
        if hasattr(self, "rgb_preview_label"):
            self.rgb_preview_label.clear()
            self.rgb_preview_label.setText("双目未打开")
        if hasattr(self, "depth_preview_label"):
            self.depth_preview_label.clear()
            self.depth_preview_label.setText("Depth 未打开")

    def keyPressEvent(self, event) -> None:
        if event.key() in (Qt.Key_Escape, Qt.Key_Q):
            self.close()
            return
        if event.key() == Qt.Key_F11:
            if self.isFullScreen():
                self.showNormal()
            else:
                self.showFullScreen()
            self._update_fullscreen_button()
            return
        if event.key() == Qt.Key_F6:
            if self.args.demo or self.args.protocol_debug_controls:
                self._simulate_workflow_ready()
            return
        if event.key() == Qt.Key_F7:
            self._capture_face_frame_with_f7()
            return
        super().keyPressEvent(event)

    def _build_ui(self) -> None:
        root = QWidget()
        root.setObjectName("root")
        shell = QHBoxLayout(root)
        shell.setContentsMargins(0, 0, 0, 0)
        shell.setSpacing(0)

        shell.addWidget(self._sidebar())

        content = QWidget()
        content.setObjectName("content")
        main = QVBoxLayout(content)
        main.setContentsMargins(8, 8, 8, 8)
        main.setSpacing(8)
        main.addWidget(self._header_bar())

        self.pages = QStackedWidget()
        self.pages.setObjectName("pages")
        self.pages.addWidget(self._overview_page())
        self.pages.addWidget(self._binocular_page())
        self.pages.addWidget(self._target_page())
        self.pages.addWidget(self._capture_page())
        self.pages.addWidget(self._scroll_page(self._device_page()))
        self.pages.addWidget(self._scroll_page(self._logs_page()))
        main.addWidget(self.pages, stretch=1)
        shell.addWidget(content, stretch=1)

        self.setCentralWidget(root)
        self.setStyleSheet(STYLE)

    def _sidebar(self) -> QWidget:
        sidebar = QWidget()
        sidebar.setObjectName("sidebar")
        sidebar.setFixedWidth(132)
        layout = QVBoxLayout(sidebar)
        layout.setContentsMargins(8, 10, 8, 10)
        layout.setSpacing(8)

        brand = QLabel("智肤定靶")
        brand.setObjectName("brand")
        brand_subtitle = QLabel("Vision Arm")
        brand_subtitle.setObjectName("brandSubtitle")
        layout.addWidget(brand)
        layout.addWidget(brand_subtitle)
        layout.addSpacing(12)

        self.nav_buttons = []
        for index, text in enumerate(("工作台", "双目安全", "末端视觉", "三视图", "设备", "记录")):
            button = QPushButton(text)
            button.setObjectName("navButton")
            button.setCheckable(True)
            button.setMinimumHeight(38)
            button.clicked.connect(lambda _checked=False, page=index: self._switch_page(page))
            self.nav_buttons.append(button)
            layout.addWidget(button)

        layout.addStretch(1)
        self.restart_btn = QPushButton("刷新界面")
        self.restart_btn.setObjectName("navButton")
        self.restart_btn.setMinimumHeight(36)
        self.restart_btn.setToolTip("关闭当前 Qt 并重新运行 run_full_arm.sh。")
        layout.addWidget(self.restart_btn)
        self.exit_btn = QPushButton("退出")
        self.exit_btn.setObjectName("navExitButton")
        self.exit_btn.setMinimumHeight(36)
        layout.addWidget(self.exit_btn)
        return sidebar

    def _header_bar(self) -> QWidget:
        header = QWidget()
        header.setObjectName("header")
        layout = QHBoxLayout(header)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)
        title_box = QVBoxLayout()
        title_box.setSpacing(1)
        title = QLabel("机械臂点痣装置")
        title.setObjectName("title")
        subtitle = QLabel("三视图采集 / 单目定靶 / RA6M5 联动")
        subtitle.setObjectName("subtitle")
        title_box.addWidget(title)
        title_box.addWidget(subtitle)
        layout.addLayout(title_box)
        layout.addStretch(1)
        self.status_label = QLabel("正在启动")
        self.status_label.setObjectName("statusPill")
        self.status_label.setMaximumWidth(230)
        self.fullscreen_btn = QPushButton("全屏")
        self.fullscreen_btn.setObjectName("secondary")
        self.fullscreen_btn.setMinimumSize(58, 32)
        self.fullscreen_btn.setMaximumWidth(72)
        self.fullscreen_btn.setToolTip("切换全屏显示，也可以按 F11。")
        self.clock_label = QLabel("--:--:--")
        self.clock_label.setObjectName("clock")
        self.clock_label.setMinimumWidth(136)
        self.clock_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        layout.addWidget(self.status_label)
        layout.addWidget(self.fullscreen_btn)
        layout.addWidget(self.clock_label)
        return header

    def _overview_page(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(10)

        metrics = QHBoxLayout()
        metrics.setSpacing(8)
        self.summary_target_value = self._metric_card(metrics, "目标数", "0", "当前识别")
        self.summary_fps_value = self._metric_card(metrics, "画面 FPS", "--", "RGB 采集")
        self.summary_safety_value = self._metric_card(metrics, "安全状态", "未启用", "近距离联锁")
        self.summary_depth_value = self._metric_card(metrics, "中心深度", "--", "Astra Depth")
        layout.addLayout(metrics)

        body = QHBoxLayout()
        body.setSpacing(10)
        left = QVBoxLayout()
        left.setSpacing(10)
        left.addWidget(self._workflow_group())
        left.addWidget(self._run_command_group())
        left.addStretch(1)
        body.addLayout(left, stretch=1)

        right = QVBoxLayout()
        right.setSpacing(10)
        right.addWidget(self._overview_steps_group())
        right.addWidget(self._overview_summary_group())
        right.addStretch(1)
        body.addLayout(right, stretch=2)
        layout.addLayout(body, stretch=1)
        return page

    def _binocular_page(self) -> QWidget:
        page = QWidget()
        page.setObjectName("pageSurface")
        layout = QHBoxLayout(page)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)
        layout.addWidget(self._camera_group(), stretch=4)
        side_panel = QWidget()
        side_panel.setObjectName("sidePanel")
        side_panel.setMaximumWidth(248)
        side = QVBoxLayout(side_panel)
        side.setContentsMargins(0, 0, 0, 0)
        side.setSpacing(8)
        side.addWidget(self._binocular_status_group(), stretch=1)
        side.addWidget(self._depth_info_group(), stretch=1)
        side.addStretch(1)
        layout.addWidget(side_panel, stretch=0)
        return page

    def _capture_page(self) -> QWidget:
        page = QWidget()
        page.setObjectName("pageSurface")
        layout = QVBoxLayout(page)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)

        board = QGroupBox("三视图人脸图")
        board_layout = QVBoxLayout(board)
        board_layout.setContentsMargins(8, 8, 8, 8)
        board_layout.setSpacing(6)
        strip = QHBoxLayout()
        strip.setSpacing(6)
        for point_id, title in ((1, "左脸"), (2, "正脸"), (3, "右脸")):
            strip.addWidget(self._face_capture_card(point_id, title), stretch=1)
        board_layout.addLayout(strip)
        layout.addWidget(board, stretch=2)
        layout.addWidget(self._capture_protocol_group())
        return page

    def _target_page(self) -> QWidget:
        page = QWidget()
        page.setObjectName("pageSurface")
        layout = QHBoxLayout(page)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(10)

        vision_panel = QGroupBox("单目定靶")
        vision_layout = QVBoxLayout(vision_panel)
        vision_layout.setContentsMargins(8, 10, 8, 8)
        self.video_label = QLabel("等待摄像头画面")
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setMinimumSize(520, 350)
        self.video_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.video_label.setObjectName("video")
        vision_layout.addWidget(self.video_label)
        layout.addWidget(vision_panel, stretch=3)

        side = QVBoxLayout()
        side.setSpacing(8)
        side.addWidget(self._result_group())
        side.addWidget(self._control_group())
        side.addStretch(1)
        layout.addLayout(side, stretch=1)
        return page

    def _device_page(self) -> QWidget:
        page = QWidget()
        layout = QHBoxLayout(page)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(10)
        layout.addWidget(self._device_info_group(), stretch=1)
        layout.addWidget(self._workflow_protocol_group(), stretch=1)
        return page

    def _run_command_group(self) -> QGroupBox:
        box = QGroupBox("流程总控")
        layout = QGridLayout(box)
        layout.setHorizontalSpacing(8)
        layout.setVerticalSpacing(8)

        self.start_btn = QPushButton("开始")
        self.capture_faces_btn = QPushButton("采集人脸图")
        self.finish_btn = QPushButton("复位/返回")
        self.stop_btn = QPushButton("取消/安全停止")
        self.estop_btn = QPushButton("硬件急停说明")
        self.start_btn.setObjectName("primaryAction")
        self.capture_faces_btn.setObjectName("primaryAction")
        self.stop_btn.setObjectName("secondary")
        self.estop_btn.setObjectName("danger")
        self.start_btn.setToolTip(
            "发送 START_MEASURE（action=0x01）；等待公共测距位和本轮安全距离锁存。"
        )
        self.capture_faces_btn.setToolTip("安全锁存后自动执行 HOME→左/正/右采图→HOME。")
        self.finish_btn.setToolTip(
            "不管是否已经点痣，均发送 WORKFLOW_CTRL FINISH_RETURN_HOME（action=0x02），"
            "请求 RA6M5 soft_reset 并返回 HOME。"
        )
        self.stop_btn.setToolTip("发送 ABORT_HOLD：立即关闭输出、中止运动并原地保持，不自动 HOME。")
        self.estop_btn.setToolTip("显示独立硬件急停说明；软件按钮不能代替硬件急停。")

        buttons = (
            self.start_btn,
            self.capture_faces_btn,
            self.finish_btn,
            self.stop_btn,
            self.estop_btn,
        )
        for index, button in enumerate(buttons):
            button.setMinimumHeight(42)
            layout.addWidget(button, index // 2, index % 2)
        return box

    def _binocular_status_group(self) -> QGroupBox:
        box = QGroupBox("RGB-D 安全状态")
        layout = QGridLayout(box)
        layout.setHorizontalSpacing(10)
        layout.setVerticalSpacing(6)
        self.binocular_yolo_state = self._value_label("等待 YOLO 识别结果")
        self.binocular_center_depth_value = self._value_label("--")
        self.binocular_safe_distance_value = self._value_label("等待开始并伸展到公共测距位")
        self.binocular_capture_faces_btn = QPushButton("采集人脸图")
        self.binocular_capture_faces_btn.setObjectName("primaryAction")
        self.binocular_capture_faces_btn.setMinimumHeight(36)
        self.binocular_capture_faces_btn.setToolTip("安全距离锁存后，自动完成 HOME、左/正/右采图和最终 HOME。")
        rows = [
            ("YOLO 状态", self.binocular_yolo_state),
            ("Depth 中心 Z", self.binocular_center_depth_value),
            ("arm-face 净距", self.binocular_safe_distance_value),
            ("当前阶段", self._value_label("双目安全监控")),
        ]
        for row, (name, value) in enumerate(rows):
            label = QLabel(name)
            label.setObjectName("fieldName")
            layout.addWidget(label, row, 0)
            layout.addWidget(value, row, 1)
        layout.addWidget(self.binocular_capture_faces_btn, len(rows), 0, 1, 2)
        return box

    def _workflow_protocol_group(self) -> QGroupBox:
        box = QGroupBox("流程联动")
        layout = QGridLayout(box)
        layout.setHorizontalSpacing(10)
        layout.setVerticalSpacing(6)
        rows = [
            ("开始", self._value_label("双目安全")),
            ("采集人脸图", self._value_label("末端视觉")),
            ("采集完成", self._value_label("三视图")),
            ("选择视图", self._value_label("末端视觉")),
        ]
        for row, (name, value) in enumerate(rows):
            label = QLabel(name)
            label.setObjectName("fieldName")
            layout.addWidget(label, row, 0)
            layout.addWidget(value, row, 1)
        return box

    def _overview_steps_group(self) -> QGroupBox:
        box = QGroupBox("主流程")
        layout = QHBoxLayout(box)
        layout.setContentsMargins(8, 12, 8, 8)
        layout.setSpacing(8)
        steps = (
            ("1", "开始", "机械臂到预定位"),
            ("2", "采图", "左中右三视图"),
            ("3", "选择", "医生确认位置"),
            ("4", "定靶", "识别坐标并执行"),
        )
        self.step_labels = []
        for number, title, caption in steps:
            card = QFrame()
            card.setObjectName("stepCard")
            card.setMinimumHeight(104)
            card_layout = QVBoxLayout(card)
            card_layout.setContentsMargins(10, 8, 10, 8)
            card_layout.setSpacing(2)
            number_label = QLabel(number)
            number_label.setObjectName("stepNumber")
            title_label = QLabel(title)
            title_label.setObjectName("stepTitle")
            caption_label = QLabel(caption)
            caption_label.setObjectName("stepCaption")
            card_layout.addWidget(number_label)
            card_layout.addWidget(title_label)
            card_layout.addWidget(caption_label)
            card_layout.addStretch(1)
            self.step_labels.append(title_label)
            layout.addWidget(card, stretch=1)
        return box

    def _overview_summary_group(self) -> QGroupBox:
        box = QGroupBox("当前任务")
        layout = QGridLayout(box)
        layout.setHorizontalSpacing(10)
        layout.setVerticalSpacing(6)
        self.selected_view_value = self._value_label("未选择")
        self.capture_progress_value = self._value_label("三视图未采集")
        rows = [
            ("选定视图", self.selected_view_value),
            ("采集进度", self.capture_progress_value),
            ("定靶策略", self._value_label("机械臂稳定到位后执行单次黑痣识别")),
            ("执行边界", self._value_label("Qt 发视觉建议，RA6M5 负责运动和激光许可")),
        ]
        for row, (name, value) in enumerate(rows):
            label = QLabel(name)
            label.setObjectName("fieldName")
            layout.addWidget(label, row, 0)
            layout.addWidget(value, row, 1)
        return box

    def _face_capture_card(self, point_id: int, title: str) -> QFrame:
        card = QFrame()
        card.setObjectName("captureCard")
        card.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        layout = QVBoxLayout(card)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)

        title_label = QLabel(title)
        title_label.setObjectName("captureTitle")
        image_label = QLabel("{}未采集".format(title))
        image_label.setAlignment(Qt.AlignCenter)
        image_label.setMinimumSize(210, 130)
        image_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        image_label.setObjectName("facePreview")
        state_label = QLabel("未采集")
        state_label.setObjectName("captureState")
        candidate_combo = QComboBox()
        candidate_combo.addItem("请选择候选暗斑", None)
        candidate_combo.setToolTip("候选检查仅供协议调试；正式流程由医生只选择视图")
        candidate_combo.setVisible(bool(self.args.protocol_debug_controls))
        select_button = QPushButton(self._point_short_name(point_id))
        select_button.setMinimumHeight(36)
        select_button.setToolTip("选择{}视图；不在静态图片中指定具体痣点".format(title))

        self.face_image_labels[point_id] = image_label
        self.face_state_labels[point_id] = state_label
        self.face_select_buttons[point_id] = select_button
        self.face_candidate_combos[point_id] = candidate_combo

        layout.addWidget(title_label)
        layout.addWidget(image_label, stretch=1)
        layout.addWidget(state_label)
        layout.addWidget(candidate_combo)
        layout.addWidget(select_button)
        return card

    def _logs_page(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(10)
        layout.addWidget(self._history_group(max_height=420))
        return page

    def _scroll_page(self, widget: QWidget) -> QScrollArea:
        scroll = QScrollArea()
        scroll.setWidget(widget)
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        QScroller.grabGesture(scroll.viewport(), QScroller.TouchGesture)
        QScroller.grabGesture(scroll.viewport(), QScroller.LeftMouseButtonGesture)
        return scroll

    def _metric_card(self, layout: QHBoxLayout, title: str, value: str, caption: str) -> QLabel:
        card = QFrame()
        card.setObjectName("metricCard")
        card.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        card.setMinimumHeight(84)
        card.setMaximumHeight(92)
        card_layout = QVBoxLayout(card)
        card_layout.setContentsMargins(10, 6, 10, 6)
        card_layout.setSpacing(1)
        name_label = QLabel(title)
        name_label.setObjectName("metricName")
        name_label.setMinimumHeight(15)
        value_label = QLabel(value)
        value_label.setObjectName("metricValue")
        value_label.setMinimumHeight(28)
        caption_label = QLabel(caption)
        caption_label.setObjectName("metricCaption")
        caption_label.setMinimumHeight(15)
        card_layout.addWidget(name_label)
        card_layout.addWidget(value_label)
        card_layout.addWidget(caption_label)
        layout.addWidget(card)
        return value_label

    def _control_group(self) -> QGroupBox:
        box = QGroupBox("定靶操作")
        controls = QGridLayout(box)
        controls.setHorizontalSpacing(6)
        controls.setVerticalSpacing(6)
        self.end_capture_faces_btn = QPushButton("采集人脸图")
        self.capture_reference_btn = QPushButton("采集基准图")
        self.capture_safety_btn = QPushButton("采集安全距离图")
        self.once_btn = QPushButton("稳定识别一次")
        self.preview_btn = QPushButton("预定位")
        self.fire_btn = QPushButton("开始点痣")
        self.end_finish_btn = QPushButton("复位/返回 HOME")
        self.end_stop_btn = QPushButton("取消/安全停止")
        self.end_capture_faces_btn.setObjectName("primaryAction")
        self.fire_btn.setObjectName("danger")
        self.end_finish_btn.setObjectName("primaryAction")
        self.end_stop_btn.setObjectName("secondary")
        self.end_capture_faces_btn.setToolTip("正式流程会在各拍摄点到位后的首个新鲜画面自动采集。")
        self.capture_reference_btn.setToolTip("无黑痣时采集多帧中值基准图。安全距离功能默认关闭，后续需要时再启用。")
        self.capture_reference_btn.setVisible(
            self.args.detector_mode in ("auto", "reference")
        )
        self.capture_safety_btn.setToolTip("采集当前距离作为安全距离图；采集期间不触发红框，采完后人脸更靠近才判危险。")
        self.once_btn.setToolTip("机械臂稳定后，只对当前画面执行一次本地识别。")
        self.once_btn.setVisible(bool(self.args.protocol_debug_controls))
        self.preview_btn.setToolTip("协议调试：手动发送一次视觉误差；正式流程隐藏。")
        self.fire_btn.setToolTip(
            "发送 TARGET_CTRL START；收到 TARGET_READY 后才周期发送视觉误差；"
            "未稳定锁定时发送 valid=0。"
        )
        self.end_finish_btn.setToolTip(
            "不管是否已经点痣，均发送 WORKFLOW_CTRL FINISH_RETURN_HOME（action=0x02），"
            "停止视觉误差并等待 RETURN_HOME_DONE。"
        )
        self.end_stop_btn.setToolTip(
            "发送 ABORT_HOLD：立即关闭输出、中止运动并原地保持，不自动 HOME。"
        )
        self.preview_btn.setVisible(bool(self.args.protocol_debug_controls))

        for btn in (
            self.end_capture_faces_btn,
            self.capture_reference_btn,
            self.capture_safety_btn,
            self.once_btn,
            self.preview_btn,
            self.fire_btn,
            self.end_finish_btn,
            self.end_stop_btn,
        ):
            btn.setMinimumHeight(34)
        controls.addWidget(self.end_capture_faces_btn, 0, 0, 1, 2)
        controls.addWidget(self.capture_reference_btn, 1, 0, 1, 2)
        controls.addWidget(self.capture_safety_btn, 2, 0, 1, 2)
        controls.addWidget(self.once_btn, 3, 0, 1, 2)
        controls.addWidget(self.preview_btn, 4, 0)
        controls.addWidget(self.fire_btn, 4, 1)
        controls.addWidget(self.end_finish_btn, 5, 0, 1, 2)
        controls.addWidget(self.end_stop_btn, 6, 0, 1, 2)
        return box

    def _camera_group(self) -> QGroupBox:
        box = QGroupBox("Astra Pro Plus / RGB-D 视图")
        layout = QGridLayout(box)
        layout.setHorizontalSpacing(8)
        layout.setVerticalSpacing(5)

        rgb_title = QLabel("RGB 主视图")
        depth_title = QLabel("Depth 伪彩")
        rgb_title.setObjectName("fieldName")
        depth_title.setObjectName("fieldName")
        self.rgb_preview_label = QLabel("等待 RGB")
        self.depth_preview_label = QLabel("Depth 未启用")
        for label in (self.rgb_preview_label, self.depth_preview_label):
            label.setAlignment(Qt.AlignCenter)
            label.setMinimumSize(220, 136)
            label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
            label.setObjectName("miniVideo")

        self.camera_layout_value = self._value_label("单目")
        self.camera_primary_value = self._value_label("RGB")
        self.camera_raw_size_value = self._value_label("--")
        self.camera_view_size_value = self._value_label("--")
        self.camera_message_value = self._value_label("等待摄像头")
        self.depth_state_value = self._value_label("未启用")
        self.depth_size_value = self._value_label("--")
        self.depth_center_value = self._value_label("--")
        self.depth_valid_value = self._value_label("--")
        self.depth_fps_value = self._value_label("--")

        layout.addWidget(rgb_title, 0, 0)
        layout.addWidget(depth_title, 0, 1)
        layout.addWidget(self.rgb_preview_label, 1, 0)
        layout.addWidget(self.depth_preview_label, 1, 1)

        rows = [
            ("布局", self.camera_layout_value),
            ("主视图", self.camera_primary_value),
            ("RGB 输入", self.camera_raw_size_value),
            ("识别视图", self.camera_view_size_value),
            ("相机状态", self.camera_message_value),
            ("Depth 状态", self.depth_state_value),
            ("Depth 尺寸", self.depth_size_value),
            ("中心深度", self.depth_center_value),
            ("有效深度", self.depth_valid_value),
            ("Depth FPS", self.depth_fps_value),
        ]
        for index, (name, value) in enumerate(rows, start=2):
            label = QLabel(name)
            label.setObjectName("fieldName")
            col = 0 if index % 2 == 0 else 1
            row = 2 + (index - 2) // 2
            layout.addWidget(label, row, col * 2)
            layout.addWidget(value, row, col * 2 + 1)
        return box

    def _device_info_group(self) -> QGroupBox:
        box = QGroupBox("设备链路")
        layout = QGridLayout(box)
        layout.setHorizontalSpacing(10)
        layout.setVerticalSpacing(6)
        args = self.args
        rows = [
            ("RGB 摄像头", self._value_label(str(args.camera))),
            ("RGB 分辨率", self._value_label("{}x{} @ {}fps".format(args.width, args.height, args.fps))),
            ("相机模型", self._value_label("Astra Pro Plus RGB-D" if args.camera_layout == "astra-rgbd" else "普通 RGB")),
            ("视觉工程", self._value_label(str(VISION_ROOT))),
            ("串口状态", self._value_label(self.arm.connection_text())),
            ("激光偏置", self._value_label(self.arm.laser_offset_text())),
        ]
        for row, (name, value) in enumerate(rows):
            label = QLabel(name)
            label.setObjectName("fieldName")
            layout.addWidget(label, row, 0)
            layout.addWidget(value, row, 1)
        return box

    def _depth_info_group(self) -> QGroupBox:
        box = QGroupBox("深度感知")
        layout = QGridLayout(box)
        layout.setHorizontalSpacing(10)
        layout.setVerticalSpacing(6)
        rows = [
            ("Depth 来源", self._value_label("pyorbbecsdk / ORBBEC Depth Sensor")),
            ("融合方式", self._value_label("RGB 负责识别，Depth 负责距离和空间约束")),
            ("主控原则", self._value_label("Jetson 给视觉建议，RA6M5 执行许可")),
            ("当前用途", self._value_label("显示中心 Z 和深度有效率，后续接入三维预定位")),
        ]
        for row, (name, value) in enumerate(rows):
            label = QLabel(name)
            label.setObjectName("fieldName")
            layout.addWidget(label, row, 0)
            layout.addWidget(value, row, 1)
        return box

    def _switch_page(self, index: int) -> None:
        if self._closing_requested or not hasattr(self, "pages"):
            return
        target_start_wait = (
            self._waiting_for_arm_ready and self._ready_after_action == "target_start"
        )
        if index != PAGE_END_CAMERA and (
            self._selected_view_ready
            or self._target_ctrl_on
            or self._target_ready
            or self._output_on
            or target_start_wait
        ):
            self._set_status("定靶/输出阶段已锁定末端视觉页；请先完成或安全停止")
            return
        self.pages.setCurrentIndex(index)
        for button_index, button in enumerate(getattr(self, "nav_buttons", [])):
            button.setChecked(button_index == index)
        self._activate_page_resources(index)
        self._refresh_controls()

    def _activate_page_resources(self, index: int) -> None:
        if self._closing_requested:
            return
        keep_astra_running = self._keep_astra_rgbd_running()
        keep_scene_monitor = (
            self._workflow_active
            and self._scene_guard.active
            and not self._scene_abort_triggered
            and not self._fault_hold
        )
        if index == PAGE_BINOCULAR:
            self._stop_end_worker()
            self._start_binocular_workers()
            return
        if index == PAGE_END_CAMERA:
            if keep_astra_running or keep_scene_monitor:
                self._start_binocular_workers()
            else:
                self._stop_binocular_workers()
            self._start_end_worker()
            return
        if keep_astra_running or keep_scene_monitor:
            self._start_binocular_workers()
        else:
            self._stop_binocular_workers()
        self._stop_end_worker()

    def _keep_astra_rgbd_running(self) -> bool:
        return bool(
            not self.args.demo
            and self.args.camera_layout == "astra-rgbd"
            and self.args.astra_always_on
            and (self.args.astra_rgb_enabled or self.args.astra_depth_enabled)
        )

    def _workflow_group(self) -> QGroupBox:
        box = QGroupBox("运行流程")
        box.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Maximum)
        layout = QGridLayout(box)
        layout.setHorizontalSpacing(10)
        layout.setVerticalSpacing(2)

        self.workflow_state = self._value_label("待机")
        self.serial_state = self._value_label(self.arm.connection_text())
        self.ra6_state = self._value_label("未收到" if self.arm.connected else "未连接")
        self.ready_wait_value = self._value_label("--")
        self.reference_state = self._value_label(
            "人脸候选检测" if self.args.detector_mode == "facial_candidate" else "未采集"
        )
        self.safety_state = self._value_label("安全距离关闭")

        rows = [
            ("流程阶段", self.workflow_state),
            ("串口", self.serial_state),
            ("RA6M5 状态", self.ra6_state),
            ("等待 READY", self.ready_wait_value),
            ("检测器", self.reference_state),
            ("安全状态", self.safety_state),
        ]
        for row, (name, value) in enumerate(rows):
            label = QLabel(name)
            label.setObjectName("fieldName")
            layout.addWidget(label, row, 0)
            layout.addWidget(value, row, 1)
        return box

    def _capture_protocol_group(self) -> QGroupBox:
        box = QGroupBox("三视图协议")
        layout = QGridLayout(box)
        layout.setHorizontalSpacing(6)
        layout.setVerticalSpacing(4)

        self.capture_left_btn = QPushButton("左拍摄点")
        self.capture_center_btn = QPushButton("正拍摄点")
        self.capture_right_btn = QPushButton("右拍摄点")
        self.capture_home_btn = QPushButton("拍完回 HOME")
        self.select_left_btn = QPushButton("选左视图")
        self.select_center_btn = QPushButton("选正视图")
        self.select_right_btn = QPushButton("选右视图")

        buttons = (
            self.capture_left_btn,
            self.capture_center_btn,
            self.capture_right_btn,
            self.capture_home_btn,
            self.select_left_btn,
            self.select_center_btn,
            self.select_right_btn,
        )
        for index, button in enumerate(buttons):
            button.setMinimumHeight(32)
            layout.addWidget(button, index // 4, index % 4)
        return box

    def _result_group(self) -> QGroupBox:
        box = QGroupBox("识别结果")
        box.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Maximum)
        layout = QGridLayout(box)
        layout.setHorizontalSpacing(10)
        layout.setVerticalSpacing(2)

        self.detect_state = self._value_label("未识别")
        self.target_count = self._value_label("0")
        self.pixel_coord = self._value_label("--")
        self.robot_coord = self._value_label("未接入")
        self.laser_offset_value = self._value_label(self.arm.laser_offset_text())
        self.radius_value = self._value_label("--")
        self.score_value = self._value_label("--")
        self.fps_value = self._value_label("--")
        self.mode_value = self._value_label("预览")

        rows = [
            ("状态", self.detect_state),
            ("目标数", self.target_count),
            ("像素坐标", self.pixel_coord),
            ("视觉误差", self.robot_coord),
            ("激光偏置", self.laser_offset_value),
            ("半径", self.radius_value),
            ("对比度", self.score_value),
            ("画面 FPS", self.fps_value),
            ("模式", self.mode_value),
        ]
        for row, (name, value) in enumerate(rows):
            label = QLabel(name)
            label.setObjectName("fieldName")
            layout.addWidget(label, row, 0)
            layout.addWidget(value, row, 1)
        return box

    def _history_group(self, max_height: Optional[int] = None) -> QGroupBox:
        box = QGroupBox("最近记录")
        layout = QVBoxLayout(box)
        layout.setContentsMargins(6, 8, 6, 6)
        self.history_list = QListWidget()
        self.history_list.setObjectName("history")
        if max_height is not None:
            self.history_list.setMinimumHeight(min(180, max_height))
            self.history_list.setMaximumHeight(max_height)
        else:
            self.history_list.setMaximumHeight(120)
        layout.addWidget(self.history_list)
        return box

    def _value_label(self, text: str) -> QLabel:
        label = QLabel(text)
        label.setObjectName("fieldValue")
        label.setWordWrap(True)
        label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        return label

    def _connect_signals(self) -> None:
        self.capture_reference_btn.clicked.connect(self._capture_reference)
        self.capture_safety_btn.clicked.connect(self._capture_safety_distance)
        self.start_btn.clicked.connect(self._start_workflow)
        self.finish_btn.clicked.connect(self._finish_workflow)
        self.end_finish_btn.clicked.connect(self._finish_workflow)
        self.capture_faces_btn.clicked.connect(self._capture_faces_button_clicked)
        self.binocular_capture_faces_btn.clicked.connect(self._capture_faces_button_clicked)
        self.end_capture_faces_btn.clicked.connect(self._capture_faces_button_clicked)
        self.once_btn.clicked.connect(self._detect_once)
        self.stop_btn.clicked.connect(self._cancel_workflow)
        self.end_stop_btn.clicked.connect(self._cancel_workflow)
        self.preview_btn.clicked.connect(self._preview_move)
        self.fire_btn.clicked.connect(self._fire_laser)
        self.estop_btn.clicked.connect(self._emergency_stop)
        self.fullscreen_btn.clicked.connect(self._toggle_fullscreen)
        self.restart_btn.clicked.connect(self._request_application_restart)
        self.capture_left_btn.clicked.connect(lambda: self._send_capture_move(1))
        self.capture_center_btn.clicked.connect(lambda: self._send_capture_move(2))
        self.capture_right_btn.clicked.connect(lambda: self._send_capture_move(3))
        self.capture_home_btn.clicked.connect(self._send_capture_home)
        self.select_left_btn.clicked.connect(lambda: self._send_capture_select(1))
        self.select_center_btn.clicked.connect(lambda: self._send_capture_select(2))
        self.select_right_btn.clicked.connect(lambda: self._send_capture_select(3))
        for point_id, button in self.face_select_buttons.items():
            button.clicked.connect(lambda _checked=False, point=point_id: self._send_capture_select(point))
        for button in (
            self.capture_left_btn,
            self.capture_center_btn,
            self.capture_right_btn,
            self.capture_home_btn,
            self.select_left_btn,
            self.select_center_btn,
            self.select_right_btn,
        ):
            button.setVisible(bool(self.args.protocol_debug_controls))
        self.exit_btn.clicked.connect(self.close)

    def _toggle_fullscreen(self) -> None:
        if self.isFullScreen():
            self.showNormal()
        else:
            self.showFullScreen()
        self._update_fullscreen_button()

    def _update_fullscreen_button(self) -> None:
        if not hasattr(self, "fullscreen_btn"):
            return
        if self.isFullScreen():
            self.fullscreen_btn.setText("窗口")
            self.fullscreen_btn.setToolTip("退出全屏显示，也可以按 F11。")
        else:
            self.fullscreen_btn.setText("全屏")
            self.fullscreen_btn.setToolTip("切换全屏显示，也可以按 F11。")

    def _set_frame(self, sample: object) -> None:
        if isinstance(sample, ImageSample):
            image = sample.image
            raw_image = sample.raw_image if sample.raw_image is not None else image
            sample_time = float(sample.sample_time)
        else:
            image = sample
            raw_image = image
            sample_time = time.monotonic()
        # Keep the rendered privacy/overlay image on screen, but retain the
        # untouched camera frame for face/triptych capture and re-detection.
        self._latest_frame_image = raw_image
        self._latest_frame_image_time = sample_time
        self._set_label_image(
            self.video_label,
            image,
            "等待摄像头画面",
            allow_upscale=False,
            smooth=False,
        )
        self._capture_fresh_frame_if_ready(sample_time)

    def _set_label_image(
        self,
        label: QLabel,
        image: Optional[QImage],
        empty_text: str,
        allow_upscale: bool = True,
        smooth: bool = True,
    ) -> None:
        if image is None:
            label.clear()
            label.setText(empty_text)
            return
        pixmap = QPixmap.fromImage(image)
        target_size = label.size()
        if not allow_upscale:
            target_size = target_size.boundedTo(QSize(pixmap.width(), pixmap.height()))
        scaled = pixmap.scaled(
            target_size,
            Qt.KeepAspectRatio,
            Qt.SmoothTransformation if smooth else Qt.FastTransformation,
        )
        label.setPixmap(scaled)

    def _rgbd_clearance_estimation_enabled(self) -> bool:
        """Return whether arm-to-face clearance is meaningful in this phase.

        At HOME the arm tip is intentionally outside the Astra view, so an
        ``arm_tip + face`` estimate can only produce expected INVALID noise.
        The initial estimate starts after RA6M5 reports
        ``MEASURE_POSITION_READY``.  Once the distance is latched, the same
        verified geometry remains active throughout the workflow for the
        local sub-120 mm abort guard.
        """

        if not bool(self.__dict__.get("_workflow_active", False)):
            return False
        if bool(self.__dict__.get("_distance_safe_latched", False)):
            return (
                float(self.__dict__.get("rgbd_continuous_stop_mm", 0.0) or 0.0)
                > 0.0
            )
        return bool(
            self.__dict__.get("_measure_position_ready", False)
            and self.__dict__.get("_waiting_for_safe_distance", False)
        )

    def _claim_rgbd_clearance_estimate(self, now: Optional[float] = None) -> bool:
        """Reserve one fresh RGB detection for a single RGB-D estimate.

        Before the formal distance latch, every new YOLO observation remains
        eligible for the protocol-v5 SAFE_DISTANCE phase.  After the latch,
        SAFE_DISTANCE transmission stays disabled, but the verified RGB-D
        geometry can continue at a bounded rate for the local emergency stop.
        """

        if not self._rgbd_clearance_estimation_enabled():
            return False

        detection_time = float(
            self.__dict__.get("_latest_rgbd_detection_time", 0.0) or 0.0
        )
        last_detection_time = float(
            self.__dict__.get("_last_rgbd_estimated_detection_time", 0.0)
            or 0.0
        )
        if detection_time <= 0.0 or detection_time <= last_detection_time:
            return False

        if self.__dict__.get("_distance_safe_latched", False):
            threshold_mm = max(
                0.0,
                float(self.__dict__.get("rgbd_continuous_stop_mm", 0.0) or 0.0),
            )
            if threshold_mm <= 0.0 or not self.__dict__.get("_workflow_active", False):
                return False
            check_time = time.monotonic() if now is None else float(now)
            interval = max(
                0.10,
                float(
                    self.__dict__.get("rgbd_continuous_guard_interval", 0.20)
                    or 0.20
                ),
            )
            last_estimate_time = float(
                self.__dict__.get("_last_rgbd_continuous_estimate_time", 0.0)
                or 0.0
            )
            if check_time - last_estimate_time < interval:
                return False
            self._last_rgbd_continuous_estimate_time = check_time

        self._last_rgbd_estimated_detection_time = detection_time
        return True

    def _set_astra_rgb_preview(self, preview: AstraRgbPreview) -> None:
        # Astra remains active across pages for safety/scene metadata, but a
        # hidden page must not spend GUI time converting and scaling QPixmaps.
        # The previous unconditional render contributed to queued-event lag
        # while the end-camera page was running.
        pages = self.__dict__.get("pages")
        if pages is not None and pages.currentIndex() == PAGE_BINOCULAR:
            self._set_label_image(
                self.rgb_preview_label,
                preview.image,
                "等待 Astra RGB",
                smooth=False,
            )
        if not preview.available:
            self._latest_rgbd_detections = {}
            self._latest_rgbd_detection_time = 0.0
            self._latest_rgbd_rgb_size = (0, 0)
            self._latest_safe_measurement_valid = False
            self._latest_safe_invalid_reason = "Astra RGB不可用"
            if self._rgbd_distance_estimator is not None:
                self._rgbd_distance_estimator.reset()
            self.camera_layout_value.setText("Astra RGB-D")
            self.camera_primary_value.setText("Astra RGB")
            self.camera_raw_size_value.setText("--")
            self.camera_view_size_value.setText("--")
            self.camera_message_value.setText(preview.message)
            if hasattr(self, "binocular_yolo_state"):
                self.binocular_yolo_state.setText(preview.message)
            return

        previous_sample_time = self._latest_rgbd_detection_time
        self._latest_rgbd_detections = dict(preview.yolo_detections or {})
        self._latest_rgbd_detection_time = float(preview.sample_time)
        self._latest_rgbd_rgb_size = (int(preview.width), int(preview.height))
        if (
            self._latest_rgbd_detection_time > previous_sample_time
            and self._rgbd_clearance_estimation_enabled()
        ):
            # A new YOLO observation invalidates the previous sendable sample
            # immediately.  The old number may remain visible as STALE until
            # the next Depth frame is paired and recomputed.
            self._latest_safe_measurement_valid = False
            self._latest_safe_invalid_reason = "等待新YOLO结果匹配Depth"
            if self._latest_safe_depth_time > 0.0:
                self._set_stale_safe_distance_text(self._latest_safe_invalid_reason)
            depth_preview = getattr(self, "_latest_depth_preview", None)
            if (
                depth_preview is not None
                and depth_preview.available
                and depth_preview.depth_mm is not None
                and float(depth_preview.sample_time) > 0.0
                and abs(
                    float(depth_preview.sample_time)
                    - self._latest_rgbd_detection_time
                )
                <= self.rgbd_sync_max_delta
                and self._claim_rgbd_clearance_estimate()
            ):
                # Usually a recent Depth frame is already waiting when YOLO
                # finishes.  Pair it immediately instead of adding another
                # Depth UI period (up to ~67 ms at 15 FPS) to the safety path.
                self._update_safe_distance_from_depth(depth_preview)
        self._update_scene_guard_from_rgb(preview)

        self.camera_layout_value.setText("Astra RGB-D")
        self.camera_primary_value.setText("Astra RGB")
        self.camera_raw_size_value.setText("{}x{} @ {:.1f}fps".format(preview.width, preview.height, preview.fps))
        self.camera_view_size_value.setText("{}x{}".format(preview.width, preview.height))
        self.camera_message_value.setText(preview.message)
        if hasattr(self, "binocular_yolo_state"):
            self.binocular_yolo_state.setText(preview.yolo_message)

    def _update_scene_guard_from_rgb(self, preview: AstraRgbPreview) -> None:
        if (
            not self._workflow_active
            or not self._scene_guard.active
            or self._scene_abort_triggered
        ):
            return
        face_detection = (preview.yolo_detections or {}).get("face")
        face_box = face_detection.get("box") if isinstance(face_detection, dict) else None
        decision = self._scene_guard.update(
            face_box,
            (int(preview.width), int(preview.height)),
            float(preview.sample_time),
        )
        self._handle_scene_guard_decision(
            decision,
            face_present=face_box is not None,
        )

    def _poll_scene_guard(self) -> None:
        if (
            not self._workflow_active
            or not self._scene_guard.active
            or self._scene_abort_triggered
        ):
            return
        decision = self._scene_guard.poll(time.monotonic())
        self._handle_scene_guard_decision(decision)

    def _waiting_for_view_selection(self) -> bool:
        """Return whether the arm is stationary after the three-view capture."""

        return bool(
            self._workflow_active
            and self._distance_safe_latched
            and self._capture_complete
            and not self._capture_sequence_active
            and self._capture_home_phase == ""
            and self._selected_capture_point == 0
            and not self._selected_view_ready
            and not self._target_ctrl_on
            and not self._target_ready
            and not self._output_on
            and not self._waiting_for_arm_ready
            and not self._waiting_for_safe_distance
            and not self._fault_hold
            and not self._scene_abort_triggered
            and not self.arm.control_busy
        )

    def _set_scene_monitor_degraded(self, degraded: bool, reason: str = "") -> None:
        degraded = bool(degraded)
        if bool(getattr(self, "_scene_monitor_degraded", False)) == degraded:
            return
        self._scene_monitor_degraded = degraded
        if degraded:
            detail = "等待选择视图：场景监测暂不可用，已禁止选择（{}）".format(
                reason or "等待稳定人脸画面"
            )
        else:
            detail = "场景监测已恢复且人脸与安全基线一致，可以选择视图"
        self._set_status(detail)
        self._log_event(detail)
        self._refresh_controls()

    def _handle_scene_guard_decision(
        self,
        decision: object,
        *,
        face_present: Optional[bool] = None,
    ) -> None:
        """Keep motion phases strict, but fail closed while stationary/selecting."""

        waiting_selection = self._waiting_for_view_selection()
        reason = str(getattr(decision, "reason", ""))
        abort = bool(getattr(decision, "abort", False))
        if waiting_selection:
            if face_present is False:
                self._set_scene_monitor_degraded(True, reason)
            # poll() can only report a missing/stalled stream.  update() with
            # no face has the same meaning.  While the arm is at final HOME,
            # keep it stationary and disable selection instead of issuing an
            # unnecessary ABORT_HOLD.  A present-but-moved face still aborts.
            if abort and face_present is not True:
                self._set_scene_monitor_degraded(True, reason)
                return
            if not abort and face_present is True and reason == "患者位置稳定":
                self._set_scene_monitor_degraded(False)
        if abort:
            self._abort_for_scene_change(reason)

    def _scene_ready_for_view_selection(
        self,
        now: Optional[float] = None,
    ) -> Tuple[bool, str]:
        """Fail-closed gate used by both the selection UI and serial sender."""

        if not self._waiting_for_view_selection():
            return False, "尚未进入最终 HOME 等待选择阶段"
        if not self._scene_guard.active:
            return False, "场景安全基线未锁存"
        if self._approved_safe_face_snapshot is None:
            return False, "缺少已批准的安全人脸快照"
        if self._scene_monitor_degraded:
            return False, "场景监测暂不可用"

        face_detection = self._latest_rgbd_detections.get("face")
        face_box = face_detection.get("box") if isinstance(face_detection, dict) else None
        try:
            x1, y1, x2, y2 = (float(value) for value in face_box)
        except (TypeError, ValueError):
            return False, "最新 YOLO 结果没有有效人脸框"
        if not all(math.isfinite(value) for value in (x1, y1, x2, y2)) or x2 <= x1 or y2 <= y1:
            return False, "最新 YOLO 人脸框无效"

        timestamp = float(self._latest_rgbd_detection_time)
        current = time.monotonic() if now is None else float(now)
        age = current - timestamp
        fresh_limit = min(
            float(self.serial_safe_distance_max_age),
            float(self._scene_guard.missing_timeout_s),
        )
        if not math.isfinite(age) or age < 0.0 or age >= fresh_limit:
            return False, "场景 YOLO 结果过期（{:.0f} ms）".format(max(0.0, age) * 1000.0)
        return True, "场景监测新鲜且人脸与安全基线一致"

    def _latch_scene_guard(self) -> bool:
        if (
            self._approved_safe_face_snapshot is None
            or not self._scene_guard.active
            or self._scene_abort_triggered
        ):
            return False
        decision = self._scene_guard.poll(time.monotonic())
        if decision.abort:
            self._abort_for_scene_change(decision.reason)
            return False
        return True

    def _clear_safe_scene_binding(self, *, reset_guard: bool = True) -> None:
        self._safe_face_snapshots.clear()
        self._approved_safe_face_snapshot = None
        self._scene_monitor_degraded = False
        if reset_guard:
            self._scene_guard.reset()

    def _provision_safe_scene_snapshot(self, snapshot: SafeFaceSnapshot) -> bool:
        if not self._scene_guard.active:
            return self._scene_guard.latch(
                snapshot.box,
                snapshot.frame_size,
                snapshot.rgb_sample_time,
            )
        decision = self._scene_guard.update(
            snapshot.box,
            snapshot.frame_size,
            snapshot.rgb_sample_time,
        )
        if decision.abort:
            self._abort_for_scene_change(decision.reason)
            return False
        return True

    def _approve_safe_scene_snapshot(self, seq: int) -> bool:
        snapshot = self._safe_face_snapshots.get(int(seq) & 0xFF)
        self._safe_face_snapshots.clear()
        if snapshot is None:
            return False
        if not self._provision_safe_scene_snapshot(snapshot):
            return False
        self._approved_safe_face_snapshot = snapshot
        self._scene_monitor_degraded = False

        # Compare the exact approved sample with the newest RGB observation
        # before accepting the workflow latch.  This closes the interval
        # between SAFE transmission, SAFE_OK and DISTANCE_SAFE_LATCHED.
        face_detection = self._latest_rgbd_detections.get("face")
        face_box = face_detection.get("box") if isinstance(face_detection, dict) else None
        if self._latest_rgbd_detection_time > snapshot.rgb_sample_time:
            decision = self._scene_guard.update(
                face_box,
                self._latest_rgbd_rgb_size,
                self._latest_rgbd_detection_time,
            )
            if decision.abort:
                self._abort_for_scene_change(decision.reason)
                return False
        return not self._scene_abort_triggered

    def _abort_for_scene_change(self, reason: str) -> None:
        if self._scene_abort_triggered or not self._workflow_active:
            return
        self._scene_abort_triggered = True
        detail = "患者/场景变化：{}；必须重新从公共测距开始".format(reason)
        self._log_event(detail)
        self._set_status(detail)
        self._cancel_workflow(reason=detail)

    def _set_camera_preview(self, preview: CameraPreview) -> None:
        layout_text = {
            "side-by-side": "普通左右拼接",
            "astra-rgbd": "Astra RGB-D",
            "mono": "单目 RGB",
        }.get(preview.layout, preview.layout)
        primary_text = {
            "left": "左目",
            "right": "右目",
            "mono": "RGB",
        }.get(preview.primary, preview.primary)
        self._latest_rgb_image = preview.left_image if preview.left_image is not None else None
        self._latest_rgb_image_time = (
            float(preview.sample_time) if preview.left_image is not None else 0.0
        )
        if self.astra_rgb_worker is None or not self.astra_rgb_worker.enabled:
            self.camera_layout_value.setText(layout_text)
            self.camera_primary_value.setText(primary_text)
            self.camera_raw_size_value.setText(f"{preview.raw_width}x{preview.raw_height}")
            self.camera_view_size_value.setText(f"{preview.left_width}x{preview.left_height}")
            self.camera_message_value.setText(preview.message)
            showing_binocular_page = self.pages.currentIndex() == PAGE_BINOCULAR
            if showing_binocular_page and preview.left_image is not None:
                self._set_label_image(
                    self.rgb_preview_label,
                    preview.left_image,
                    "等待 RGB",
                    smooth=False,
                )
            if showing_binocular_page and preview.layout == "side-by-side":
                self._set_label_image(
                    self.depth_preview_label,
                    preview.right_image,
                    "等待右目",
                    smooth=False,
                )

    def _set_depth_preview(self, preview: DepthPreview) -> None:
        self._latest_depth_preview = preview
        # RGB-D now remains a business input for the whole active workflow.
        # The formal SAFE_DISTANCE packet still stops after RA6M5 latches it;
        # post-latch estimates are rate-limited and feed only the local
        # emergency-stop guard, so end-camera tracking is not starved.
        distance_latched = self.__dict__.get("_distance_safe_latched", False)
        continuous_guard_active = (
            distance_latched
            and self.__dict__.get("_workflow_active", False)
            and float(self.__dict__.get("rgbd_continuous_stop_mm", 0.0) or 0.0)
            > 0.0
        )
        clearance_enabled = self._rgbd_clearance_estimation_enabled()
        if clearance_enabled and (not preview.available or preview.depth_mm is None):
            if not distance_latched or continuous_guard_active:
                self._update_safe_distance_from_depth(preview)
        elif clearance_enabled and self._claim_rgbd_clearance_estimate():
            # The expensive point-cloud estimate runs at most once for each
            # claimed YOLO sample, and at the configured bounded rate after
            # the initial distance has been latched.
            self._update_safe_distance_from_depth(preview)
        elif (
            clearance_enabled
            and (not distance_latched or continuous_guard_active)
            and self._latest_rgbd_detection_time > 0.0
            and time.monotonic() - self._latest_rgbd_detection_time
            > self.serial_safe_distance_max_age
            and self._latest_safe_measurement_valid
        ):
            self._invalidate_safe_distance(
                "YOLO结果过期",
                preview,
                preserve_last=True,
            )
        pages = self.__dict__.get("pages")
        if pages is not None and pages.currentIndex() == PAGE_BINOCULAR:
            self._set_label_image(
                self.depth_preview_label,
                preview.image,
                preview.message or "Depth 未启用",
                smooth=False,
            )
        self.depth_state_value.setText(preview.message)
        if not preview.available:
            self.depth_size_value.setText("--")
            self.depth_center_value.setText("--")
            self.depth_valid_value.setText("--")
            self.depth_fps_value.setText("--")
            self.summary_depth_value.setText("--")
            if hasattr(self, "binocular_center_depth_value"):
                self.binocular_center_depth_value.setText("--")
            return

        self.depth_size_value.setText("{}x{}".format(preview.width, preview.height))
        if preview.center_depth_mm > 0:
            depth_text = "{:.0f} mm".format(preview.center_depth_mm)
        else:
            depth_text = "无效"
        self.depth_center_value.setText(depth_text)
        self.depth_valid_value.setText("{:.0f}%".format(preview.valid_ratio * 100.0))
        self.depth_fps_value.setText("{:.1f}".format(preview.fps))
        self.summary_depth_value.setText(depth_text)
        if hasattr(self, "binocular_center_depth_value"):
            self.binocular_center_depth_value.setText(depth_text)

    def _invalidate_safe_distance(
        self,
        reason: str,
        preview: DepthPreview,
        *,
        preserve_last: bool = False,
        reset_filter: bool = False,
        point_ratio: Optional[float] = None,
        result=None,
    ) -> None:
        self._latest_safe_measurement_valid = False
        self._latest_safe_invalid_reason = reason
        if reset_filter and self._rgbd_distance_estimator is not None:
            self._rgbd_distance_estimator.reset()
        if preserve_last and self._latest_safe_depth_time > 0.0:
            self._set_stale_safe_distance_text(reason)
        else:
            self._latest_safe_depth_mm = 0.0
            self._latest_safe_depth_valid_ratio = 0.0
            self._latest_safe_depth_time = 0.0
            self._latest_safe_rgb_time = 0.0
            self._latest_safe_face_box = None
            self._latest_safe_face_size = (0, 0)
            self._set_serial_safe_distance_text(0, False, reason)
        self._log_rgbd_distance_terminal(
            0.0,
            False,
            reason,
            preview,
            point_ratio=point_ratio,
            result=result,
        )

    def _update_safe_distance_from_depth(self, preview: DepthPreview) -> None:
        def invalidate(
            reason: str,
            *,
            preserve_last: bool = False,
            reset_filter: bool = False,
            point_ratio: Optional[float] = None,
            result=None,
        ) -> None:
            self._invalidate_safe_distance(
                reason,
                preview,
                preserve_last=preserve_last,
                reset_filter=reset_filter,
                point_ratio=point_ratio,
                result=result,
            )

        if not preview.available or preview.depth_mm is None:
            invalidate("Depth不可用", reset_filter=True)
            return
        if self._rgbd_distance_estimator is None or np is None:
            invalidate("RGB-D距离模块不可用")
            return
        if preview.intrinsics is None:
            invalidate("Depth内参不可用", reset_filter=True)
            return
        v4l_rgb_calibration = getattr(self, "_astra_rgb_calibration", None)
        has_calibrated_projection = (
            v4l_rgb_calibration is not None
            and preview.rgb_intrinsics is not None
            and preview.depth_to_rgb_rotation is not None
            and preview.depth_to_rgb_translation is not None
            and preview.depth_distortion is not None
        )
        allow_approximate_mapping = (
            self.args.rgbd_allow_approximate_mapping
            and (self.args.demo or self.args.protocol_debug_controls)
        )
        if not self.args.rgbd_hardware_pair_verified and not allow_approximate_mapping:
            invalidate(
                "V4L RGB/SDK Depth设备配对、畸变和时序尚未完成真机验证",
                reset_filter=True,
            )
            return
        if not has_calibrated_projection and not allow_approximate_mapping:
            if v4l_rgb_calibration is None:
                detail = getattr(self, "_astra_rgb_calibration_error", "")
                invalidate(
                    "V4L RGB标定不可用{}".format("：" + detail if detail else ""),
                    reset_filter=True,
                )
            elif preview.depth_distortion is None:
                invalidate(
                    "Depth硬件去畸变状态未知，禁止正式测距",
                    reset_filter=True,
                )
            else:
                invalidate("缺少RGB-Depth外参，禁止正式测距", reset_filter=True)
            return
        if has_calibrated_projection:
            actual_size = self._latest_rgbd_rgb_size
            if (
                preview.depth_calibration_size != (int(preview.width), int(preview.height))
                or preview.rgb_calibration_size is None
                or actual_size != tuple(preview.rgb_calibration_size)
                or actual_size != tuple(v4l_rgb_calibration.image_size)
            ):
                invalidate(
                    "RGB/Depth实际分辨率与SDK外参标定不严格匹配",
                    reset_filter=True,
                )
                return

        now = time.monotonic()
        detection_age = (
            now - self._latest_rgbd_detection_time
            if self._latest_rgbd_detection_time > 0.0
            else float("inf")
        )
        if detection_age > self.serial_safe_distance_max_age:
            invalidate("YOLO结果过期", preserve_last=True)
            return
        depth_sample_time = float(preview.sample_time)
        if depth_sample_time <= 0.0:
            invalidate("Depth采样时间无效")
            return
        if (
            abs(depth_sample_time - self._latest_rgbd_detection_time)
            > self.rgbd_sync_max_delta
        ):
            invalidate("RGB/Depth采样不同步", preserve_last=True)
            return
        if not {"arm", "face"}.issubset(self._latest_rgbd_detections):
            invalidate("未同时识别arm_tip和face", preserve_last=True)
            return

        result = self._rgbd_distance_estimator.estimate(
            preview.depth_mm,
            self._latest_rgbd_rgb_size,
            self._latest_rgbd_detections,
            preview.intrinsics,
            (
                v4l_rgb_calibration.intrinsics
                if has_calibrated_projection
                else preview.rgb_intrinsics
            ),
            preview.depth_to_rgb_rotation,
            preview.depth_to_rgb_translation,
            self.args.astra_rgb_flip,
            preview.depth_distortion,
            (
                v4l_rgb_calibration.distortion
                if has_calibrated_projection
                else preview.rgb_distortion
            ),
            update_filter=False,
        )
        if result is None or not math.isfinite(result.distance_mm):
            invalidate("检测框Depth点不足", preserve_last=True)
            return

        point_ratio = min(
            float(result.arm_valid_ratio),
            float(result.face_valid_ratio),
        )
        if point_ratio < self.serial_safe_distance_min_valid_ratio:
            invalidate(
                "检测框有效Depth不足",
                preserve_last=True,
                point_ratio=point_ratio,
                result=result,
            )
            return
        if (
            time.monotonic() - self._latest_rgbd_detection_time
            > self.serial_safe_distance_max_age
        ):
            # The source observation may have crossed the freshness boundary
            # while the 3-D estimator was running.  Never flash it as valid or
            # hand it to the wire merely because it was fresh at entry.
            invalidate(
                "YOLO结果在测距计算期间过期",
                preserve_last=True,
                point_ratio=point_ratio,
                result=result,
            )
            return
        # Only a frame that passed every geometry, quality, synchronization,
        # and freshness gate may influence later filtered measurements.
        result = self._rgbd_distance_estimator.commit_result(result)
        self._latest_safe_depth_mm = max(0.0, float(result.distance_mm))
        self._latest_safe_depth_valid_ratio = point_ratio
        self._latest_safe_depth_time = depth_sample_time
        face_detection = self._latest_rgbd_detections.get("face")
        self._latest_safe_face_box = (
            tuple(float(value) for value in face_detection.get("box", ()))
            if isinstance(face_detection, dict)
            else None
        )
        if self._latest_safe_face_box is not None and len(self._latest_safe_face_box) != 4:
            self._latest_safe_face_box = None
        self._latest_safe_face_size = tuple(self._latest_rgbd_rgb_size)
        self._latest_safe_rgb_time = self._latest_rgbd_detection_time
        self._latest_safe_measurement_valid = True
        self._latest_safe_invalid_reason = ""
        self._set_serial_safe_distance_text(
            int(round(self._latest_safe_depth_mm)),
            True,
            "arm-face三维净距",
        )
        self._log_rgbd_distance_terminal(
            self._latest_safe_depth_mm,
            True,
            "arm-face三维净距",
            preview,
            point_ratio=point_ratio,
            result=result,
        )
        self._enforce_continuous_rgbd_guard()
        # Do not wait for the next periodic Qt service tick when a genuinely
        # new sample is already available.  Sending here removes up to one
        # business interval of avoidable capture-to-wire latency; the sender's
        # phase/window checks still prevent out-of-state traffic.
        if (
            self._measure_position_ready
            and self._waiting_for_safe_distance
        ):
            send_now = time.monotonic()
            if (
                send_now - self._last_serial_safe_distance
                >= self.serial_safe_distance_interval
            ):
                self._send_serial_safe_distance_now(send_now)
        # This phase-gated transmission cannot bypass RA6M5: protocol v5 only
        # permits SAFE_DISTANCE after MEASURE_POSITION_READY, and RA6M5 still
        # owns the threshold, latch, retreat, and all motion decisions.

    def _enforce_continuous_rgbd_guard(self) -> bool:
        """Abort an active post-latch workflow on verified sub-threshold RGB-D.

        This is deliberately separate from PARTIAL_FACE.  PARTIAL_FACE protects
        target identity/geometry in the end camera; this guard uses the Astra
        arm-tip-to-face 3-D surface clearance and the existing ABORT_HOLD
        command.  It never emits a second SAFE_DISTANCE phase or action 0x04.
        """

        threshold_mm = max(
            0.0,
            float(self.__dict__.get("rgbd_continuous_stop_mm", 0.0) or 0.0),
        )
        if (
            threshold_mm <= 0.0
            or not self.__dict__.get("_workflow_active", False)
            or not self.__dict__.get("_distance_safe_latched", False)
            or self.__dict__.get("_rgbd_continuous_stop_triggered", False)
            or not self.__dict__.get("_latest_safe_measurement_valid", False)
        ):
            return False

        distance_mm = float(
            self.__dict__.get("_latest_safe_depth_mm", float("inf"))
        )
        if not math.isfinite(distance_mm) or distance_mm >= threshold_mm:
            return False

        self._rgbd_continuous_stop_triggered = True
        reason = (
            "RGB-D连续安全门触发：arm-face三维净距 "
            "{:.1f} mm < {:.1f} mm，已请求 RA6M5 ABORT_HOLD"
        ).format(distance_mm, threshold_mm)
        print("[RGBD_GUARD] " + reason, flush=True)
        log_event = getattr(self, "_log_event", None)
        if callable(log_event):
            log_event(reason)
        self._cancel_workflow(reason)
        return True

    def _log_rgbd_distance_terminal(
        self,
        distance_mm: float,
        valid: bool,
        reason: str,
        preview: DepthPreview,
        *,
        point_ratio: Optional[float] = None,
        result=None,
    ) -> None:
        """Print throttled RGB-D diagnostics without changing safety validity."""

        if not getattr(self, "rgbd_distance_log_enabled", False):
            return

        now = time.monotonic()
        state = (bool(valid), str(reason))
        last_state = getattr(self, "_last_rgbd_distance_log_state", None)
        last_time = float(getattr(self, "_last_rgbd_distance_log_time", 0.0))
        interval = max(0.10, float(getattr(self, "rgbd_distance_log_interval", 0.50)))
        if state == last_state and now - last_time < interval:
            return

        depth_sample_time = float(getattr(preview, "sample_time", 0.0) or 0.0)
        rgb_sample_time = float(getattr(self, "_latest_rgbd_detection_time", 0.0) or 0.0)
        sync_ms = (
            abs(depth_sample_time - rgb_sample_time) * 1000.0
            if depth_sample_time > 0.0 and rgb_sample_time > 0.0
            else float("inf")
        )
        rgb_age_ms = (
            max(0.0, now - rgb_sample_time) * 1000.0
            if rgb_sample_time > 0.0
            else float("inf")
        )
        detections = ",".join(sorted(getattr(self, "_latest_rgbd_detections", {}).keys())) or "none"
        if (
            getattr(self, "_astra_rgb_calibration", None) is not None
            and getattr(preview, "depth_to_rgb_rotation", None) is not None
            and getattr(preview, "depth_to_rgb_translation", None) is not None
            and getattr(preview, "depth_distortion", None) is not None
        ):
            projection = "v4l-brown+d2c/{}".format(
                getattr(preview, "depth_distortion_mode", "unknown")
            )
        elif (
            getattr(preview, "rgb_intrinsics", None) is not None
            and getattr(preview, "depth_to_rgb_rotation", None) is not None
            and getattr(preview, "depth_to_rgb_translation", None) is not None
        ):
            projection = "sdk-geometry-unknown"
        else:
            projection = "approx"
        common = (
            "sync_ms={:.1f} rgb_age_ms={:.1f} depth_valid={:.0f}% "
            "depth_fps={:.1f} detections={} projection={}"
        ).format(
            sync_ms,
            rgb_age_ms,
            float(getattr(preview, "valid_ratio", 0.0)) * 100.0,
            float(getattr(preview, "fps", 0.0)),
            detections,
            projection,
        )
        stamp = time.strftime("%H:%M:%S")
        if valid:
            raw_mm = float(getattr(result, "raw_distance_mm", distance_mm))
            filtered_mm = float(getattr(result, "filtered_distance_mm", distance_mm))
            sampled_minimum_mm = float(
                getattr(result, "sampled_minimum_mm", raw_mm)
            )
            arm_cover_mm = float(getattr(result, "arm_cover_radius_mm", 0.0))
            face_cover_mm = float(getattr(result, "face_cover_radius_mm", 0.0))
            ratio_text = "--" if point_ratio is None else "{:.0f}%".format(point_ratio * 100.0)
            arm_points = int(getattr(result, "arm_points", 0))
            face_points = int(getattr(result, "face_points", 0))
            print(
                "[RGBD_DISTANCE {}] VALID reported_mm={:.1f} raw_mm={:.1f} "
                "filtered_mm={:.1f} sampled_min_mm={:.1f} cover_mm={:.1f}+{:.1f} "
                "point_ratio={} arm_points={} face_points={} {}".format(
                    stamp,
                    float(distance_mm),
                    raw_mm,
                    filtered_mm,
                    sampled_minimum_mm,
                    arm_cover_mm,
                    face_cover_mm,
                    ratio_text,
                    arm_points,
                    face_points,
                    common,
                ),
                flush=True,
            )
        else:
            ratio_text = "--" if point_ratio is None else "{:.0f}%".format(point_ratio * 100.0)
            print(
                "[RGBD_DISTANCE {}] INVALID reason={} point_ratio={} {}".format(
                    stamp,
                    reason,
                    ratio_text,
                    common,
                ),
                flush=True,
            )
        self._last_rgbd_distance_log_state = state
        self._last_rgbd_distance_log_time = now

    def _set_result(self, result: DetectionResult) -> None:
        self.current_result = result
        self._latest_result_time = (
            float(result.sample_time) if result.sample_time > 0.0 else time.monotonic()
        )
        self._update_target_identity_lock(result)
        self._set_safety_state(result)
        self._publish_latest_vision_result()
        self.mode_value.setText(result.mode)
        self.fps_value.setText(f"{result.fps:.1f}")
        identity_lock = self.__dict__.get("_target_identity_lock")
        identity_active = bool(
            identity_lock is not None and identity_lock.active
        )
        if identity_active:
            display_targets = (
                [self._locked_current_target]
                if self._locked_current_target is not None
                else []
            )
        else:
            display_targets = list(result.targets)
        display_target_count = len(display_targets)
        self.target_count.setText(str(display_target_count))
        self.summary_fps_value.setText(f"{result.fps:.1f}")
        self.summary_target_value.setText(str(display_target_count))

        if not display_targets:
            if identity_active:
                self.detect_state.setText(
                    getattr(self, "_last_target_lock_reason", "")
                    or "锁定目标暂时丢失，禁止切换其他候选"
                )
            elif result.message:
                self.detect_state.setText(result.message)
            else:
                self.detect_state.setText("未发现目标" if result.mode != MODE_PREVIEW else "预览中")
            self.pixel_coord.setText("--")
            self.robot_coord.setText("--")
            self.radius_value.setText("--")
            self.score_value.setText("--")
            self._refresh_controls()
            return

        target_locked = (
            self._locked_current_target is not None
            and self._locked_target_sample_time == self._latest_result_time
        )
        primary = display_targets[0]
        self.detect_state.setText(
            "目标已稳定锁定" if target_locked else "发现候选，等待稳定锁定"
        )
        self.pixel_coord.setText(f"X {primary.x} / Y {primary.y}")
        try:
            dcx, dcy = self.arm.aim_corrected_error(
                ArmTarget(
                    pixel=(primary.x, primary.y),
                    frame_size=(result.frame_width, result.frame_height),
                )
            )
        except ValueError:
            dcx = dcy = None

        if dcx is None or dcy is None:
            self.robot_coord.setText(
                "激光瞄准参考像素未标定（valid=0，禁止启动定靶）"
            )
        elif target_locked and self._target_ready:
            self.robot_coord.setText(
                f"上报激光参考误差 dcx {dcx} / dcy {dcy} / valid=1"
            )
        elif target_locked:
            self.robot_coord.setText(
                f"锁定激光参考误差 dcx {dcx} / dcy {dcy}（等待 TARGET_READY）"
            )
        else:
            self.robot_coord.setText(
                f"候选激光参考误差 dcx {dcx} / dcy {dcy}（valid=0，不上报坐标）"
            )
        self.radius_value.setText(f"{primary.radius} px")
        self.score_value.setText(f"{primary.contrast:.1f}")
        self._add_history(primary)
        self._refresh_controls()

    def _update_target_identity_lock(self, result: DetectionResult) -> None:
        if (
            not self._workflow_active
            or not self._selected_view_ready
            or self._latest_result_time <= self._selected_view_ready_time
        ):
            self._locked_current_target = None
            self._locked_target_sample_time = 0.0
            self._last_target_lock_reason = "等待选中视图到位后的新识别结果"
            return
        was_active = self._target_identity_lock.active
        targets_for_lock = list(result.targets)
        original_indices = list(range(len(targets_for_lock)))
        seed_target = self.__dict__.get("_selected_triptych_target")
        if (
            not was_active
            and seed_target is not None
            and len(targets_for_lock) > 1
        ):
            seed_x = float(getattr(seed_target, "x", -1.0))
            seed_y = float(getattr(seed_target, "y", -1.0))
            seed_radius = max(1.0, float(getattr(seed_target, "radius", 8.0)))
            ranked = sorted(
                enumerate(targets_for_lock),
                key=lambda item: math.hypot(
                    float(getattr(item[1], "x", -1.0)) - seed_x,
                    float(getattr(item[1], "y", -1.0)) - seed_y,
                ),
            )
            best_distance = (
                math.hypot(
                    float(getattr(ranked[0][1], "x", -1.0)) - seed_x,
                    float(getattr(ranked[0][1], "y", -1.0)) - seed_y,
                )
                if ranked
                else float("inf")
            )
            seed_gate = max(36.0, seed_radius * 8.0)
            if best_distance <= seed_gate:
                original_indices = [index for index, _target in ranked]
                targets_for_lock = [target for _index, target in ranked]
                self._last_target_lock_reason = (
                    "使用三视图候选作为实时初始锚点，最近候选距离 {:.1f}px"
                ).format(best_distance)
            else:
                self._last_target_lock_reason = (
                    "三视图候选与实时候选距离 {:.1f}px 超过 {:.1f}px，等待实时稳定"
                ).format(best_distance, seed_gate)
        decision = self._target_identity_lock.update(
            [(target.x, target.y, target.radius) for target in targets_for_lock],
            (result.frame_width, result.frame_height),
            self._latest_result_time,
            candidate_ids=[
                int(getattr(target, "track_id", 0)) for target in targets_for_lock
            ],
        )
        if decision.reason:
            self._last_target_lock_reason = decision.reason
        if decision.index is None or decision.index >= len(targets_for_lock):
            self._locked_current_target = None
            self._locked_target_sample_time = 0.0
            return
        original_index = original_indices[decision.index]
        self._locked_current_target = result.targets[original_index]
        self._locked_target_sample_time = self._latest_result_time
        worker = self.__dict__.get("worker")
        if worker is not None and hasattr(worker, "set_display_track_id"):
            worker.set_display_track_id(
                int(getattr(self._locked_current_target, "track_id", 0))
            )
        if not was_active:
            self._log_event(
                "定靶会话已锁定 track_id={} 目标 X{} Y{}；"
                "后续只接受同一跟踪ID，并保留尺度、歧义和已排除候选保护".format(
                    int(getattr(self._locked_current_target, "track_id", 0)),
                    self._locked_current_target.x,
                    self._locked_current_target.y,
                )
            )

    def _reset_target_identity_lock(self) -> None:
        self._target_identity_lock.reset()
        self._locked_current_target = None
        self._locked_target_sample_time = 0.0
        worker = self.__dict__.get("worker")
        if worker is not None and hasattr(worker, "set_display_track_id"):
            worker.set_display_track_id(0)
        self._last_target_lock_reason = "等待稳定候选建立身份锁"
        self._vision_last_published_valid = False
        self._vision_invalid_transition_sent = False
        self._vision_transient_loss_started_at = 0.0

    def _set_safety_state(self, result: DetectionResult) -> None:
        if result.safety_message == "安全检测未启用":
            self._safety_blocking = False
            self.safety_state.setText("安全距离关闭")
            self.summary_safety_value.setText("未启用")
            self.safety_state.setStyleSheet(
                "color: #8fa2b7; font-size: 12px; font-weight: 700;"
            )
            return

        detail = "脸部占比 {:.0f}% / 边界 {:.0f}%".format(
            result.face_area_ratio * 100.0,
            result.face_bbox_ratio * 100.0,
        )
        self._safety_blocking = result.safety_level == SAFETY_DANGER
        if result.safety_level == SAFETY_DANGER:
            self.summary_safety_value.setText("危险")
            self.safety_state.setText("{}；{}".format(result.safety_message, detail))
            self.safety_state.setStyleSheet(
                "color: #ff6b73; font-size: 12px; font-weight: 800;"
            )
        elif result.safety_level == SAFETY_WARN:
            self.summary_safety_value.setText("警告")
            self.safety_state.setText("{}；{}".format(result.safety_message, detail))
            self.safety_state.setStyleSheet(
                "color: #f8c14a; font-size: 12px; font-weight: 800;"
            )
        else:
            self.summary_safety_value.setText("正常")
            self.safety_state.setText("{}；{}".format(result.safety_message, detail))
            self.safety_state.setStyleSheet(
                "color: #51d494; font-size: 12px; font-weight: 700;"
            )

    def _set_reference_state(self, ok: bool, message: str) -> None:
        self._reference_capturing = False
        self.reference_state.setText("已启用" if ok else "未启用")
        self._set_status(message)
        self._log_event(message)
        self._refresh_controls()

    def _set_safety_capture_state(self, ok: bool, message: str) -> None:
        self._safety_capturing = False
        self.safety_state.setText("已启用" if ok else "未启用")
        self._set_status(message)
        self._log_event(message)
        self._refresh_controls()

    def _add_history(self, target: DetectionTarget) -> None:
        stamp = QDateTime.currentDateTime().toString("HH:mm:ss")
        item = f"{stamp}  X {target.x} / Y {target.y}  对比度 {target.contrast:.1f}"
        self._add_history_text(item)

    def _add_history_text(self, item: str) -> None:
        if self.history and self.history[0] == item:
            return
        self.history.appendleft(item)
        self.history_list.clear()
        self.history_list.addItems(list(self.history))

    def _log_event(self, text: str) -> None:
        stamp = QDateTime.currentDateTime().toString("HH:mm:ss")
        self._add_history_text(f"{stamp}  {text}")

    def _set_status(self, text: str) -> None:
        self.status_label.setToolTip(text)
        self.status_label.setText(self._compact_status_text(text))

    def _compact_status_text(self, text: str, limit: int = 32) -> str:
        if len(text) <= limit:
            return text
        return "{}...".format(text[: limit - 3])

    def _clear_current_target(self, state_text: str) -> None:
        self.current_result = None
        self._locked_current_target = None
        self._locked_target_sample_time = 0.0
        self.detect_state.setText(state_text)
        self.target_count.setText("0")
        self.summary_target_value.setText("0")
        self.pixel_coord.setText("--")
        self.robot_coord.setText("--")
        self.radius_value.setText("--")
        self.score_value.setText("--")

    def _capture_reference(self) -> None:
        if self.args.detector_mode == "facial_candidate":
            QMessageBox.information(
                self,
                "无需基准图",
                "正式人脸候选检测模式不需要采集无黑痣基准图。",
            )
            return
        if (
            self._waiting_for_arm_ready
            or self._waiting_for_safe_distance
            or self._detection_running
            or self._reference_capturing
            or self._safety_capturing
            or self._capture_sequence_active
        ):
            return

        self._switch_page(PAGE_END_CAMERA)
        if self.worker is not None:
            self.worker.stop_detection()
        self._clear_current_target("准备采集基准图")
        if self.arm.connected:
            ok, message = self.arm.start_target_state_machine()
            self._log_event(message)
            if not ok:
                self.workflow_state.setText("基准采集启动失败")
                self.ready_wait_value.setText("--")
                self._set_status(f"基准采集启动失败：{message}")
                self._refresh_controls()
                QMessageBox.warning(self, "基准采集启动失败", message)
                return

            self._waiting_for_arm_ready = True
            self._detection_running = False
            self._ready_after_action = "reference"
            self._arm_ready_wait_started = time.monotonic()
            self.workflow_state.setText("等待机械臂 READY 后采集基准图")
            self.ready_wait_value.setText("0.0 s")
            self._set_status(f"已请求机械臂伸展，等待 READY 后采集基准图：{message}")
            self._refresh_controls()
            return

        message = "RA6M5 未连接，直接采集基准图；{}".format(
            self.arm.connection_text()
        )
        self._set_status(message)
        self._log_event("未发送机械臂伸展指令：" + self.arm.connection_text())
        self._start_reference_capture_after_ready()

    def _capture_safety_distance(self) -> None:
        if (
            self._waiting_for_arm_ready
            or self._waiting_for_safe_distance
            or self._detection_running
            or self._reference_capturing
            or self._safety_capturing
            or self._capture_sequence_active
        ):
            return

        self._switch_page(PAGE_END_CAMERA)
        if self.worker is not None:
            self.worker.stop_detection()
        self._clear_current_target("准备采集安全距离图")
        self._safety_capturing = True
        if self.worker is not None:
            self.worker.request_safety_capture()
        self.workflow_state.setText("采集安全距离图")
        self.ready_wait_value.setText("--")
        self.safety_state.setText("采集中")
        self._set_status("开始采集安全距离图，请保持当前距离稳定")
        self._log_event("开始采集安全距离图")
        self._refresh_controls()

    def _start_workflow(self) -> None:
        if getattr(self, "_unsupported_continuous_safe_distance_requested", False):
            self.workflow_state.setText("启动参数不受支持")
            self.ready_wait_value.setText("请删除旧参数")
            self._set_status(UNSUPPORTED_CONTINUOUS_SAFE_DISTANCE_MESSAGE)
            QMessageBox.critical(
                self,
                "连续测距模式未启用",
                UNSUPPORTED_CONTINUOUS_SAFE_DISTANCE_MESSAGE,
            )
            return
        if self._restart_blocked:
            QMessageBox.warning(self, "通信状态未知", "安全停止未获确认且串口已关闭；请排故并重新启动应用后再开始。")
            return
        if (
            self._waiting_for_arm_ready
            or self._waiting_for_safe_distance
            or self._detection_running
            or self._reference_capturing
            or self._safety_capturing
            or self._capture_sequence_active
        ):
            return
        if self.arm.serial_protocol != "unified" and not self.args.protocol_debug_controls:
            QMessageBox.warning(self, "协议不允许", "比赛流程只允许 A5 5A 统一协议。")
            return

        self._reset_protocol_session(clear_captures=True)
        self._switch_page(PAGE_BINOCULAR)
        self._clear_current_target("等待公共测距")
        if self.arm.connected:
            self._send_workflow_start_command()
            return

        if not self.args.demo:
            self.workflow_state.setText("等待机械臂连接")
            self._set_status("RA6M5 未连接，正式流程不能继续")
            QMessageBox.warning(self, "机械臂未连接", self.arm.connection_text())
            self._refresh_controls()
            return
        self._workflow_active = True
        self._measure_position_ready = True
        self._distance_safe_latched = True
        self._preposition_ready = True
        self.workflow_state.setText("演示：安全距离已锁存")
        self.ready_wait_value.setText("可采图")
        self._set_status("演示模式：已模拟公共测距位和安全距离锁存")
        self._refresh_controls()

    def _send_workflow_start_command(self) -> None:
        if not self.arm.connected:
            return
        if self.arm.control_busy:
            return
        # Defensive invariant: even a caller that bypasses main() cannot make
        # the Qt formal Start button emit the unapproved action=0x04.
        self.arm.continuous_safe_distance = False
        ok, message = self.arm.start_workflow()
        self._log_event(message)
        if not ok:
            self.workflow_state.setText("流程启动失败")
            self.ready_wait_value.setText("--")
            self._set_status(f"流程启动失败：{message}")
            self._refresh_controls()
            QMessageBox.warning(self, "流程启动失败", message)
            return

        self._workflow_active = True
        self._waiting_for_arm_ready = True
        self._waiting_for_safe_distance = False
        self._detection_running = False
        self._ready_after_action = "measure_position"
        self._arm_ready_wait_started = time.monotonic()
        self.workflow_state.setText("前往公共测距位")
        self.ready_wait_value.setText("0.0 s")
        label = self.__dict__.get("binocular_safe_distance_value")
        if label is not None:
            label.setText("机械臂伸展中，等待公共测距位")
        self._set_status(f"{message}；等待 MEASURE_POSITION_READY")
        self._refresh_controls()

    def _reset_protocol_session(self, clear_captures: bool) -> None:
        self._protocol_session_generation += 1
        if self.triptych_worker is not None and self.triptych_worker.isRunning():
            self.triptych_worker.requestInterruption()
        if self.worker is not None:
            self.worker.stop_detection()
        self._waiting_for_arm_ready = False
        self._waiting_for_safe_distance = False
        self._detection_running = False
        self._preposition_ready = False
        self._workflow_active = False
        self._measure_position_ready = False
        self._measure_position_ready_time = 0.0
        self._distance_safe_latched = False
        self._capture_home_phase = ""
        self._capture_complete = False
        self._selected_view_ready = False
        self._selected_view_ready_time = 0.0
        self._target_ctrl_on = False
        self._target_ready = False
        self._align_done = False
        self._output_on = False
        self._vision_valid = False
        self._recoverable_vision_loss_active = False
        self._fault_hold = False
        self._restart_blocked = False
        self._capture_sequence_active = False
        self._capture_sequence_manual = False
        self._capture_sequence_points = []
        self._capture_sequence_index = 0
        self._capture_pending_point = 0
        self._capture_ready_frame_time = 0.0
        self._selected_capture_point = 0
        self._selected_triptych_target = None
        self._reset_target_identity_lock()
        self._ready_after_action = ""
        self._arm_ready_wait_started = None
        self._last_serial_vision_error = 0.0
        self._last_sent_vision_sample_time = 0.0
        self._last_sent_safe_sample_time = 0.0
        self._latest_safe_depth_mm = 0.0
        self._latest_safe_depth_valid_ratio = 0.0
        self._latest_safe_depth_time = 0.0
        self._latest_safe_rgb_time = 0.0
        self._latest_safe_face_box = None
        self._latest_safe_face_size = (0, 0)
        self._latest_safe_measurement_valid = False
        self._latest_safe_invalid_reason = "等待开始并伸展到公共测距位"
        self._last_rgbd_estimated_detection_time = 0.0
        self._last_rgbd_continuous_estimate_time = 0.0
        self._rgbd_continuous_stop_triggered = False
        self._clear_safe_scene_binding()
        self._scene_abort_triggered = False
        self._scene_monitor_degraded = False
        if self._rgbd_distance_estimator is not None:
            self._rgbd_distance_estimator.reset()
        label = self.__dict__.get("binocular_safe_distance_value")
        if label is not None:
            label.setText("等待开始并伸展到公共测距位")
        if clear_captures:
            for point_id in (1, 2, 3):
                self._capture_images[point_id] = None
            self._triptych_detection_results = {}
            self.capture_progress_value.setText("三视图未采集")
            self.selected_view_value.setText("未选择")
            self._update_capture_selection_ui()

    def _start_detection(self) -> None:
        if (
            self._waiting_for_arm_ready
            or self._waiting_for_safe_distance
            or self._detection_running
            or self._reference_capturing
            or self._safety_capturing
            or self._capture_sequence_active
        ):
            return
        if self._safety_blocking:
            QMessageBox.warning(self, "安全距离危险", "人脸距离过近，已禁止开始识别。请先后退或停止靠近。")
            return

        self._clear_current_target("等待识别")
        if self.arm.connected:
            ok, message = self.arm.start_target_state_machine()
            self._log_event(message)
            if not ok:
                self.workflow_state.setText("启动失败")
                self.ready_wait_value.setText("--")
                self._set_status(f"启动失败；{message}")
                self._refresh_controls()
                QMessageBox.warning(self, "状态机启动未发送", message)
                return

            self._switch_page(PAGE_END_CAMERA)
            if self.worker is not None:
                self.worker.stop_detection()
            self._waiting_for_arm_ready = True
            self._detection_running = False
            self._ready_after_action = "detect"
            self._arm_ready_wait_started = time.monotonic()
            self.workflow_state.setText("等待机械臂 READY")
            self.ready_wait_value.setText("0.0 s")
            self._set_status(f"已发送机械臂伸展指令，等待 READY；{message}")
            self._refresh_controls()
        else:
            self._switch_page(PAGE_END_CAMERA)
            if self.worker is not None:
                self.worker.start_detection()
            self._waiting_for_arm_ready = False
            self._detection_running = True
            self._ready_after_action = ""
            self._arm_ready_wait_started = None
            self.workflow_state.setText("连续识别")
            self.ready_wait_value.setText("--")
            self._set_status("未连接 RA6M5，直接连续识别")
            self._refresh_controls()

    def _detect_once(self) -> None:
        if self._waiting_for_safe_distance:
            QMessageBox.information(self, "正在等待双目安全距离", "双目 Depth 有效后再执行单次识别。")
            return
        if self._waiting_for_arm_ready:
            QMessageBox.information(self, "正在等待 READY", "机械臂未回传 READY，暂不执行单次识别。")
            return
        if self._reference_capturing:
            QMessageBox.information(self, "正在采集基准图", "基准图采集完成后再执行识别。")
            return
        if self._safety_capturing:
            QMessageBox.information(self, "正在采集安全距离图", "安全距离图采集完成后再执行识别。")
            return
        if self._capture_sequence_active:
            QMessageBox.information(self, "正在采集三视图", "左中右三视图采集完成后再执行识别。")
            return
        self._switch_page(PAGE_END_CAMERA)
        if self.worker is not None:
            self.worker.request_detect_once()
        self.workflow_state.setText("单次识别")
        self._set_status("已请求单次识别")

    def _stop_detection(self) -> None:
        self._cancel_workflow()

    def _finish_workflow(self) -> None:
        if self.arm.control_busy:
            QMessageBox.information(self, "动作执行中", "当前控制动作尚未完成；如需立即停止，请使用取消/安全停止。")
            return
        if self._restart_blocked:
            QMessageBox.warning(self, "通信状态未知", "串口已关闭或控制链路状态未知；请确认机械臂安全后重启 Qt/RA6M5。")
            return
        if not self.arm.connected:
            if self.args.demo:
                self._reset_protocol_session(clear_captures=False)
                self.workflow_state.setText("演示流程已完成")
                self.ready_wait_value.setText("--")
                self._refresh_controls()
                return
            QMessageBox.warning(self, "机械臂未连接", self.arm.connection_text())
            return
        reason = (
            "手动复位：请求返回 HOME"
            if not self._workflow_active
            else "手动完成/复位：请求返回 HOME"
        )
        ok, message = self._request_finish_return_home(reason)
        if not ok:
            QMessageBox.warning(self, "复位/返回未发送", message)

    def _request_finish_return_home(self, reason: str = "") -> Tuple[bool, str]:
        """Send FINISH_RETURN_HOME and prepare Qt for the terminal HOME wait."""

        if self.worker is not None:
            self.worker.stop_detection()
        self._target_ready = False
        self._target_ctrl_on = False
        self._align_done = False
        self._output_on = False
        self._recoverable_vision_loss_active = False
        self._detection_running = False
        ok, message = self.arm.finish_workflow()
        self._log_event(message)
        if not ok:
            return False, message
        self._waiting_for_arm_ready = True
        self._ready_after_action = "finish_return_home"
        self._arm_ready_wait_started = time.monotonic()
        self.workflow_state.setText("复位并返回 HOME")
        self.ready_wait_value.setText("等待 RETURN_HOME_DONE")
        self._set_status(reason or message)
        self._refresh_controls()
        return True, message

    def _cancel_workflow(self, reason: str = "") -> None:
        if not isinstance(reason, str):
            reason = ""
        if self._ready_after_action == "abort_hold" or self.arm.abort_pending:
            self.workflow_state.setText("安全停止中")
            self.ready_wait_value.setText("等待 ABORTED_HOLD")
            self._set_status(reason or "ABORT_HOLD 已发送，正在等待 RA6M5 确认")
            self._refresh_controls()
            return
        if self.worker is not None:
            self.worker.stop_detection()
            self.worker.cancel_reference_capture()
            self.worker.cancel_safety_capture()
        self._target_ready = False
        self._target_ctrl_on = False
        self._align_done = False
        self._output_on = False
        self._vision_valid = False
        self._recoverable_vision_loss_active = False
        self._detection_running = False
        self._waiting_for_safe_distance = False
        self._measure_position_ready = False
        self._measure_position_ready_time = 0.0
        self._distance_safe_latched = False
        self._preposition_ready = False
        self._selected_view_ready = False
        self._selected_view_ready_time = 0.0
        self._capture_sequence_active = False
        self._capture_pending_point = 0
        self._last_sent_safe_sample_time = 0.0
        self._last_sent_vision_sample_time = 0.0
        self._reset_target_identity_lock()
        self._clear_safe_scene_binding()
        if not self.arm.connected:
            if self._restart_blocked:
                self._workflow_active = False
                self._fault_hold = True
                self.workflow_state.setText("通信状态未知，禁止重新开始")
                self._set_status(reason or "安全停止未获确认；请排故并重新启动应用")
                self._refresh_controls()
                return
            self._reset_protocol_session(clear_captures=False)
            self.workflow_state.setText("已取消")
            self._set_status(reason or "本地流程已取消；RA6M5 未连接")
            self._refresh_controls()
            return
        ok, message = self.arm.abort_workflow()
        self._log_event(message)
        if not ok:
            self._fault_hold = True
            self._restart_blocked = True
            self._workflow_active = False
            self.arm.close()
            self.serial_state.setText("串口已关闭，等待 RA6M5 心跳超时保护")
            self._set_status(reason or "安全停止发送失败：{}".format(message))
            QMessageBox.warning(self, "安全停止未发送", message)
            self._refresh_controls()
            return
        self._waiting_for_arm_ready = True
        self._ready_after_action = "abort_hold"
        self._arm_ready_wait_started = time.monotonic()
        self.workflow_state.setText("安全停止中")
        self.ready_wait_value.setText("等待 ABORTED_HOLD")
        self._set_status("{}；{}".format(reason, message) if reason else message)
        self._activate_page_resources(self.pages.currentIndex())
        self._refresh_controls()

    def _legacy_stop_detection(self) -> None:
        if self.worker is not None:
            self.worker.stop_detection()
            self.worker.cancel_reference_capture()
            self.worker.cancel_safety_capture()
        self._waiting_for_arm_ready = False
        self._waiting_for_safe_distance = False
        self._detection_running = False
        self._reference_capturing = False
        self._safety_capturing = False
        self._capture_sequence_active = False
        self._capture_sequence_manual = False
        self._preposition_ready = False
        self._capture_sequence_points = []
        self._capture_sequence_index = 0
        self._capture_pending_point = 0
        self._ready_after_action = ""
        self._arm_ready_wait_started = None
        self._last_sent_vision_sample_time = 0.0
        self.workflow_state.setText("已停止")
        self.ready_wait_value.setText("--")
        if self.arm.connected:
            ok, message = self.arm.stop_target_state_machine()
            self._set_status(f"已停止识别；{message}")
            self._log_event(message)
            if not ok:
                QMessageBox.warning(self, "状态机关闭未发送", message)
        else:
            self._set_status("已停止识别，保留预览")
        self._refresh_controls()

    def _preview_move(self) -> None:
        if self._safety_blocking:
            QMessageBox.warning(self, "安全距离危险", "人脸距离过近，已禁止预定位。请先后退或停止靠近。")
            return
        arm_target = self._current_arm_target()
        if arm_target is None:
            QMessageBox.warning(self, "无法预定位", "当前没有有效识别目标。")
            return
        ok, message = self.arm.preview_move(arm_target)
        if ok:
            QMessageBox.information(self, "预定位", message)
        else:
            QMessageBox.warning(self, "预定位未发送", message)

    def _fire_laser(self) -> None:
        if not self._workflow_active or self._fault_hold:
            QMessageBox.warning(self, "流程状态无效", "当前不在已确认安全的比赛流程中。")
            return
        if not self._distance_safe_latched:
            QMessageBox.warning(self, "安全门未满足", "本轮安全距离尚未锁存，不能开始点痣。")
            return
        if not self._selected_view_ready:
            QMessageBox.warning(self, "视图未到位", "必须先选择左/正/右视图，并等待 SELECTED_VIEW_READY。")
            return
        if self.arm.connected and not self.arm.aim_reference_configured:
            message = (
                "尚未配置在实际 15–20 mm 工作距离标定的激光瞄准参考像素。\n"
                "必须显式配置 --laser-offset-x-px 和 --laser-offset-y-px；"
                "禁止使用 27.81 mm/mm-per-pixel 换算值代替真实标定。\n"
                "本次不会发送 TARGET_CTRL START。"
            )
            self._set_status("激光瞄准参考像素未标定，已拦截定靶启动")
            QMessageBox.warning(self, "激光瞄准参考未标定", message)
            return
        if (
            self._target_ctrl_on
            or self._target_ready
            or self._waiting_for_arm_ready
            or self.arm.control_busy
        ):
            return
        confirmed_generation = self._protocol_session_generation
        reply = QMessageBox.question(
            self,
            "确认开始点痣",
            "确认请求 RA6M5 启动定靶？收到 TARGET_READY 后才发送视觉误差；"
            "首次尚未稳定锁定时发送 valid=0；已锁定目标的单帧抖动会短时静默复锁。"
            "Jetson 不会直接打开 P801；"
            "对准后仍需按住 P000。",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if reply != QMessageBox.Yes:
            return
        if (
            confirmed_generation != self._protocol_session_generation
            or not self._workflow_active
            or self._fault_hold
            or self._restart_blocked
            or not self._distance_safe_latched
            or not self._selected_view_ready
            or self._target_ctrl_on
            or self._target_ready
            or self._waiting_for_arm_ready
            or self.arm.control_busy
        ):
            QMessageBox.warning(
                self,
                "流程状态已变化",
                "确认期间安全门或会话状态发生变化，本次 TARGET_CTRL START 已取消。",
            )
            self._refresh_controls()
            return
        if self.arm.connected and not self.arm.aim_reference_configured:
            QMessageBox.warning(
                self,
                "激光瞄准参考未标定",
                "确认期间激光瞄准标定状态已失效，"
                "本次 TARGET_CTRL START 已取消。",
            )
            self._refresh_controls()
            return
        if self.args.demo and not self.arm.connected:
            self._target_ctrl_on = True
            self._target_ready = True
            self._detection_running = True
            if self.worker is not None:
                self.worker.start_detection()
            self.workflow_state.setText("演示：视觉对准中")
            self.ready_wait_value.setText("模拟 TARGET_READY")
            self._refresh_controls()
            return
        ok, message = self.arm.start_target_state_machine()
        self._log_event(message)
        if not ok:
            QMessageBox.warning(self, "开始点痣未发送", message)
            return
        self._waiting_for_arm_ready = True
        self._ready_after_action = "target_start"
        self._arm_ready_wait_started = time.monotonic()
        self.workflow_state.setText("定靶安全门检查中")
        self.ready_wait_value.setText("等待 TARGET_READY")
        self._set_status(
            "{}；首次锁定前发送 valid=0，锁定后的短抖动不复用旧坐标并有界静默".format(
                message
            )
        )
        self._refresh_controls()

    def _emergency_stop(self) -> None:
        QMessageBox.warning(
            self,
            "硬件急停说明",
            "软件界面不能代替独立硬件急停。紧急情况请立即使用硬件急停；一般安全取消请点击“取消/安全停止”。",
        )

    def _point_short_name(self, point_id: int) -> str:
        return {1: "左", 2: "中", 3: "右"}.get(int(point_id), str(point_id))

    def _point_full_name(self, point_id: int) -> str:
        return {1: "左脸", 2: "正脸", 3: "右脸"}.get(int(point_id), str(point_id))

    def _prepare_face_capture_session(self) -> bool:
        """Create a unique directory before any three-view movement starts."""
        root = Path(getattr(self, "_face_capture_root", DEFAULT_FACE_CAPTURE_DIR))
        stamp = QDateTime.currentDateTime().toString("yyyyMMdd_HHmmss_zzz")
        try:
            root.mkdir(parents=True, exist_ok=True)
            session_dir = root / stamp
            suffix = 1
            while session_dir.exists():
                session_dir = root / "{}_{}".format(stamp, suffix)
                suffix += 1
            session_dir.mkdir()
        except Exception as exc:
            self._capture_session_dir = None
            self._capture_file_paths = {}
            self._capture_persistence_errors = {}
            self._set_status("三视图原始图保存目录创建失败：{}".format(exc))
            self._log_event("三视图原始图保存目录创建失败：{}".format(exc))
            return False

        self._capture_session_dir = session_dir
        self._capture_file_paths = {}
        self._capture_persistence_errors = {}
        self._log_event("三视图原始图保存目录：{}".format(session_dir))
        return True

    def _capture_faces_button_clicked(self) -> None:
        if IMPORT_ERROR is not None:
            QMessageBox.critical(
                self,
                "视觉环境加载失败",
                "当前 Python：{}\n实际错误：{}\n\n请关闭界面后重新使用 run_qt_app.sh 启动。".format(
                    sys.executable,
                    IMPORT_ERROR,
                ),
            )
            return
        if self.pages.currentIndex() == PAGE_END_CAMERA:
            if self._capture_sequence_active:
                self._capture_pending_face_frame("按钮上传")
                return
            if not (self.args.demo or self.args.protocol_debug_controls):
                QMessageBox.information(self, "正式流程门控", "请从总览或双目页点击采集人脸图；正式流程必须先完成安全测距。")
                return
            self._start_manual_face_capture_sequence()
            return
        if self._capture_sequence_active:
            self._capture_pending_face_frame("按钮上传")
            return
        self._start_face_capture_sequence()

    def _start_manual_face_capture_sequence(self) -> None:
        if not (self.args.demo or self.args.protocol_debug_controls):
            return
        if self._safety_blocking:
            QMessageBox.warning(self, "安全距离危险", "人脸距离过近，已禁止采集三视图。")
            return
        if (
            self._waiting_for_arm_ready
            or self._waiting_for_safe_distance
            or self._detection_running
            or self._reference_capturing
            or self._safety_capturing
        ):
            return
        if not self._prepare_face_capture_session():
            QMessageBox.warning(self, "三视图采集失败", "无法创建原始照片保存目录，请检查磁盘空间和目录权限。")
            return

        self._switch_page(PAGE_END_CAMERA)
        self._capture_sequence_active = True
        self._capture_sequence_manual = True
        self._capture_sequence_points = [1, 2, 3]
        self._capture_sequence_index = 0
        self._capture_pending_point = 1
        self._selected_capture_point = 0
        self.selected_view_value.setText("未选择")
        self._update_capture_selection_ui()
        for point_id in self._capture_sequence_points:
            self._set_face_capture_state(point_id, "等待上传", clear_image=True)
        self.capture_progress_value.setText("正在上传 0/3")
        self.workflow_state.setText("本地三视图采集中")
        self.ready_wait_value.setText("上传左视图")
        self._set_status("本地采集模式：点击采集人脸图依次上传左 / 中 / 右视图")
        self._log_event("本地三视图采集开始")
        self._capture_pending_face_frame("按钮上传")

    def _start_face_capture_sequence(self) -> None:
        if self._safety_blocking:
            QMessageBox.warning(self, "安全距离危险", "人脸距离过近，已禁止采集三视图。")
            return
        if not self._distance_safe_latched:
            QMessageBox.information(self, "等待安全距离", "只有 RA6M5 回传 DISTANCE_SAFE_LATCHED 后才能采集人脸图。")
            return
        if (
            self._waiting_for_arm_ready
            or self._waiting_for_safe_distance
            or self._detection_running
            or self._reference_capturing
            or self._safety_capturing
            or self._capture_sequence_active
        ):
            return
        if not self._prepare_face_capture_session():
            QMessageBox.warning(self, "三视图采集失败", "无法创建原始照片保存目录，请检查磁盘空间和目录权限。")
            return

        self._preposition_ready = False
        self._capture_complete = False
        self._switch_page(PAGE_END_CAMERA)
        if self.worker is not None:
            self.worker.stop_detection()
        self._capture_sequence_active = True
        self._capture_sequence_manual = False
        self._capture_sequence_points = [1, 2, 3]
        self._capture_sequence_index = 0
        self._capture_pending_point = 0
        self._selected_capture_point = 0
        self.selected_view_value.setText("未选择")
        self._update_capture_selection_ui()
        for point_id in self._capture_sequence_points:
            self._set_face_capture_state(point_id, "等待采集", clear_image=True)
        self.capture_progress_value.setText("正在采集 0/3")
        self.workflow_state.setText("三视图采集中")
        self.ready_wait_value.setText("等待采集前 HOME")
        self._set_status("开始采集左 / 中 / 右三张人脸图")
        self._log_event("开始采集三视图人脸图")

        if not self.arm.connected:
            if not (self.args.demo or self.args.protocol_debug_controls):
                self._capture_sequence_active = False
                self.workflow_state.setText("机械臂连接已断开")
                self.ready_wait_value.setText("请检查串口")
                self._set_status("RA6M5 已断开，正式三视图流程不能使用本地模拟旁路")
                self._refresh_controls()
                QMessageBox.warning(self, "机械臂连接已断开", self.arm.connection_text())
                return
            self._log_event("RA6M5 未连接，使用当前 RGB 预览生成三视图演示")
            QTimer.singleShot(700, self._store_demo_face_captures)
            return

        self._capture_home_phase = "before"
        ok, message = self.arm.capture_home()
        self._set_status(message)
        self._log_event(message)
        if not ok:
            self._capture_sequence_active = False
            self._capture_home_phase = ""
            self.workflow_state.setText("采集前 HOME 失败")
            QMessageBox.warning(self, "三视图采集失败", message)
        self._refresh_controls()

    def _store_demo_face_captures(self) -> None:
        if not self._capture_sequence_active:
            return
        for point_id in self._capture_sequence_points:
            self._store_face_capture(point_id, "演示采集")
        self._capture_sequence_active = False
        collected = sum(1 for item in self._capture_images.values() if item is not None)
        if collected == 3:
            self._capture_complete = True
            self.capture_progress_value.setText("三视图已采集 3/3")
            self.workflow_state.setText("三视图已采集")
            self._set_status("三视图演示采集完成")
            self._switch_page(PAGE_TRIPTYCH)
            self._start_triptych_detection()
        else:
            self.capture_progress_value.setText("已采集 {}/3，等待有效画面".format(collected))
            self.workflow_state.setText("三视图未完成")
            self._set_status("当前没有足够的 RGB 画面，请等预览稳定后再采集")
        self.ready_wait_value.setText("--")
        self._refresh_controls()

    def _request_next_face_capture(self) -> None:
        if self._closing_requested or not self._capture_sequence_active:
            return
        if self._capture_sequence_index >= len(self._capture_sequence_points):
            self._finish_face_capture_sequence()
            return

        point_id = self._capture_sequence_points[self._capture_sequence_index]
        self._capture_pending_point = 0
        self._capture_ready_frame_time = 0.0
        self.ready_wait_value.setText("等待{}拍摄点".format(self._point_short_name(point_id)))
        ok, message = self.arm.capture_move(point_id)
        self._set_status(message)
        self._log_event(message)
        if ok:
            self.workflow_state.setText("等待{}拍摄点".format(self._point_short_name(point_id)))
            self._switch_page(PAGE_END_CAMERA)
            self._set_face_capture_state(point_id, "机械臂移动中", clear_image=False)
            self._refresh_controls()
            return

        self._capture_sequence_active = False
        self._capture_sequence_manual = False
        self._capture_pending_point = 0
        self._capture_ready_frame_time = 0.0
        self.workflow_state.setText("三视图采集失败")
        self.ready_wait_value.setText("--")
        self.capture_progress_value.setText("采集失败")
        self._refresh_controls()
        QMessageBox.warning(self, "三视图采集失败", message)

    def _handle_capture_point_ready(self, point_id: int) -> None:
        if point_id not in (1, 2, 3):
            return
        if not self._capture_sequence_active:
            return
        if self._capture_sequence_index >= len(self._capture_sequence_points):
            return

        expected = self._capture_sequence_points[self._capture_sequence_index]
        if point_id != expected:
            self._log_event(
                "收到{}拍摄点，但当前等待{}".format(
                    self._point_short_name(point_id),
                    self._point_short_name(expected),
                )
            )
            return

        self._capture_pending_point = point_id
        self._capture_ready_frame_time = time.monotonic()
        self.workflow_state.setText("{}拍摄点已到位".format(self._point_short_name(point_id)))
        self.ready_wait_value.setText("等待到位后的新画面")
        self._set_face_capture_state(point_id, "已到位，等待新帧", clear_image=False)
        self._set_status("{}拍摄点已到位，等待到位后采集的新鲜画面".format(self._point_full_name(point_id)))
        self._log_event("{}拍摄点已到位，自动保存当前新鲜画面".format(self._point_full_name(point_id)))
        self._refresh_controls()

    def _capture_fresh_frame_if_ready(self, sample_time: float) -> None:
        if (
            self._capture_sequence_active
            and not self._capture_sequence_manual
            and self._capture_pending_point in (1, 2, 3)
            and sample_time > self._capture_ready_frame_time
        ):
            QTimer.singleShot(
                0,
                lambda: self._capture_pending_face_frame("到位后首个新鲜帧"),
            )

    def _capture_face_frame_with_f7(self) -> None:
        self._capture_pending_face_frame("F7采集")

    def _capture_pending_face_frame(self, source_text: str) -> None:
        if not self._capture_sequence_active:
            self._set_status("当前不在三视图采集流程，拍照未执行")
            return
        if self._capture_pending_point not in (1, 2, 3):
            self._set_status("还没有拍摄点到位，请等待 RA6M5 到位后再采集")
            return

        point_id = self._capture_pending_point
        min_frame_time = 0.0 if self._capture_sequence_manual else self._capture_ready_frame_time
        stored = self._store_face_capture(point_id, source_text, min_frame_time=min_frame_time)
        if not stored:
            self.workflow_state.setText("等待{}画面".format(self._point_short_name(point_id)))
            self.ready_wait_value.setText("等待 RGB")
            self._set_status("尚未收到拍摄点到位后的新画面，继续等待")
            return

        self._capture_pending_point = 0
        self._capture_ready_frame_time = 0.0
        self._capture_sequence_index += 1
        captured = min(self._capture_sequence_index, len(self._capture_sequence_points))
        progress_action = "上传" if self._capture_sequence_manual else "采集"
        self.capture_progress_value.setText("正在{} {}/3".format(progress_action, captured))
        if self._capture_sequence_index >= len(self._capture_sequence_points):
            self._finish_face_capture_sequence(send_home=not self._capture_sequence_manual)
            return
        if self._capture_sequence_manual:
            next_point = self._capture_sequence_points[self._capture_sequence_index]
            self._capture_pending_point = next_point
            self.workflow_state.setText("本地三视图采集中")
            self.ready_wait_value.setText("上传{}视图".format(self._point_short_name(next_point)))
            self._set_face_capture_state(next_point, "等待上传", clear_image=False)
            self._set_status(
                "{}视图已上传，请点击采集人脸图上传{}视图".format(
                    self._point_short_name(point_id),
                    self._point_short_name(next_point),
                )
            )
            self._refresh_controls()
            return
        QTimer.singleShot(350, self._request_next_face_capture)
        self._refresh_controls()

    def _finish_face_capture_sequence(self, send_home: bool = True) -> None:
        manual_capture = self._capture_sequence_manual
        self._capture_pending_point = 0
        if send_home and self.arm.connected:
            self._capture_home_phase = "after"
            self.workflow_state.setText("三视图完成，返回 HOME")
            self.ready_wait_value.setText("等待采集后 HOME")
            ok, message = self.arm.capture_home()
            self._log_event(message)
            self._set_status(message)
            if not ok:
                self._capture_sequence_active = False
                self._capture_home_phase = ""
                self.workflow_state.setText("采集后 HOME 失败")
                QMessageBox.warning(self, "三视图回 HOME 失败", message)
            self._refresh_controls()
            return
        self._complete_face_capture_sequence_after_home(manual_capture=manual_capture)

    def _complete_face_capture_sequence_after_home(self, manual_capture: bool = False) -> None:
        self._capture_sequence_active = False
        self._capture_sequence_manual = False
        self._capture_home_phase = ""
        self._capture_complete = True
        self.workflow_state.setText("三视图已采集并回 HOME")
        self.ready_wait_value.setText("请选择视图")
        self.capture_progress_value.setText("三视图已采集 3/3")
        self._set_status("左 / 正 / 右人脸图采集完成，请选择一张视图")
        self._log_event("三视图采集完成且 CAPTURE_HOME_READY")
        self._switch_page(PAGE_TRIPTYCH)
        self._start_triptych_detection()
        if manual_capture:
            self._set_status("本地三视图上传完成，已切换到三视图页面")
        self._refresh_controls()

    def _start_triptych_detection(self) -> None:
        if self._closing_requested:
            return
        images = [
            (point_id, image)
            for point_id, image in sorted(self._capture_images.items())
            if image is not None
        ]
        if len(images) < 3:
            self._set_status("三视图未完整，暂不后台识别黑痣")
            return
        if self.triptych_worker is not None and self.triptych_worker.isRunning():
            self._log_event("等待上一轮三视图识别线程退出")
            return
        for point_id, state in self.face_state_labels.items():
            if self._capture_images.get(point_id) is not None:
                state.setText("后台识别中")
        self.workflow_state.setText("三视图后台识别")
        self._set_status("三视图已采集，正在后台识别黑痣")
        self._log_event("三视图后台黑痣识别开始")
        self.triptych_worker = TriptychDetectionWorker(
            images,
            self.args.detector_mode,
            self.spot_params,
            self.spot_roi,
            Path(self.args.facial_face_model),
            self.args.facial_face_conf,
            self.args.facial_face_device,
            self.args.facial_max_candidates,
            self.args.facial_face_imgsz,
            bool(self.args.facial_enhanced_filters),
            bool(self.args.facial_triptych_top_candidate_only),
            bool(self.args.baseline_face_enabled),
            self.args.baseline_face_dir,
        )
        generation = self._protocol_session_generation
        self.triptych_worker.session_generation = generation
        self.triptych_worker.detection_finished.connect(
            lambda results, token=generation: self._set_triptych_detection_results(results, token)
        )
        self.triptych_worker.finished.connect(self._clear_triptych_worker)
        self.triptych_worker.finished.connect(self.triptych_worker.deleteLater)
        self.triptych_worker.start()

    def _clear_triptych_worker(self) -> None:
        worker = self.sender()
        if self.triptych_worker is worker:
            self.triptych_worker = None
            if (
                getattr(worker, "session_generation", -1)
                != self._protocol_session_generation
                and self._capture_complete
                and self._selected_capture_point == 0
            ):
                QTimer.singleShot(0, self._start_triptych_detection)
            self._refresh_controls()

    def _set_triptych_detection_results(self, results, generation: int) -> None:
        if (
            self._closing_requested
            or generation != self._protocol_session_generation
            or not self._capture_complete
        ):
            self._log_event("忽略上一轮迟到的三视图识别结果")
            return
        self._triptych_detection_results = {}
        best_result = None
        best_target = None
        for result in results:
            self._triptych_detection_results[int(result.point_id)] = result
            label = self.face_image_labels.get(result.point_id)
            if label is not None:
                self._set_label_image(label, result.image, "{}未采集".format(self._point_full_name(result.point_id)))
            state = self.face_state_labels.get(result.point_id)
            combo = self.face_candidate_combos.get(result.point_id)
            if combo is not None:
                combo.clear()
                combo.addItem("请选择候选暗斑", None)
                for index, item in enumerate(result.targets, start=1):
                    combo.addItem(
                        "#{}  X={} Y={}  对比度={:.1f}".format(
                            index,
                            item.x,
                            item.y,
                            item.contrast,
                        ),
                        index - 1,
                    )
            if result.targets:
                target = max(result.targets, key=lambda item: item.contrast)
                if state is not None:
                    state.setText("发现黑痣 X{} Y{}".format(target.x, target.y))
                if best_target is None or target.contrast > best_target.contrast:
                    best_target = target
                    best_result = result
            else:
                if state is not None:
                    state.setText(result.message)

        if best_result is None or best_target is None:
            self._clear_current_target("三视图未识别到黑痣")
            self.workflow_state.setText("三视图识别完成")
            self._set_status("三视图后台识别完成：未识别到黑痣")
            self._log_event("三视图后台识别未发现黑痣")
            self._refresh_controls()
            return

        self._selected_capture_point = 0
        self._selected_triptych_target = None
        self.selected_view_value.setText(
            "建议：{}；等待医生选择视图".format(
                self._point_full_name(best_result.point_id)
            )
        )
        selected_state = self.face_state_labels.get(best_result.point_id)
        if selected_state is not None:
            selected_state.setText(
                "建议视图：已标注候选（实时末端画面将自动定靶）"
            )
        self._clear_current_target("等待医生选择左/正/右视图")
        self.workflow_state.setText("等待医生选择视图")
        self._set_status(
            "三视图候选已标注；医生只选择视图，不选择静态痣点。建议视图{}".format(
                self._point_full_name(best_result.point_id)
            )
        )
        self._log_event("三视图建议{}，等待医生选择视图".format(self._point_full_name(best_result.point_id)))
        self._refresh_controls()

    def _rerun_selected_triptych_cv(self, point_id: int) -> bool:
        image = self._capture_images.get(point_id)
        if image is None:
            self._clear_current_target("{}未采集，无法识别黑痣".format(self._point_full_name(point_id)))
            self._set_status("{}没有采集图，无法用 CV 重算黑痣位置".format(self._point_full_name(point_id)))
            return False
        if cv2 is None or np is None or detect_moles is None:
            self._clear_current_target("CV 依赖不可用")
            self._set_status("CV 依赖不可用，无法重算{}黑痣位置".format(self._point_full_name(point_id)))
            return False

        frame = _qimage_to_bgr(image)
        if frame is None:
            self._clear_current_target("{}图像转换失败".format(self._point_full_name(point_id)))
            self._set_status("{}图像转换失败，无法重算黑痣位置".format(self._point_full_name(point_id)))
            return False

        targets, message, shown = _detect_moles_on_bgr(frame, self.spot_params, self.spot_roi)
        result = TriptychDetectionResult(
            point_id=point_id,
            image=_bgr_to_qimage(shown),
            targets=targets,
            message=message,
            frame_width=int(frame.shape[1]),
            frame_height=int(frame.shape[0]),
        )
        self._triptych_detection_results[int(point_id)] = result
        label = self.face_image_labels.get(point_id)
        if label is not None:
            self._set_label_image(label, result.image, "{}未采集".format(self._point_full_name(point_id)))

        if not targets:
            self._clear_current_target("{}未识别到黑痣".format(self._point_full_name(point_id)))
            state = self.face_state_labels.get(point_id)
            if state is not None:
                state.setText(result.message)
            self._set_status("{}CV 重算完成：{}".format(self._point_full_name(point_id), result.message))
            return False

        target = max(targets, key=lambda item: item.contrast)
        state = self.face_state_labels.get(point_id)
        if state is not None:
            state.setText("已选择：黑痣 X{} Y{}".format(target.x, target.y))
        self._set_result(
            DetectionResult(
                detected=True,
                targets=[target],
                frame_width=result.frame_width,
                frame_height=result.frame_height,
                fps=0.0,
                mode="选中视图CV",
                message="{}CV重算".format(self._point_full_name(point_id)),
                safety_level=SAFETY_SAFE,
                safety_message="安全检测未启用",
            )
        )
        self._set_status(
            "{}CV 重算完成：黑痣 X{} Y{}".format(
                self._point_full_name(point_id),
                target.x,
                target.y,
            )
        )
        self._log_event(
            "{}选中后 CV 重算黑痣 X{} Y{}".format(
                self._point_full_name(point_id),
                target.x,
                target.y,
            )
        )
        return True

    def _set_face_capture_state(self, point_id: int, text: str, clear_image: bool) -> None:
        if clear_image:
            self._capture_images[point_id] = None
            self._triptych_detection_results.pop(point_id, None)
            combo = self.face_candidate_combos.get(point_id)
            if combo is not None:
                combo.clear()
                combo.addItem("请选择候选暗斑", None)
            label = self.face_image_labels.get(point_id)
            if label is not None:
                label.clear()
                label.setText("{}未采集".format(self._point_full_name(point_id)))
        state = self.face_state_labels.get(point_id)
        if state is not None:
            state.setText(text)

    def _store_face_capture(
        self,
        point_id: int,
        source_text: str,
        min_frame_time: float = 0.0,
    ) -> bool:
        # Formal three-view capture accepts only the untouched frame delivered
        # by the end-camera worker after CAPTURE_POINT_READY.  Preview caches
        # must never substitute for the treatment camera source.
        if (
            self._latest_frame_image is None
            or self._latest_frame_image_time <= min_frame_time
        ):
            self._set_face_capture_state(point_id, "无可用画面", clear_image=False)
            return False

        saved = self._latest_frame_image.copy()
        session_dir = getattr(self, "_capture_session_dir", None)
        capture_path = None
        persistence_error = ""
        if session_dir is not None:
            point_label = {1: "left", 2: "front", 3: "right"}.get(
                int(point_id), str(point_id)
            )
            capture_path = Path(session_dir) / "point_{}_{}_raw.png".format(
                int(point_id), point_label
            )
            temporary_path = capture_path.with_name(capture_path.name + ".tmp")
            try:
                if not saved.save(str(temporary_path), "PNG"):
                    raise OSError("QImage PNG 编码失败")
                os.replace(str(temporary_path), str(capture_path))
            except Exception as exc:
                try:
                    temporary_path.unlink()
                except OSError:
                    pass
                persistence_error = str(exc)
                capture_path = None

        self._capture_images[point_id] = saved
        if capture_path is not None:
            self._capture_file_paths[point_id] = capture_path
            self._capture_persistence_errors.pop(point_id, None)
        elif persistence_error:
            self._capture_persistence_errors[point_id] = persistence_error
        label = self.face_image_labels.get(point_id)
        if label is not None:
            self._set_label_image(label, saved, "{}未采集".format(self._point_full_name(point_id)))
        stamp = QDateTime.currentDateTime().toString("HH:mm:ss")
        state = self.face_state_labels.get(point_id)
        if state is not None:
            suffix = "（原始图落盘失败）" if persistence_error else ""
            state.setText("{} {}{}".format(source_text, stamp, suffix))
        collected = sum(1 for item in self._capture_images.values() if item is not None)
        self.capture_progress_value.setText("已采集 {}/3".format(collected))
        if persistence_error:
            self._set_status(
                "{}画面已保留在内存，但原始 PNG 保存失败：{}".format(
                    self._point_full_name(point_id), persistence_error
                )
            )
            self._log_event(
                "{}原始图保存失败，继续完成回 HOME：{}".format(
                    self._point_full_name(point_id), persistence_error
                )
            )
        elif capture_path is None:
            self._log_event("{}人脸图已保存到界面缓存".format(self._point_full_name(point_id)))
        else:
            self._log_event(
                "{}人脸原始图已保存：{}".format(self._point_full_name(point_id), capture_path)
            )
        return True

    def _send_capture_move(self, point_id: int) -> None:
        if self._safety_blocking:
            QMessageBox.warning(self, "安全距离危险", "人脸距离过近，已禁止进入拍摄点。")
            return
        ok, message = self.arm.capture_move(point_id)
        self._set_status(message)
        self._log_event(message)
        if ok:
            self.workflow_state.setText("等待拍摄点 {}".format(point_id))
            self._switch_page(PAGE_END_CAMERA)
        else:
            QMessageBox.warning(self, "拍摄点命令未发送", message)

    def _send_capture_home(self) -> None:
        ok, message = self.arm.capture_home()
        self._set_status(message)
        self._log_event(message)
        if ok:
            self.workflow_state.setText("三视图回 HOME")
        else:
            QMessageBox.warning(self, "回 HOME 命令未发送", message)

    def _selected_candidate_for_point(self, point_id: int) -> Optional[DetectionTarget]:
        result = self.__dict__.get("_triptych_detection_results", {}).get(
            int(point_id)
        )
        combo = self.__dict__.get("face_candidate_combos", {}).get(int(point_id))
        if result is None or not result.targets:
            return None
        if combo is None:
            return result.targets[0]
        target_index = combo.currentData()
        if target_index is None:
            return result.targets[0]
        try:
            return result.targets[int(target_index)]
        except (IndexError, TypeError, ValueError):
            return result.targets[0]

    def _send_capture_select(self, point_id: int) -> None:
        if not self._distance_safe_latched:
            QMessageBox.warning(self, "安全门未满足", "本轮安全距离尚未锁存。")
            return
        if not self._capture_complete:
            QMessageBox.information(self, "三视图未完成", "必须等待采集后的 CAPTURE_HOME_READY，才能选择视图。")
            return
        if not self.args.demo:
            current = time.monotonic()
            decision = self._scene_guard.poll(current)
            if decision.abort:
                self._handle_scene_guard_decision(decision)
            scene_ready, scene_reason = self._scene_ready_for_view_selection(current)
            if not scene_ready:
                QMessageBox.warning(
                    self,
                    "场景监测暂不可用",
                    "{}，已禁止发送 CAPTURE_SELECT；"
                    "请保持人脸与机械臂静止，等监测恢复。".format(scene_reason),
                )
                return
        if self.arm.control_busy:
            return
        self._selected_capture_point = int(point_id)
        self._selected_triptych_target = self._selected_candidate_for_point(point_id)
        self._selected_view_ready = False
        self._selected_view_ready_time = 0.0
        self._reset_target_identity_lock()
        self._detection_running = False
        if self.worker is not None:
            self.worker.set_baseline_face_view(_triptych_view_name(point_id))
            self.worker.stop_detection()
        self.selected_view_value.setText(self._point_full_name(point_id))
        self._update_capture_selection_ui()
        if self._selected_triptych_target is not None:
            self._clear_current_target(
                "等待选中视图实时识别；三视图锚点 X{} Y{}".format(
                    int(self._selected_triptych_target.x),
                    int(self._selected_triptych_target.y),
                )
            )
            self._log_event(
                "已保存{}三视图定靶锚点 X{} Y{}，实时锁定将优先匹配该候选".format(
                    self._point_full_name(point_id),
                    int(self._selected_triptych_target.x),
                    int(self._selected_triptych_target.y),
                )
            )
        else:
            self._clear_current_target("等待选中视图实时识别")
        ok, message = self.arm.capture_select(point_id)
        self._set_status(message)
        self._log_event(message)
        if ok:
            self.workflow_state.setText("等待定靶预开始点 {}".format(point_id))
            self._switch_page(PAGE_END_CAMERA)
        else:
            if self.arm.connected:
                QMessageBox.warning(self, "视图选择命令未发送", message)
            elif self.args.demo:
                self._selected_view_ready = True
                self.workflow_state.setText("已选择{}，RA6M5 未连接".format(self._point_full_name(point_id)))
                self._switch_page(PAGE_END_CAMERA)
                if self.worker is not None:
                    self.worker.start_detection()
                self._refresh_controls()

    def _request_end_camera_detect_once(self) -> None:
        if self.pages.currentIndex() != PAGE_END_CAMERA:
            return
        if self.worker is None:
            return
        self.worker.request_detect_once()
        if self._selected_capture_point in (1, 2, 3):
            self.workflow_state.setText("{}末端视觉 CV".format(self._point_full_name(self._selected_capture_point)))
        self._set_status("已请求单目末端 CV 单次识别")
        self._log_event("选中视图后触发单目末端 CV 单次识别")

    def _update_capture_selection_ui(self) -> None:
        for point_id, button in self.face_select_buttons.items():
            button.setProperty("selected", point_id == self._selected_capture_point)
            button.style().unpolish(button)
            button.style().polish(button)
            button.update()
        for point_id, state in self.face_state_labels.items():
            if point_id == self._selected_capture_point:
                state.setText("{}已选择".format(self._point_full_name(point_id)))
            elif state.text().endswith("已选择"):
                state.setText("已采集" if self._capture_images.get(point_id) is not None else "未采集")

    def _current_primary_target(self) -> Optional[DetectionTarget]:
        if self.current_result is None or not self.current_result.targets:
            return None
        return self.current_result.targets[0]

    def _current_arm_target_with_reason(self) -> Tuple[Optional[ArmTarget], str]:
        if not self._detection_running:
            return None, "末端连续识别未运行"
        if self.pages.currentIndex() != PAGE_END_CAMERA:
            return None, "当前不在末端视觉页"
        if self.worker is None or not self.worker.isRunning():
            return None, "末端摄像头线程未运行"
        if self.arm.connected and not self.arm.aim_reference_configured:
            return None, "激光瞄准参考像素未标定"
        if self._latest_result_time <= 0.0:
            return None, "尚无末端识别结果"
        if self._latest_result_time <= self._selected_view_ready_time:
            return None, "识别结果早于选中视图到位"
        result_age = time.monotonic() - self._latest_result_time
        if result_age > self.vision_target_max_age:
            return None, "末端识别结果过期 {:.0f}ms".format(result_age * 1000.0)
        if self._locked_current_target is None:
            return None, (
                getattr(self, "_last_target_lock_reason", "")
                or "候选尚未通过身份稳定锁"
            )
        if self.current_result is None:
            return None, "末端识别结果为空"
        if self._locked_target_sample_time != self._latest_result_time:
            return None, "锁定目标与最新帧不一致"
        target = self._locked_current_target
        return (
            ArmTarget(
                pixel=(target.x, target.y),
                frame_size=(
                    self.current_result.frame_width,
                    self.current_result.frame_height,
                ),
            ),
            "稳定目标有效",
        )

    def _current_arm_target(self) -> Optional[ArmTarget]:
        target, _reason = self._current_arm_target_with_reason()
        return target

    def _publish_latest_vision_result(self) -> None:
        """Hand one small result snapshot to the serial I/O worker."""

        # Some unit tests deliberately construct a lightweight MainWindow via
        # __new__ without initializing the SIP/QMainWindow base.  __dict__ is
        # also the safest way to keep this helper a no-op for those stubs.
        arm = self.__dict__.get("arm")
        if arm is None or not self._target_ready or not getattr(
            arm,
            "background_vision_enabled",
            False,
        ):
            return
        target, reason = self._current_arm_target_with_reason()
        publish = getattr(arm, "publish_target_sample", None)
        if publish is None:
            return
        now = time.monotonic()
        state = "VALID" if target is not None else "INVALID"
        invalid_reason = _classify_vision_invalid_reason(
            target_valid=target is not None,
            gate_reason=reason,
            result=self.current_result,
            aim_reference_configured=(
                not bool(getattr(arm, "connected", False))
                or bool(getattr(arm, "aim_reference_configured", False))
            ),
        )
        # A result that is only slightly older than the coordinate freshness
        # limit is not a new camera observation.  Publishing it as an explicit
        # valid=0 makes RA6M5 leave ALIGN even when the next fresh result is
        # already being processed.  Stay silent here; SerialSender owns the
        # longer loss timeout and emits one fail-closed invalid only if no
        # fresh result arrives during its bounded grace window.  Real detector
        # invalidations still publish immediately.
        did_publish = invalid_reason != "RESULT_STALE"
        publish_message = ""
        previous_valid = bool(
            self.__dict__.get("_vision_last_published_valid", False)
        )
        invalid_transition_sent = bool(
            self.__dict__.get("_vision_invalid_transition_sent", False)
        )
        transient_started_at = float(
            self.__dict__.get("_vision_transient_loss_started_at", 0.0)
        )

        if target is not None:
            self._vision_transient_loss_started_at = 0.0
            self._vision_invalid_transition_sent = False
        elif invalid_reason == "RESULT_STALE":
            # SerialSender ages the last valid mailbox sample independently;
            # never turn a repeated UI poll of the same frame into a new loss.
            did_publish = False
        elif invalid_reason in VISION_TRANSIENT_INVALID_REASONS and previous_valid:
            if transient_started_at <= 0.0:
                transient_started_at = now
                self._vision_transient_loss_started_at = transient_started_at
            loss_age_s = max(0.0, now - transient_started_at)
            if loss_age_s < VISION_TRANSIENT_LOSS_GRACE_S:
                # Do not replay the last coordinate and do not enqueue a
                # one-frame valid=0.  RA6M5's independent watchdog remains the
                # outer fail-safe if the camera stops producing frames.
                did_publish = False
                publish_message = (
                    "同一目标短时失配 {:.0f}ms，串口静默等待新鲜帧"
                ).format(loss_age_s * 1000.0)
            else:
                # The bounded grace expired: emit one explicit transition to
                # invalid, then suppress duplicate invalid frames.
                did_publish = not invalid_transition_sent
        else:
            self._vision_transient_loss_started_at = 0.0
            # Critical/non-transient invalid reasons are immediate, but one
            # state transition is sufficient; repeated valid=0 frames only
            # consume UART bandwidth and cannot make the state safer.
            did_publish = not invalid_transition_sent

        if did_publish:
            ok, message = publish(target, self._latest_result_time)
            if ok:
                self._vision_last_published_valid = target is not None
                self._vision_invalid_transition_sent = target is None
                if target is None:
                    self._vision_transient_loss_started_at = 0.0
        else:
            ok = True
            message = publish_message or "视觉状态未变化，串口保持静默"
        if (
            reason != self._last_vision_gate_reason
            or invalid_reason != getattr(self, "_last_vision_invalid_reason", "")
            or now - self._last_vision_gate_log_time >= 0.5
        ):
            result_age_ms = (
                max(0.0, now - self._latest_result_time) * 1000.0
                if self._latest_result_time > 0.0
                else float("inf")
            )
            target_count = (
                len(self.current_result.targets)
                if self.current_result is not None
                else 0
            )
            diagnostic_target = None
            if self._locked_current_target is not None:
                diagnostic_target = self._locked_current_target
            elif self.current_result is not None and self.current_result.targets:
                diagnostic_target = self.current_result.targets[0]
            stable_count = int(getattr(diagnostic_target, "stable_count", 0))
            required_count = int(getattr(diagnostic_target, "required_count", 0))
            target_age_ms = max(
                0.0,
                float(getattr(diagnostic_target, "track_age_s", 0.0)) * 1000.0,
            )
            candidate_track_id = int(getattr(diagnostic_target, "track_id", 0))
            locked_track_id = int(getattr(self._target_identity_lock, "track_id", 0))
            candidate_x = float(getattr(diagnostic_target, "x", -1.0))
            candidate_y = float(getattr(diagnostic_target, "y", -1.0))
            candidate_radius = float(getattr(diagnostic_target, "radius", -1.0))
            locked_x = locked_y = locked_radius = -1.0
            rebind_distance_px = rebind_gate_px = -1.0
            anchor = getattr(self._target_identity_lock, "anchor", None)
            if anchor is not None and self.current_result is not None:
                frame_diagonal = math.hypot(
                    self.current_result.frame_width,
                    self.current_result.frame_height,
                )
                locked_x = float(anchor[0]) * frame_diagonal
                locked_y = float(anchor[1]) * frame_diagonal
                locked_radius = float(anchor[2]) * frame_diagonal
                if diagnostic_target is not None:
                    rebind_distance_px = math.hypot(
                        candidate_x - locked_x,
                        candidate_y - locked_y,
                    )
                    rebind_gate_px = min(
                        float(self._target_identity_lock.max_step_ratio)
                        * frame_diagonal,
                        max(locked_radius, candidate_radius)
                        * float(
                            self._target_identity_lock.track_id_rebind_radius_factor
                        ),
                    )
            last_locked_sample_time = float(
                getattr(
                    self._target_identity_lock,
                    "last_locked_sample_time",
                    0.0,
                )
            )
            lost_age_ms = (
                max(0.0, now - last_locked_sample_time) * 1000.0
                if last_locked_sample_time > 0.0
                else -1.0
            )
            detector_message = (
                str(getattr(self.current_result, "message", "") or "--")
                if self.current_result is not None
                else "--"
            )
            raw_candidate_count = int(
                getattr(self.current_result, "raw_candidate_count", 0)
                if self.current_result is not None
                else 0
            )
            detector_reason = str(
                getattr(self.current_result, "detector_reason", "") or "--"
                if self.current_result is not None
                else "--"
            )
            print(
                "[VISION_GATE {}] {} invalid_reason={} reason={} result_age_ms={:.1f} "
                "raw_candidates={} targets={} stable={}/{} target_age_ms={:.1f} "
                "locked_track_id={} candidate_track_id={} "
                "locked_xy=({:.1f},{:.1f})/r{:.1f} "
                "candidate_xy=({:.1f},{:.1f})/r{:.1f} "
                "rebind_distance_px={:.1f} rebind_gate_px={:.1f} "
                "lost_age_ms={:.1f} "
                "lock_active={} lock_lost={} lock_confirm={}/{} published={} "
                "detector_reason={} detector={}".format(
                    time.strftime("%H:%M:%S"),
                    state,
                    invalid_reason,
                    reason,
                    result_age_ms,
                    raw_candidate_count,
                    target_count,
                    stable_count,
                    required_count,
                    target_age_ms,
                    locked_track_id,
                    candidate_track_id,
                    locked_x,
                    locked_y,
                    locked_radius,
                    candidate_x,
                    candidate_y,
                    candidate_radius,
                    rebind_distance_px,
                    rebind_gate_px,
                    lost_age_ms,
                    1 if self._target_identity_lock.active else 0,
                    1 if getattr(self._target_identity_lock, "lost", False) else 0,
                    int(getattr(self._target_identity_lock, "pending_count", 0)),
                    int(getattr(self._target_identity_lock, "stable_frames", 0)),
                    1 if did_publish and ok else 0,
                    detector_reason,
                    detector_message,
                ),
                flush=True,
            )
            self._last_vision_gate_reason = reason
            self._last_vision_invalid_reason = invalid_reason
            self._last_vision_gate_log_time = now
        if not ok and "重复或倒退" not in message and "duplicate or out-of-order" not in message:
            self._set_status("视觉结果发布失败：{}".format(message))

    def _poll_arm_status(self) -> None:
        # Local scene safety is evaluated before any remote status can trigger
        # the next GOTO/SELECT action in this tick.
        self._poll_scene_guard()
        abort_request_in_flight = (
            self._ready_after_action == "abort_hold" or self.arm.abort_pending
        )
        statuses = self.arm.poll_statuses()

        def recoverable_vision_lost(status: object) -> bool:
            return (
                str(getattr(status, "protocol", "")) == "unified"
                and int(getattr(status, "error_code", 0)) == 0x0A
            )

        local_stop_in_flight = self._scene_abort_triggered or abort_request_in_flight
        batch_has_blocker = local_stop_in_flight or any(
            (self._is_ra6_error(status) and not recoverable_vision_lost(status))
            or str(getattr(status, "protocol", "")) == "unified_error"
            or (
                int(getattr(status, "func", -1)) == RA6_WORKFLOW_FUNC
                and int(getattr(status, "value", -1)) == WORKFLOW_FAULT_HOLD
            )
            for status in statuses
        )
        if batch_has_blocker:
            safe_terminal_workflow = {
                WORKFLOW_RETREAT_DONE_WAIT_RESTART,
                WORKFLOW_RETURN_HOME_DONE,
                WORKFLOW_ABORTED_HOLD,
                WORKFLOW_FAULT_HOLD,
                WORKFLOW_IDLE,
            }
            filtered_statuses = []
            for status in statuses:
                is_error = self._is_ra6_error(status)
                is_safe_terminal = (
                    int(getattr(status, "func", -1)) == RA6_WORKFLOW_FUNC
                    and int(getattr(status, "value", -1)) in safe_terminal_workflow
                )
                if is_error or is_safe_terminal:
                    filtered_statuses.append(status)
            if len(filtered_statuses) != len(statuses):
                self._log_event("本地停止或同批致命错误/FAULT 已生效，丢弃可能推进运动的业务状态")
            statuses = filtered_statuses
        else:
            # Preserve receive order across loss and recovery. Positive
            # ALIGN/OUTPUT reports are stale only while loss is active; a
            # later VISION_VALID in this same batch re-enables subsequent
            # reports.
            vision_loss_active = self._recoverable_vision_loss_active
            filtered_statuses = []
            for status in statuses:
                if recoverable_vision_lost(status):
                    vision_loss_active = True
                    filtered_statuses.append(status)
                    continue
                event = int(getattr(status, "func", -1))
                value = int(getattr(status, "value", -1))
                error_code = int(getattr(status, "error_code", 0))
                if (
                    event == RA6_VISION_STATE_FUNC
                    and value == RA6_VALUE_ON
                    and error_code == 0
                ):
                    vision_loss_active = False
                    filtered_statuses.append(status)
                    continue
                if (
                    vision_loss_active
                    and event in (RA6_ALIGN_DONE_FUNC, RA6_OUTPUT_FUNC)
                    and value == RA6_VALUE_ON
                ):
                    continue
                filtered_statuses.append(status)
            if len(filtered_statuses) != len(statuses):
                self._log_event("视觉丢失尚未恢复，已按原顺序丢弃过时的 ALIGN_DONE/OUTPUT_ON")
            statuses = filtered_statuses

        for status in statuses:
            text = (
                f"RA6M5：{status.label} "
                f"protocol={getattr(status, 'protocol', 'legacy')} "
                f"func=0x{status.func:02X} value=0x{status.value:02X} "
                f"error=0x{getattr(status, 'error_code', 0):02X}"
            )
            self.ra6_state.setText(text)
            self._set_status(text)
            self._log_event(text)

            if self._is_ra6_error(status):
                if self._handle_protocol_error(status):
                    continue

            event = int(getattr(status, "func", -1))
            value = int(getattr(status, "value", -1))
            if event == RA6_COMMAND_ACK_FUNC:
                continue
            if event == RA6_WORKFLOW_FUNC:
                self._handle_workflow_status(value)
            elif event == RA6_SAFE_DISTANCE_FUNC:
                self._waiting_for_safe_distance = False
                if value == RA6_VALUE_ON:
                    if (
                        self.serial_safe_distance_source == "depth"
                        and not self.args.demo
                        and not self._approve_safe_scene_snapshot(
                            int(getattr(status, "seq", -1))
                        )
                    ):
                        self._abort_for_scene_change(
                            "SAFE_DISTANCE_OK 未匹配到发送时的人脸场景快照"
                        )
                        continue
                    self._distance_safe_latched = False
                    self._preposition_ready = False
                    self.workflow_state.setText("距离安全，等待流程锁存")
                    self.ready_wait_value.setText("等待 DISTANCE_SAFE_LATCHED")
                    self._set_status("SAFE_DISTANCE_OK：等待本轮流程锁存确认")
                else:
                    self._clear_safe_scene_binding()
                    self._distance_safe_latched = False
                    self._preposition_ready = False
                    self.workflow_state.setText("距离不安全，等待撤离")
            elif event == RA6_CAPTURE_POINT_FUNC:
                self.workflow_state.setText("拍摄点 {} 已到位".format(status.value))
                self.ready_wait_value.setText("可拍照")
                self._handle_capture_point_ready(int(status.value))
            elif event == RA6_CAPTURE_HOME_FUNC and value == RA6_VALUE_ON:
                if self._capture_home_phase == "before":
                    self._capture_home_phase = ""
                    self.workflow_state.setText("采集前 HOME 已完成")
                    self.ready_wait_value.setText("准备左视图")
                    self._request_next_face_capture()
                elif self._capture_home_phase == "after":
                    self._complete_face_capture_sequence_after_home()
                else:
                    self.workflow_state.setText("CAPTURE HOME 已完成")
                    self.ready_wait_value.setText("--")
            elif event == RA6_TARGET_PRESTART_FUNC:
                self.workflow_state.setText("定靶预开始点 {} 已到位".format(status.value))
                if int(status.value) == self._selected_capture_point:
                    self._selected_view_ready = True
                    self._selected_view_ready_time = time.monotonic()
                    self._reset_target_identity_lock()
                    self._clear_current_target("等待选中视图到位后的新检测帧")
                    self.ready_wait_value.setText("可开始点痣")
                    self._switch_page(PAGE_END_CAMERA)
                    if self.worker is not None:
                        self.worker.start_detection()
                    self._detection_running = True
                    self._set_status("选中视图已到位，实时自动跟踪已启动；请点击开始点痣")
            elif event == RA6_TARGET_CTRL_FUNC:
                self._target_ctrl_on = value == RA6_VALUE_ON
                if self._target_ctrl_on:
                    self.workflow_state.setText("定靶已启用，等待 TARGET_READY")
                else:
                    self._target_ready = False
                    self._recoverable_vision_loss_active = False
                    self._reset_target_identity_lock()
            elif event == RA6_READY_FUNC and value == RA6_VALUE_ON:
                if not self._target_ctrl_on:
                    self._set_status("协议顺序错误：TARGET_CTRL_ON 前收到 TARGET_READY，未启动视觉发送")
                    continue
                self._waiting_for_arm_ready = False
                self._ready_after_action = ""
                self._arm_ready_wait_started = None
                self._target_ready = True
                self._recoverable_vision_loss_active = False
                self._detection_running = True
                self._last_serial_vision_error = 0.0
                self._last_sent_vision_sample_time = 0.0
                if self.worker is not None:
                    self.worker.start_detection()
                self.workflow_state.setText("视觉对准中")
                self.ready_wait_value.setText("等待 ALIGN_DONE")
                self._set_status(
                    "TARGET_READY：开始周期发送 VISION_ERROR；"
                    "稳定目标有效时 valid=1，否则 valid=0"
                )
            elif event == RA6_ALIGN_DONE_FUNC and value == RA6_VALUE_ON:
                self._align_done = True
                self.workflow_state.setText("对准完成")
                self.ready_wait_value.setText("请按住 P000")
                self._set_status("ALIGN_DONE：请按住物理 P000；Jetson 不直接打开 P801")
            elif event == RA6_OUTPUT_FUNC:
                self._handle_output_status(value)
            elif event == RA6_VISION_STATE_FUNC:
                self._vision_valid = value == RA6_VALUE_ON
                if self._vision_valid:
                    self._recoverable_vision_loss_active = False
                    self.workflow_state.setText("视觉已恢复，重新对准")
                else:
                    self._recoverable_vision_loss_active = True
                    self._output_on = False
                    self._align_done = False
                    self.workflow_state.setText("视觉丢失，原地等待恢复")

        for event_text in self.arm.service_transport():
            self._log_event(event_text)
            if self._is_terminal_serial_transport_event(event_text):
                self._handle_terminal_serial_transport_event(event_text)
                break
            if any(token in event_text for token in ("超时", "被拒绝", "失败", "时序错误")):
                if (
                    abort_request_in_flight
                    or self._ready_after_action == "abort_hold"
                    or self.arm.abort_pending
                ):
                    reason = "ABORT_HOLD 未获确认，已停止串口心跳：{}".format(event_text)
                    self._stop_automatic_progress(reason, restart_blocked=True)
                    self.arm.close()
                    self.serial_state.setText("串口已关闭，等待 RA6M5 心跳超时保护")
                else:
                    self._cancel_workflow(reason="控制链路异常：{}".format(event_text))
                break
        self._update_arm_ready_wait()
        # Consume and apply every received fault/status before scheduling the
        # next SAFE_DISTANCE or VISION_ERROR sample.
        self._poll_scene_guard()
        self._service_unified_serial_keepalive()
        self._refresh_controls()

    def _handle_output_status(self, value: int) -> None:
        was_output_on = self._output_on
        self._output_on = int(value) == RA6_VALUE_ON
        if self._output_on:
            self.workflow_state.setText("P801 输出中")
            self.ready_wait_value.setText("持续视觉监测")
            return

        if (
            was_output_on
            and self._workflow_active
            and self.arm.connected
            and not self.arm.control_busy
            and self._ready_after_action != "finish_return_home"
        ):
            ok, message = self._request_finish_return_home(
                "OUTPUT_OFF：点痣完成，已自动请求返回 HOME"
            )
            if ok:
                self._log_event("点痣完成后自动返回 HOME：{}".format(message))
            else:
                self.workflow_state.setText("P801 已关闭，原地保持")
                self.ready_wait_value.setText("完成返回发送失败")
                self._set_status("自动返回 HOME 未发送：{}".format(message))
            return

        self.workflow_state.setText("P801 已关闭，原地保持")
        self.ready_wait_value.setText("可完成/返回")

    @staticmethod
    def _is_terminal_serial_transport_event(event_text: str) -> bool:
        text = str(event_text)
        return any(
            marker in text
            for marker in (
                "串口帧短写",
                "串口TX写入失败且无法确认是否已写入部分帧",
                "串口I/O连续失败",
                "SEQ_CONFLICT",
            )
        )

    def _handle_terminal_serial_transport_event(self, event_text: str) -> None:
        # The UART byte stream is no longer trustworthy.  Do not attempt an
        # ABORT frame on the same locked transport: that only produces the
        # misleading secondary dialog "安全停止未发送".  Closing heartbeat lets
        # RA6M5 independently enter its watchdog hold.
        reason = "串口发送已停止，机械臂通信状态未知：{}".format(event_text)
        self._stop_automatic_progress(reason, restart_blocked=True)
        self.arm.close()
        self.serial_state.setText("串口已关闭，请排查后重启 Qt")

    def _handle_protocol_error(self, status: object) -> bool:
        error_code = int(getattr(status, "error_code", 0))
        event = int(getattr(status, "func", -1))
        label = str(getattr(status, "label", "ERROR"))
        if error_code in (
            ERROR_SAFE_DISTANCE_TOO_CLOSE,
            ERROR_SAFE_DISTANCE_TIMEOUT,
        ) and event == RA6_SAFE_DISTANCE_FUNC:
            self._clear_safe_scene_binding()
            self._waiting_for_safe_distance = False
            self._distance_safe_latched = False
            self._preposition_ready = False
            self._latest_safe_measurement_valid = False
            self.ready_wait_value.setText("等待 RETREAT_DONE")
            if error_code == ERROR_SAFE_DISTANCE_TIMEOUT:
                reason = "安全距离更新超时，安全撤离中"
                self._latest_safe_invalid_reason = "RA6M5安全距离更新超时"
                self.workflow_state.setText(reason)
                self._set_stale_safe_distance_text(self._latest_safe_invalid_reason)
                self._set_status("安全距离更新超时：RA6M5 正在执行固化撤离")
            else:
                self._latest_safe_invalid_reason = "安全距离不合格"
                self.workflow_state.setText("距离过近，安全撤离中")
                self._set_status("安全距离不合格：RA6M5 正在执行固化撤离")
            return True
        if error_code == ERROR_VISION_LOST:
            self._output_on = False
            self._vision_valid = False
            self._recoverable_vision_loss_active = True
            self._align_done = False
            self._target_ready = self._target_ready or event == RA6_VISION_STATE_FUNC
            self._detection_running = True
            if self.worker is not None:
                self.worker.start_detection()
            self.workflow_state.setText("视觉丢失，原地等待恢复")
            self.ready_wait_value.setText("持续发送 valid=0/等待恢复")
            self._set_status("VISION_LOST：不退出流程，继续实时识别并等待 valid=1")
            return True
        if error_code == ERROR_SEQ_CONFLICT:
            # A cached (TYPE, SEQ) from an earlier 8-bit generation makes the
            # two endpoints' command epochs ambiguous.  Even an exact-looking
            # ABORT response could be a replay from that stale cache, so stop
            # heartbeat and let the RA6M5 watchdog establish an independent
            # FAULT_HOLD before an operator resets both sides.
            reason = (
                "RA6M5 SEQ缓存与当前会话冲突；已停止串口心跳并禁止重新开始，"
                "请确认机械臂安全保持后复位 RA6M5，再重启 Qt"
            )
            self._stop_automatic_progress(reason, restart_blocked=True)
            self.workflow_state.setText("SEQ会话冲突，等待硬件复位")
            self.ready_wait_value.setText("串口已关闭 / 等待看门狗 HOLD")
            self.arm.close()
            self.serial_state.setText("串口已关闭，请复位 RA6M5 后重启 Qt")
            return True
        if event == RA6_COMMAND_ACK_FUNC:
            error_name = (
                label.rsplit("_ERR_", 1)[-1]
                if "_ERR_" in label
                else "0x{:02X}".format(error_code)
            )
            reason = "RA6M5 拒绝控制命令：{}（{}）".format(error_name, label)
            self._stop_automatic_progress(reason)
            self.workflow_state.setText("命令被拒绝：{}".format(error_name))
            self.ready_wait_value.setText("自动流程已停止")
            return True
        self._stop_automatic_progress("RA6M5 错误：{}".format(label))
        return True

    def _handle_workflow_status(self, value: int) -> None:
        if value == WORKFLOW_IDLE:
            self._reset_protocol_session(clear_captures=False)
            self.workflow_state.setText("待机")
        elif value == WORKFLOW_START_ACCEPTED:
            self._workflow_active = True
            self.workflow_state.setText("开始已接受，安全复位中")
        elif value == WORKFLOW_MEASURE_POSITION_READY:
            self._waiting_for_arm_ready = False
            self._ready_after_action = ""
            self._arm_ready_wait_started = None
            self._measure_position_ready = True
            self._measure_position_ready_time = time.monotonic()
            self._waiting_for_safe_distance = True
            self._last_sent_safe_sample_time = 0.0
            self._latest_safe_depth_mm = 0.0
            self._latest_safe_depth_valid_ratio = 0.0
            self._latest_safe_depth_time = 0.0
            self._latest_safe_rgb_time = 0.0
            self._latest_safe_face_box = None
            self._latest_safe_face_size = (0, 0)
            self._latest_safe_measurement_valid = False
            self._latest_safe_invalid_reason = "等待测距位到达后的新RGB-D采样"
            self._last_rgbd_estimated_detection_time = 0.0
            self._clear_safe_scene_binding()
            self._latest_rgbd_detections = {}
            self._latest_rgbd_detection_time = 0.0
            if self._rgbd_distance_estimator is not None:
                self._rgbd_distance_estimator.reset()
            self.workflow_state.setText("公共测距位已到达")
            self.ready_wait_value.setText("等待安全距离裁决")
            label = self.__dict__.get("binocular_safe_distance_value")
            if label is not None:
                label.setText("等待 arm_tip / face 三维净距")
            self._set_status("MEASURE_POSITION_READY：开始阶段性发送 SAFE_DISTANCE")
        elif value == WORKFLOW_DISTANCE_SAFE_LATCHED:
            self._waiting_for_safe_distance = False
            self._distance_safe_latched = True
            self._preposition_ready = True
            if (
                self.serial_safe_distance_source == "depth"
                and not self.args.demo
                and not self._latch_scene_guard()
            ):
                self._abort_for_scene_change("安全测距样本缺少可追踪的人脸基线")
                return
            self.workflow_state.setText("安全距离已锁存")
            self.ready_wait_value.setText("可采图")
        elif value == WORKFLOW_RETREAT_DONE_WAIT_RESTART:
            self._workflow_active = False
            self._waiting_for_safe_distance = False
            self._waiting_for_arm_ready = False
            self._measure_position_ready = False
            self._measure_position_ready_time = 0.0
            self._distance_safe_latched = False
            self._preposition_ready = False
            self._selected_view_ready = False
            self._target_ctrl_on = False
            self._target_ready = False
            self._detection_running = False
            self._ready_after_action = ""
            self._arm_ready_wait_started = None
            self._reset_target_identity_lock()
            self._clear_safe_scene_binding()
            self._scene_abort_triggered = False
            if self.worker is not None:
                self.worker.stop_detection()
            self.workflow_state.setText("撤离完成，等待重新开始")
            self.ready_wait_value.setText("请点击开始")
            self._activate_page_resources(self.pages.currentIndex())
        elif value == WORKFLOW_RETURN_HOME_DONE:
            self._reset_protocol_session(clear_captures=False)
            self.workflow_state.setText("完成并已返回 HOME")
            self.ready_wait_value.setText("--")
            self._set_status("RETURN_HOME_DONE：流程已回到 IDLE")
            self._switch_page(PAGE_WORKBENCH)
            self._activate_page_resources(PAGE_WORKBENCH)
        elif value == WORKFLOW_ABORTED_HOLD:
            self._reset_protocol_session(clear_captures=False)
            self.workflow_state.setText("已安全停止并原地保持")
            self.ready_wait_value.setText("重新开始前请确认现场安全")
            self._activate_page_resources(self.pages.currentIndex())
        elif value == WORKFLOW_FAULT_HOLD:
            self._stop_automatic_progress("RA6M5 FAULT_HOLD：自动流程已清除")
            self.workflow_state.setText("故障锁存，原地保持")
            self.ready_wait_value.setText("排故后重新开始")
            self._activate_page_resources(self.pages.currentIndex())

    def _stop_automatic_progress(self, reason: str, restart_blocked: bool = False) -> None:
        if self.worker is not None:
            self.worker.stop_detection()
        self._waiting_for_arm_ready = False
        self._waiting_for_safe_distance = False
        self._capture_sequence_active = False
        self._capture_sequence_manual = False
        self._capture_home_phase = ""
        self._capture_pending_point = 0
        self._capture_complete = False
        self._detection_running = False
        self._workflow_active = False
        self._measure_position_ready = False
        self._measure_position_ready_time = 0.0
        self._distance_safe_latched = False
        self._preposition_ready = False
        self._selected_view_ready = False
        self._selected_view_ready_time = 0.0
        self._target_ctrl_on = False
        self._target_ready = False
        self._align_done = False
        self._output_on = False
        self._vision_valid = False
        self._recoverable_vision_loss_active = False
        self._fault_hold = True
        self._restart_blocked = self._restart_blocked or bool(restart_blocked)
        self._ready_after_action = ""
        self._arm_ready_wait_started = None
        self._last_sent_vision_sample_time = 0.0
        self._last_sent_safe_sample_time = 0.0
        self._reset_target_identity_lock()
        self._latest_safe_depth_mm = 0.0
        self._latest_safe_depth_valid_ratio = 0.0
        self._latest_safe_depth_time = 0.0
        self._latest_safe_rgb_time = 0.0
        self._latest_safe_face_box = None
        self._latest_safe_face_size = (0, 0)
        self._latest_safe_measurement_valid = False
        self._latest_safe_invalid_reason = reason
        self._last_rgbd_estimated_detection_time = 0.0
        self._clear_safe_scene_binding()
        self.ready_wait_value.setText("人工处理")
        self._set_status(reason)
        self._activate_page_resources(self.pages.currentIndex())

    def _service_unified_serial_keepalive(self) -> None:
        if not self.arm.connected or self.arm.serial_protocol != "unified":
            return
        now = time.monotonic()
        # HEARTBEAT and RX parsing run in SerialSender's independent I/O
        # thread.  Qt only schedules phase-gated business samples here.
        if (
            self._measure_position_ready
            and self._waiting_for_safe_distance
            and now - self._last_serial_safe_distance >= self.serial_safe_distance_interval
        ):
            self._send_serial_safe_distance_now(now)
        # On a real unified link VISION_ERROR is owned by SerialSender's
        # independent I/O thread.  Keep the old direct branch only for legacy
        # test/dummy controllers that do not expose that scheduler.
        if (
            not getattr(self.arm, "background_vision_enabled", False)
            and self._target_ready
            and now - self._last_serial_vision_error >= self.serial_vision_interval
        ):
            target = self._current_arm_target()
            has_new_sample = (
                target is not None
                and self._latest_result_time > self._last_sent_vision_sample_time
            )
            ok, message = self.arm.send_target_sample(target if has_new_sample else None)
            if ok and has_new_sample:
                self._last_sent_vision_sample_time = self._latest_result_time
            self._last_serial_vision_error = now
            if not ok:
                self._set_status("视觉误差发送失败：{}".format(message))

    def _send_serial_safe_distance_now(self, now: Optional[float] = None) -> bool:
        if not self.arm.connected or self.arm.serial_protocol != "unified":
            return False
        if not self._measure_position_ready or not self._waiting_for_safe_distance:
            return False
        if self._distance_safe_latched:
            return False
        if now is None:
            now = time.monotonic()
        distance_mm, valid, source_text = self._serial_safe_distance_payload(now)
        if (
            self.serial_safe_distance_source == "depth"
            and valid
            and self._latest_safe_depth_time <= self._last_sent_safe_sample_time
        ):
            distance_mm, valid, source_text = 0, False, "RGB-D无新采样"
        if not valid:
            # Never retransmit a stale distance and do not flood invalid
            # business frames. HEARTBEAT remains independent.
            self._last_serial_safe_distance = now
            return False
        ok, message = self.arm.send_safe_distance(distance_mm, valid)
        if (
            ok
            and valid
            and self.serial_safe_distance_source == "depth"
        ):
            seq = self.arm.last_safe_distance_seq
            try:
                face_box = tuple(float(value) for value in self._latest_safe_face_box)
            except (TypeError, ValueError):
                face_box = ()
            if (
                seq is None
                or len(face_box) != 4
                or self._latest_safe_face_size[0] <= 0
                or self._latest_safe_face_size[1] <= 0
                or self._latest_safe_rgb_time <= 0.0
                or self._latest_safe_depth_time <= 0.0
            ):
                self._abort_for_scene_change(
                    "已发送 valid=1 测距帧，但无法绑定其人脸场景快照"
                )
                return False
            snapshot = SafeFaceSnapshot(
                box=face_box,
                frame_size=(
                    int(self._latest_safe_face_size[0]),
                    int(self._latest_safe_face_size[1]),
                ),
                rgb_sample_time=float(self._latest_safe_rgb_time),
                depth_sample_time=float(self._latest_safe_depth_time),
            )
            self._safe_face_snapshots[int(seq) & 0xFF] = snapshot
            while len(self._safe_face_snapshots) > 32:
                self._safe_face_snapshots.pop(next(iter(self._safe_face_snapshots)))
            if not self.args.demo and not self._provision_safe_scene_snapshot(snapshot):
                if not self._scene_abort_triggered:
                    self._abort_for_scene_change("无法建立测距发送时的人脸场景基线")
                return False
            self._last_sent_safe_sample_time = self._latest_safe_depth_time
        if not ok:
            self._set_status("SAFE_DISTANCE 发送失败：{}".format(message))
        self._set_serial_safe_distance_text(distance_mm, valid, source_text)
        self._last_serial_safe_distance = now
        return bool(ok and valid)

    def _serial_safe_distance_payload(self, now: Optional[float] = None) -> Tuple[int, bool, str]:
        if self.serial_safe_distance_source == "fixed":
            if not (self.args.demo or self.args.protocol_debug_controls):
                return 0, False, "正式模式禁止固定距离"
            return self.serial_safe_distance_mm, self.serial_safe_distance_valid, "固定值"

        if now is None:
            now = time.monotonic()
        depth_age = now - self._latest_safe_depth_time if self._latest_safe_depth_time > 0 else float("inf")
        rgb_age = now - self._latest_safe_rgb_time if self._latest_safe_rgb_time > 0 else float("inf")
        if (
            self._latest_safe_measurement_valid
            and depth_age <= self.serial_safe_distance_max_age
            and rgb_age <= self.serial_safe_distance_max_age
            and self._latest_safe_depth_mm >= 0
            and abs(self._latest_safe_rgb_time - self._latest_rgbd_detection_time) <= 1e-6
            and {"arm", "face"}.issubset(self._latest_rgbd_detections)
        ):
            if (
                self._latest_safe_depth_time <= self._measure_position_ready_time
                or self._latest_safe_rgb_time <= self._measure_position_ready_time
            ):
                return 0, False, "等待测距位到达后的新RGB-D采样"
            distance_mm = max(0, min(65535, int(round(self._latest_safe_depth_mm))))
            valid_percent = int(round(self._latest_safe_depth_valid_ratio * 100.0))
            return distance_mm, True, "RGB-D净距 {}%".format(valid_percent)
        return 0, False, "RGB-D净距无效"

    def _has_fresh_depth_safe_distance(self) -> bool:
        if self.serial_safe_distance_source != "depth":
            return True
        return self._serial_safe_distance_payload(time.monotonic())[1]

    def _set_serial_safe_distance_text(self, distance_mm: int, valid: bool, source_text: str) -> None:
        label = self.__dict__.get("binocular_safe_distance_value")
        if label is None:
            return
        if valid:
            label.setText("{} mm / {}".format(distance_mm, source_text))
        else:
            label.setText("无效 / {}".format(source_text))

    def _set_stale_safe_distance_text(self, reason: str) -> None:
        label = self.__dict__.get("binocular_safe_distance_value")
        if label is None:
            return
        if self._latest_safe_depth_time > 0.0:
            label.setText(
                "{} mm / 已过期：{}（未发送）".format(
                    int(round(self._latest_safe_depth_mm)),
                    reason,
                )
            )
        else:
            label.setText("无效 / {}".format(reason))

    def _is_ra6_status(self, status: object, func: int, value: int) -> bool:
        return int(getattr(status, "func", -1)) == func and int(getattr(status, "value", -1)) == value

    def _is_ra6_error(self, status: object) -> bool:
        protocol = str(getattr(status, "protocol", "legacy"))
        func = int(getattr(status, "func", -1))
        value = int(getattr(status, "value", -1))
        if protocol in ("unified", "unified_error"):
            return func == RA6_ERROR_FUNC or int(getattr(status, "error_code", 0)) != 0
        return func == RA6_ERROR_FUNC and value == RA6_VALUE_ON

    def _start_detection_after_ready(self) -> None:
        if not self._waiting_for_arm_ready:
            return
        self._waiting_for_arm_ready = False
        self._detection_running = True
        self._ready_after_action = ""
        self._arm_ready_wait_started = None
        self._switch_page(PAGE_END_CAMERA)
        if self.worker is not None:
            self.worker.start_detection()
        self.workflow_state.setText("连续识别")
        self.ready_wait_value.setText("已就绪")
        self._set_status("机械臂已到位，开始连续识别")
        self._log_event("机械臂 READY，开始连续识别")
        self._refresh_controls()

    def _start_workflow_after_ready(self) -> None:
        if not self._waiting_for_arm_ready:
            return
        self._waiting_for_arm_ready = False
        self._detection_running = False
        self._preposition_ready = True
        self._ready_after_action = ""
        self._arm_ready_wait_started = None
        self.workflow_state.setText("预定位已到位")
        self.ready_wait_value.setText("可采图")
        self._set_status("机械臂已到预定位，请点击采集人脸图")
        self._log_event("机械臂 READY，等待采集三视图")
        self._switch_page(PAGE_BINOCULAR)
        self._refresh_controls()

    def _simulate_workflow_ready(self) -> None:
        self._waiting_for_safe_distance = False
        self._waiting_for_arm_ready = True
        self._ready_after_action = "workflow_start"
        self._arm_ready_wait_started = time.monotonic()
        self._log_event("调试模拟：RA6M5 预定位 READY")
        self._start_workflow_after_ready()

    def _start_reference_capture_after_ready(self) -> None:
        self._waiting_for_arm_ready = False
        self._detection_running = False
        self._ready_after_action = ""
        self._reference_capturing = True
        self._arm_ready_wait_started = None
        self._switch_page(PAGE_END_CAMERA)
        if self.worker is not None:
            self.worker.request_reference_capture()
        self.workflow_state.setText("采集基准图")
        self.ready_wait_value.setText("--")
        self.reference_state.setText("采集中")
        self._set_status("开始采集无黑痣基准图，请保持画面稳定")
        self._log_event("开始采集无黑痣基准图")
        self._refresh_controls()

    def _update_arm_ready_wait(self) -> None:
        if not self._waiting_for_arm_ready or self._arm_ready_wait_started is None:
            return

        elapsed = time.monotonic() - self._arm_ready_wait_started
        self.ready_wait_value.setText(f"{elapsed:.1f} s")
        if self.arm_ready_timeout_ms <= 0:
            return
        if elapsed * 1000 < self.arm_ready_timeout_ms:
            return

        if self._ready_after_action == "reference":
            action_text = "采集基准图"
        elif self._ready_after_action == "measure_position":
            action_text = "到达公共测距位"
        elif self._ready_after_action == "target_start":
            action_text = "启动定靶"
        elif self._ready_after_action == "finish_return_home":
            action_text = "完成返回 HOME"
        elif self._ready_after_action == "abort_hold":
            action_text = "安全停止"
        else:
            action_text = "识别"
        reason = f"等待 {action_text} 状态超时 {self.arm_ready_timeout_ms / 1000:.1f} s"
        if self._ready_after_action == "abort_hold":
            self._stop_automatic_progress(
                reason + "；已停止串口心跳",
                restart_blocked=True,
            )
            self.arm.close()
            self.serial_state.setText("串口已关闭，等待 RA6M5 心跳超时保护")
        else:
            self._cancel_workflow(reason=reason)
        QMessageBox.warning(self, "协议状态超时", reason)

    def _abort_detection(self, reason: str, send_stop: bool) -> None:
        if self.worker is not None:
            self.worker.stop_detection()
            self.worker.cancel_reference_capture()
            self.worker.cancel_safety_capture()
        self._waiting_for_arm_ready = False
        self._detection_running = False
        self._reference_capturing = False
        self._safety_capturing = False
        self._capture_sequence_active = False
        self._capture_sequence_manual = False
        self._preposition_ready = False
        self._capture_sequence_points = []
        self._capture_sequence_index = 0
        self._capture_pending_point = 0
        self._ready_after_action = ""
        self._arm_ready_wait_started = None
        self.workflow_state.setText("已停止")
        self.ready_wait_value.setText("--")
        if send_stop and self.arm.connected:
            ok, message = self.arm.stop_target_state_machine()
            self._log_event(message)
            if not ok:
                reason = f"{reason}；{message}"
        self._set_status(reason)
        self._log_event(reason)
        self._refresh_controls()

    def _refresh_controls(self) -> None:
        if not hasattr(self, "start_btn"):
            return

        busy = (
            self.arm.control_busy
            or
            self._waiting_for_arm_ready
            or self._waiting_for_safe_distance
            or self._reference_capturing
            or self._safety_capturing
            or self._capture_sequence_active
        )
        target_page_locked = (
            self._selected_view_ready
            or self._target_ctrl_on
            or self._target_ready
            or self._output_on
            or (self._waiting_for_arm_ready and self._ready_after_action == "target_start")
        )
        for page_index, button in enumerate(getattr(self, "nav_buttons", [])):
            button.setEnabled(
                not busy
                and (not target_page_locked or page_index == PAGE_END_CAMERA)
            )
        has_target = self._current_arm_target() is not None
        arm_action_available = (
            self.arm.connected
            and not self._waiting_for_arm_ready
            and not self._waiting_for_safe_distance
            and not self._reference_capturing
            and not self._safety_capturing
            and not self._capture_sequence_active
            and not self._safety_blocking
        )

        can_capture_faces = (
            self._distance_safe_latched
            and not self._capture_complete
            and not busy
            and not self._safety_blocking
        )
        manual_capture_available = (
            (self.args.demo or self.args.protocol_debug_controls)
            and
            self.pages.currentIndex() == PAGE_END_CAMERA
            and not busy
            and not self._detection_running
            and not self._safety_blocking
        )
        can_store_capture_frame = (
            self._capture_sequence_active
            and self._capture_pending_point in (1, 2, 3)
            and not self._safety_blocking
        )

        self.capture_reference_btn.setEnabled(not busy and not self._detection_running and not self._workflow_active)
        self.capture_safety_btn.setEnabled(not busy and not self._detection_running and not self._workflow_active)
        self.start_btn.setEnabled(
            not busy
            and not self._workflow_active
            and not self._restart_blocked
            and not self._target_ready
            and not self._detection_running
        )
        finish_enabled = (
            self.arm.connected
            and not busy
            and not self.arm.control_busy
            and not self._restart_blocked
        ) or (
            self.args.demo
            and self._workflow_active
            and not busy
            and not self._fault_hold
        )
        self.finish_btn.setEnabled(finish_enabled)
        self.end_finish_btn.setEnabled(finish_enabled)
        if can_store_capture_frame:
            capture_text = "上传{}视图".format(self._point_short_name(self._capture_pending_point))
        elif self._capture_sequence_active:
            capture_text = "等待拍摄点"
        else:
            capture_text = "采集人脸图"
        for button in (
            self.capture_faces_btn,
            self.binocular_capture_faces_btn,
            self.end_capture_faces_btn,
        ):
            button.setText(capture_text)
            button.setEnabled(can_capture_faces or manual_capture_available or can_store_capture_frame)
        self.once_btn.setEnabled(
            bool(self.args.protocol_debug_controls)
            and not self._workflow_active
            and not self._detection_running
            and not self._target_ctrl_on
            and not self._waiting_for_arm_ready
            and not self._waiting_for_safe_distance
            and not self._reference_capturing
            and not self._safety_capturing
            and not self._capture_sequence_active
        )
        abort_waiting = self._ready_after_action == "abort_hold" or self.arm.abort_pending
        cancel_context = (
            self._workflow_active
            or busy
            or self._selected_view_ready
            or self._target_ctrl_on
            or self._target_ready
            or self._align_done
            or self._output_on
            or self._detection_running
        )
        cancel_enabled = not abort_waiting and cancel_context
        self.stop_btn.setEnabled(cancel_enabled)
        self.end_stop_btn.setEnabled(cancel_enabled)
        self.preview_btn.setEnabled(
            bool(self.args.protocol_debug_controls)
            and arm_action_available
            and self._target_ready
            and has_target
        )
        self.fire_btn.setEnabled(
            self._workflow_active
            and not self._fault_hold
            and self._distance_safe_latched
            and self._selected_view_ready
            and not self._target_ctrl_on
            and not self._target_ready
            and not busy
            and (self.arm.connected or self.args.demo)
        )
        capture_available = (
            bool(self.args.protocol_debug_controls)
            and self.arm.connected
            and not busy
            and not self._safety_blocking
        )
        capture_home_available = self.arm.connected and not busy
        if hasattr(self, "capture_left_btn"):
            for button in (
                self.capture_left_btn,
                self.capture_center_btn,
                self.capture_right_btn,
                self.select_left_btn,
                self.select_center_btn,
                self.select_right_btn,
            ):
                button.setEnabled(capture_available)
            self.capture_home_btn.setEnabled(capture_home_available)
        if getattr(self, "face_select_buttons", None):
            triptych_analysis_ready = (
                self.triptych_worker is None or not self.triptych_worker.isRunning()
            )
            scene_select_ready = (
                self.args.demo
                or not self._workflow_active
                or self._scene_ready_for_view_selection()[0]
            )
            for point_id, button in self.face_select_buttons.items():
                button.setEnabled(
                    self._capture_complete
                    and triptych_analysis_ready
                    and self._distance_safe_latched
                    and scene_select_ready
                    and not busy
                    and not self._safety_blocking
                    and not self._target_ctrl_on
                    and not self._target_ready
                    and not self._output_on
                )

    def _update_clock(self) -> None:
        self.clock_label.setText(QDateTime.currentDateTime().toString("yyyy-MM-dd HH:mm:ss"))


STYLE = """
#root {
    background: #080d12;
    color: #e9f1f8;
    font-family: "Noto Sans CJK SC", "Microsoft YaHei", "Arial";
}
#sidebar {
    background: #0d1520;
    border-right: 1px solid #1c2937;
}
#content {
    background: #0a1017;
}
#pages, #pageSurface {
    background: #0a1017;
    border: 0;
}
#brand {
    color: #f3f8ff;
    font-size: 16px;
    font-weight: 800;
}
#brandSubtitle {
    color: #6ee7c8;
    font-size: 11px;
}
#navButton {
    background: transparent;
    color: #9fb2c7;
    border: 0;
    border-radius: 6px;
    text-align: left;
    padding-left: 10px;
    font-size: 12px;
    font-weight: 700;
}
#navButton:hover {
    background: #142132;
    color: #dce8f4;
}
#navButton:checked {
    background: #17332f;
    color: #72f0cf;
    border-left: 3px solid #20c997;
}
#navExitButton {
    background: #161f2b;
    color: #d7e2ee;
    border: 1px solid #25364a;
    border-radius: 6px;
    font-size: 13px;
    font-weight: 700;
}
#header {
    background: transparent;
}
#title {
    color: #f1f7ff;
    font-size: 19px;
    font-weight: 700;
}
#subtitle {
    font-size: 11px;
    color: #8ea1b5;
}
#statusPill {
    background: #10221f;
    color: #76f0d0;
    border: 1px solid #1f685c;
    border-radius: 6px;
    padding: 6px 8px;
    font-size: 11px;
}
#clock {
    color: #a8b8c9;
    font-size: 11px;
    padding-left: 4px;
}
#metricCard {
    background: #101923;
    border: 1px solid #213044;
    border-radius: 8px;
}
#metricName {
    color: #8fa1b6;
    font-size: 11px;
}
#metricValue {
    color: #f2f8ff;
    font-size: 20px;
    font-weight: 800;
}
#metricCaption {
    color: #6f8399;
    font-size: 11px;
}
#video {
    background: #05080c;
    color: #dce7f2;
    border: 1px solid #243348;
    border-radius: 8px;
    font-size: 20px;
}
#miniVideo {
    background: #070b10;
    color: #cfdbe8;
    border: 1px solid #243348;
    border-radius: 6px;
    font-size: 13px;
}
QGroupBox {
    background: #0f1721;
    color: #e8f2fb;
    border: 1px solid #223247;
    border-radius: 8px;
    margin-top: 8px;
    padding: 8px;
    font-size: 13px;
    font-weight: 700;
}
QGroupBox::title {
    subcontrol-origin: margin;
    left: 12px;
    padding: 0 6px;
    color: #dce8f4;
}
#fieldName {
    color: #8fa2b7;
    font-size: 11px;
}
#fieldValue {
    color: #e8f2fb;
    font-size: 12px;
    font-weight: 600;
}
#history {
    background: #081018;
    color: #d8e6f3;
    border: 1px solid #223247;
    border-radius: 6px;
    font-size: 11px;
    padding: 3px;
}
#history::item {
    padding: 4px;
}
#history::item:selected {
    background: #16342f;
    color: #7ff0d1;
}
QScrollArea {
    border: 0;
    background: #0a1017;
}
QScrollArea > QWidget > QWidget {
    background: #0a1017;
}
QScrollArea QWidget#qt_scrollarea_viewport {
    background: #0a1017;
}
QScrollBar:vertical {
    background: #0a1017;
    width: 8px;
    margin: 0;
}
QScrollBar::handle:vertical {
    background: #24384d;
    border-radius: 4px;
    min-height: 28px;
}
QScrollBar::add-line:vertical,
QScrollBar::sub-line:vertical {
    height: 0;
    border: 0;
    background: transparent;
}
QScrollBar:horizontal {
    background: #0a1017;
    height: 8px;
    margin: 0;
}
QScrollBar::handle:horizontal {
    background: #24384d;
    border-radius: 4px;
    min-width: 28px;
}
QScrollBar::add-line:horizontal,
QScrollBar::sub-line:horizontal {
    width: 0;
    border: 0;
    background: transparent;
}
#stepCard {
    background: #111c28;
    border: 1px solid #24384e;
    border-radius: 8px;
}
#stepNumber {
    color: #20c997;
    font-size: 18px;
    font-weight: 900;
}
#stepTitle {
    color: #eff7ff;
    font-size: 15px;
    font-weight: 800;
}
#stepCaption {
    color: #8fa2b7;
    font-size: 11px;
}
#captureCard {
    background: #101923;
    border: 1px solid #23364a;
    border-radius: 8px;
}
#captureTitle {
    color: #f0f7ff;
    font-size: 15px;
    font-weight: 800;
}
#captureState {
    color: #8fa2b7;
    font-size: 11px;
    font-weight: 700;
}
#facePreview {
    background: #060a0f;
    color: #72869c;
    border: 1px solid #243348;
    border-radius: 6px;
    font-size: 16px;
}
QPushButton {
    background: #1b5f90;
    color: white;
    border: 0;
    border-radius: 7px;
    font-size: 13px;
    font-weight: 700;
}
QPushButton:hover {
    background: #2375ad;
}
QPushButton:pressed {
    background: #154b72;
}
QPushButton:disabled {
    background: #1a2430;
    color: #536679;
}
QPushButton#primaryAction {
    background: #148f77;
    color: #f4fffb;
}
QPushButton#primaryAction:hover {
    background: #18aa8d;
}
QPushButton#danger {
    background: #b83b48;
}
QPushButton#danger:hover {
    background: #d34a58;
}
QPushButton#secondary {
    background: #263548;
    color: #dce8f4;
    border: 1px solid #38506a;
}
QPushButton#secondary:hover {
    background: #31445c;
}
QPushButton[selected="true"] {
    background: #d9a63a;
    color: #101923;
}
QToolTip {
    background: #101923;
    color: #e8f2fb;
    border: 1px solid #2c425a;
}
"""


def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="机械臂点痣装置 Qt 界面")
    parser.add_argument("--camera", default="0", help="摄像头编号、/dev/videoX、csi 或 GStreamer pipeline")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--camera-brightness", type=float, default=0.0, help="末端摄像头亮度；识别优先基线为0")
    parser.add_argument("--camera-gain", type=float, default=64.0, help="末端摄像头增益；识别优先基线为64")
    parser.add_argument("--camera-backlight-compensation", type=float, default=0.0, help="末端摄像头背光补偿；识别优先基线为0")
    parser.add_argument("--face-capture-dir", default=str(DEFAULT_FACE_CAPTURE_DIR), help="三视图末端相机原始 PNG 保存根目录")
    parser.add_argument("--ui-fps", type=float, default=20.0, help="Qt 画面刷新上限；末端检测实际帧率还受算法耗时限制")
    parser.add_argument("--flip", choices=["none", "0", "1", "-1"], default="none")
    parser.add_argument(
        "--rotate",
        choices=["none", "90cw", "90ccw", "180"],
        default="none",
        help="末端视觉主摄像头画面旋转；默认 none，保持摄像头原始方向",
    )
    parser.add_argument("--camera-layout", choices=["mono", "side-by-side", "astra-rgbd"], default="mono", help="摄像头画面布局；astra-rgbd 表示 Astra Pro Plus 的 RGB + Depth 架构")
    parser.add_argument("--stereo", action="store_true", help="快捷启用普通双目左右拼接画面，等同于 --camera-layout side-by-side")
    parser.add_argument("--rgbd", action="store_true", help="快捷启用 Astra Pro Plus RGB-D 深度模式，等同于 --camera-layout astra-rgbd")
    parser.add_argument("--stereo-primary", choices=["left", "right"], default="left", help="双目左右拼接时用于识别和机械臂误差计算的主视图")
    parser.add_argument("--astra-rgb-enabled", type=int, choices=[0, 1], default=1, help="启用 Astra RGB 预览；默认启用")
    parser.add_argument("--astra-rgb-camera", default="/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera_SN0001-video-index0", help="Astra RGB 摄像头路径")
    parser.add_argument("--astra-rgb-width", type=int, default=640)
    parser.add_argument("--astra-rgb-height", type=int, default=480)
    parser.add_argument("--astra-rgb-fps", type=int, default=30)
    parser.add_argument("--astra-rgb-flip", choices=["none", "0", "1", "-1"], default="none")
    parser.add_argument(
        "--astra-rgb-calibration",
        default=str(DEFAULT_ASTRA_RGB_CALIBRATION),
        help="实际V4L Astra RGB流的OpenCV NPZ标定文件",
    )
    parser.add_argument("--astra-always-on", type=int, choices=[0, 1], default=1, help="Astra RGB-D 随 Qt 启动并跨页面持续运行；设为0恢复按页面启停")
    parser.add_argument("--astra-yolo-enabled", type=int, choices=[0, 1], default=1, help="在双目安全页的 Astra RGB 图上启用 YOLO arm_tip / face 识别")
    parser.add_argument("--astra-yolo-model", default=str(DEFAULT_ASTRA_YOLO_MODEL), help="Astra RGB YOLO 模型路径")
    parser.add_argument("--astra-yolo-conf", type=float, default=0.35, help="Astra RGB YOLO 置信度阈值")
    parser.add_argument("--astra-yolo-iou", type=float, default=0.45, help="Astra RGB YOLO NMS IoU 阈值")
    parser.add_argument("--astra-yolo-imgsz", type=int, default=640, help="Astra RGB YOLO 推理尺寸")
    parser.add_argument("--astra-yolo-device", default="", help="Astra RGB YOLO 推理设备；留空自动选择，CUDA 可尝试 0")
    parser.add_argument("--astra-yolo-arm-name", default="arm_tip", help="YOLO 中机械臂末端类别名")
    parser.add_argument("--astra-yolo-face-name", default="face", help="YOLO 中人脸类别名")
    parser.add_argument("--astra-yolo-interval", type=float, default=0.15, help="Astra RGB YOLO 推理间隔秒数；正式测距建议不超过0.2秒")
    parser.add_argument("--rgbd-arm-bbox-scale", type=float, default=1.0, help="arm_tip 检测框映射到 Depth ROI 的缩放")
    parser.add_argument("--rgbd-face-bbox-scale", type=float, default=1.0, help="face 检测框映射到 Depth ROI 的缩放")
    parser.add_argument("--rgbd-arm-depth-offset-x", type=int, default=0, help="arm_tip Depth ROI 水平标定偏移（Depth像素）")
    parser.add_argument("--rgbd-arm-depth-offset-y", type=int, default=0, help="arm_tip Depth ROI 垂直标定偏移（Depth像素）")
    parser.add_argument("--rgbd-face-depth-offset-x", type=int, default=0, help="face Depth ROI 水平标定偏移（Depth像素）")
    parser.add_argument("--rgbd-face-depth-offset-y", type=int, default=0, help="face Depth ROI 垂直标定偏移（Depth像素）")
    parser.add_argument("--rgbd-min-depth-mm", type=float, default=20.0, help="RGB-D 点云最小有效深度")
    parser.add_argument("--rgbd-max-depth-mm", type=float, default=10000.0, help="RGB-D 点云最大有效深度")
    parser.add_argument("--rgbd-max-points", type=int, default=600, help="每个检测框参与三维净距计算的最大点数")
    parser.add_argument("--rgbd-max-sample-cover-radius-mm", type=float, default=30.0, help="无SciPy抽样回退允许的最大点云覆盖半径；超过则整帧无效")
    parser.add_argument("--rgbd-filter-measurement-noise-mm", type=float, default=20.0, help="三维净距滤波测量噪声")
    parser.add_argument("--rgbd-filter-process-noise-mm", type=float, default=6.0, help="三维净距滤波过程噪声")
    parser.add_argument("--rgbd-filter-jump-reset-mm", type=float, default=80.0, help="净距突变超过该值时重置滤波")
    parser.add_argument("--rgbd-sync-max-delta", type=float, default=0.15, help="RGB检测帧与Depth帧允许的最大采样时差")
    parser.add_argument("--rgbd-distance-log", action="store_true", help="在终端打印带节流的RGB-D净距、有效性及失效原因诊断")
    parser.add_argument("--rgbd-distance-log-interval", type=float, default=0.5, help="RGB-D净距有效状态的终端日志间隔秒数；状态或失效原因变化会立即打印")
    parser.add_argument("--rgbd-continuous-stop-mm", type=float, default=0.0, help="距离锁存后继续计算arm-face三维净距；小于该毫米阈值时发送ABORT_HOLD，0=关闭")
    parser.add_argument("--rgbd-continuous-guard-interval", type=float, default=0.20, help="距离锁存后RGB-D连续安全门的计算间隔秒数，自动限制为0.10~1.00")
    parser.add_argument("--rgbd-allow-approximate-mapping", action="store_true", help="调试：缺少SDK RGB-Depth外参时允许按分辨率/偏移近似映射；正式流程不要启用")
    parser.add_argument("--rgbd-hardware-pair-verified", action="store_true", help="真机验收后显式确认V4L RGB与SDK Depth来自同一Astra，且畸变、外参与最坏时序已验证；未确认时正式SAFE_DISTANCE无效且不发送")
    parser.add_argument("--scene-center-shift-ratio", type=float, default=0.08, help="距离锁存后人脸中心允许的最大归一化位移，超出即 ABORT_HOLD")
    parser.add_argument("--scene-scale-ratio", type=float, default=1.25, help="距离锁存后人脸宽高允许的最大对称倍率，超出即 ABORT_HOLD")
    parser.add_argument("--scene-missing-timeout", type=float, default=0.8, help="距离锁存后人脸/场景监测连续丢失多少秒即 ABORT_HOLD")
    parser.add_argument("--astra-depth-enabled", type=int, choices=[0, 1], default=1, help="启用 Astra Depth 预览；默认启用")
    parser.add_argument("--depth-timeout-ms", type=int, default=1000, help="Astra Pro Plus 深度帧等待超时")
    parser.add_argument("--depth-rotate-180", action="store_true", help="深度图旋转 180 度显示")
    parser.add_argument(
        "--astra-depth-distortion-mode",
        choices=["auto", "raw", "rectified"],
        default="auto",
        help="Depth帧几何；auto仅在SDK可查询硬件去畸变状态时放行，raw/rectified只用于完成真机验证后的显式配置",
    )
    parser.add_argument(
        "--detector-mode",
        choices=["facial_candidate", "auto", "spot", "reference"],
        default="facial_candidate",
        help="facial_candidate=无需基准图的人脸候选检测；其余模式仅用于实验和回归",
    )
    parser.add_argument("--facial-face-model", default=str(DEFAULT_FACIAL_FACE_MODEL), help="人脸 ROI YOLO 模型；失败时自动回退 OpenCV 人脸检测")
    parser.add_argument("--facial-face-conf", type=float, default=0.35, help="人脸 ROI 置信度阈值")
    parser.add_argument("--facial-face-imgsz", type=int, default=416, help="Haar/YOLO 人脸定位尺寸；末端大人脸默认416兼顾速度")
    parser.add_argument("--facial-face-device", default="", help="人脸 ROI 推理设备；留空自动选择，Orin 可设置 0")
    parser.add_argument("--facial-face-refresh-frames", type=int, default=6, help="候选有效时每隔多少个末端检测帧重新定位人脸框；候选或人脸丢失后下一帧立即刷新")
    parser.add_argument("--facial-max-candidates", type=int, default=12, help="每个视图最多显示的候选暗斑数量")
    parser.add_argument("--facial-stable-frames", type=int, default=4, help="末端视频流连续稳定出现多少帧后才确认候选")
    parser.add_argument("--facial-stable-seconds", type=float, default=0.35, help="末端候选连续稳定的最短时间；与稳定帧门同时满足后才确认")
    parser.add_argument("--facial-stable-radius", type=float, default=6.0, help="多帧候选中心允许的最大漂移像素")
    parser.add_argument("--facial-enhanced-filters", type=int, choices=[0, 1], default=1, help="启用眼眉/底边等增强候选过滤；0=回退到较宽松候选")
    parser.add_argument("--facial-triptych-top-candidate-only", type=int, choices=[0, 1], default=1, help="三视图每张图只显示最可信候选；0=显示全部候选，便于医生选择 #1/#2/#3")
    parser.add_argument("--baseline-face-enabled", type=int, choices=[0, 1], default=1, help="三视图识别时启用无黑痣基准图库辅助排序；0=关闭")
    parser.add_argument("--baseline-face-dir", default="/home/jetson/jixiebi/calibration/baseline_face/raw", help="无黑痣基准图库 raw 根目录，包含 front/left/right")
    parser.add_argument("--reference", default="", help="无黑痣基准图路径，默认使用 jetson_nano_vision/reference/clean_face.png")
    parser.add_argument("--reference-frames", type=int, default=30, help="Qt 界面采集基准图时使用的帧数")
    parser.add_argument("--reference-warmup", type=int, default=30, help="Qt 界面采集基准图前丢弃的预热帧数")
    parser.add_argument("--ref-min-darkening", type=int, default=8, help="基准差分检测的最小变暗阈值")
    parser.add_argument("--ref-min-mean-darkening", type=float, default=8.0, help="基准差分检测的平均变暗阈值")
    parser.add_argument("--ref-min-confidence", type=float, default=0.45, help="基准差分检测的最小置信度")
    parser.add_argument("--ref-mask-erode", type=int, default=5, help="基准检测皮肤区域收缩尺寸；越小保留区域越多")
    parser.add_argument("--ref-max-translation-px", type=float, default=80.0, help="基准对齐允许的最大平移像素")
    parser.add_argument("--ref-max-rotation-deg", type=float, default=15.0, help="基准对齐允许的最大旋转角度")
    parser.add_argument("--ref-min-alignment-score", type=float, default=0.30, help="基准对齐最低 ECC 分数")
    parser.add_argument("--spot-min-area", type=int, default=8, help="普通黑点检测最小面积；调大可减少小噪点")
    parser.add_argument("--spot-max-area", type=int, default=220, help="普通黑点检测最大面积；调小可排除眼睛/大阴影")
    parser.add_argument("--spot-min-circularity", type=float, default=0.62, help="普通黑点检测最小圆度")
    parser.add_argument("--spot-min-solidity", type=float, default=0.85, help="普通黑点检测最小实心度")
    parser.add_argument("--spot-max-aspect-ratio", type=float, default=2.40, help="普通黑点检测最大长宽比；2.4可容纳软画面中的水滴形黑痣")
    parser.add_argument("--spot-blackhat-ksize", type=int, default=19, help="普通黑点检测 blackhat 核大小")
    parser.add_argument("--spot-min-contrast", type=int, default=28, help="普通黑点检测最小局部对比度；调大可减少误检")
    parser.add_argument("--spot-skin-dilate", type=int, default=11, help="普通黑点检测皮肤区域收缩尺寸；调大可减少边缘误检")
    parser.add_argument("--spot-roi-x-min-ratio", type=float, default=0.0, help="普通黑点检测区域左边界比例")
    parser.add_argument("--spot-roi-x-max-ratio", type=float, default=1.0, help="普通黑点检测区域右边界比例")
    parser.add_argument("--spot-roi-y-min-ratio", type=float, default=0.0, help="普通黑点检测区域上边界比例")
    parser.add_argument("--spot-roi-y-max-ratio", type=float, default=1.0, help="普通黑点检测区域下边界比例")
    parser.add_argument("--safety-enabled", type=int, choices=[0, 1], default=0, help="启用近距离安全警告；1=启用，0=关闭；默认先关闭")
    parser.add_argument("--safety-image", default="", help="安全距离图路径，默认使用 jetson_nano_vision/reference/safety_distance.png")
    parser.add_argument("--safety-reference", default="", help="安全距离标定文件路径，默认使用 jetson_nano_vision/reference/safety_reference.json")
    parser.add_argument("--safety-warn-face-ratio", type=float, default=0.48, help="未标定安全距离时的警告兜底阈值")
    parser.add_argument("--safety-danger-face-ratio", type=float, default=0.62, help="未标定安全距离时的危险兜底阈值")
    parser.add_argument("--safety-warn-scale", type=float, default=1.10, help="安全距离图测量值的警告倍率；当前测量值低于该倍率开始警告")
    parser.add_argument("--safety-danger-scale", type=float, default=1.00, help="安全距离图测量值的危险倍率；当前测量值低于该倍率判定靠近危险")
    parser.add_argument("--demo", action="store_true", help="使用模拟画面，不打开真实摄像头")
    parser.add_argument("--fullscreen", action="store_true", help="启动后全屏显示")
    parser.add_argument("--serial-port", default="", help="RA6M5 串口号，空则不连接")
    parser.add_argument("--baudrate", type=int, default=115200, help="RA6M5 串口波特率")
    parser.add_argument("--serial-protocol", choices=["unified", "legacy"], default="unified", help="RA6M5 串口协议；正式演示使用 unified")
    parser.add_argument(
        "--continuous-safe-distance",
        action="store_true",
        help="已停用的旧提案参数；当前正式协议不定义 action=0x04，传入时程序将拒绝启动",
    )
    parser.add_argument("--protocol-debug-controls", action="store_true", help="显示手动 GOTO/HOME/单帧视觉等协议调试控件；比赛流程不要启用")
    parser.add_argument("--serial-heartbeat-interval", type=float, default=0.15, help="新协议 HEARTBEAT 周期，单位秒")
    parser.add_argument("--serial-safe-distance-interval", type=float, default=0.15, help="新协议 SAFE_DISTANCE 周期，单位秒")
    parser.add_argument("--serial-vision-interval", type=float, default=0.15, help="新协议 VISION_ERROR 周期，自动限制在100~200ms")
    parser.add_argument("--serial-safe-distance-source", choices=["depth", "fixed"], default="depth", help="SAFE_DISTANCE 来源；depth=arm_tip 与 face 的 RGB-D 三维净距，fixed=仅演示/协议调试")
    parser.add_argument("--serial-safe-distance-max-age", type=float, default=0.5, help="YOLO/三维净距超过该秒数未更新则标记过期并停止发送 SAFE_DISTANCE")
    parser.add_argument("--serial-safe-distance-min-valid-ratio", type=float, default=0.20, help="检测框及中心区域有效 Depth 比例低于该值时不发送 SAFE_DISTANCE")
    parser.add_argument("--serial-safe-distance-mm", type=int, default=0, help="固定 SAFE_DISTANCE 值，仅 --serial-safe-distance-source fixed 时使用")
    parser.add_argument("--serial-safe-distance-valid", type=int, choices=[0, 1], default=0, help="固定 SAFE_DISTANCE 是否有效，仅 --serial-safe-distance-source fixed 时使用")
    parser.add_argument("--vision-target-max-age", type=float, default=0.5, help="末端目标结果超过该秒数即发送 VISION_ERROR valid=0")
    parser.add_argument("--target-lock-max-step-ratio", type=float, default=0.10, help="同一定靶目标相邻帧允许的最大归一化位移，超出后只发 valid=0，禁止换痣")
    parser.add_argument("--target-lock-max-radius-ratio", type=float, default=1.5, help="同一定靶目标相邻帧允许的最大半径倍率，超出后禁止续跟")
    parser.add_argument("--target-lock-max-step-radius-factor", type=float, default=1.0, help="相邻帧续跟位移上限相对痣半径的倍率")
    parser.add_argument("--target-lock-reacquire-radius-factor", type=float, default=0.5, help="丢失后重捕位移门相对正常续跟门的倍率")
    parser.add_argument("--target-lock-ambiguity-radius-factor", type=float, default=0.75, help="两个近邻候选距离差小于该痣半径倍率时按歧义处理")
    parser.add_argument("--target-lock-stable-frames", type=int, default=2, help="首次锁定或丢失重捕必须连续稳定的帧数")
    parser.add_argument("--target-lock-rebind-timeout", type=float, default=5.0, help="上游跟踪ID因漏检重建后允许保守重绑定的最长秒数；超时后保持 valid=0")
    parser.add_argument("--laser-offset-mm", type=float, default=27.81, help="摄像头到激光点的 X 向物理偏置，单位 mm")
    parser.add_argument("--mm-per-pixel-x", type=float, default=0.0, help="X 方向每像素对应的毫米数，用于把激光偏置换算成像素")
    parser.add_argument("--laser-offset-x-px", type=int, default=None, help="直接指定摄像头中心到激光中心的 X 向像素偏置；设置后优先于 mm 换算")
    parser.add_argument("--laser-offset-y-px", type=int, default=None, help="实测摄像头中心到激光瞄准参考点的 Y 向像素偏置，向下为正；正式定靶必须与 X 一起显式配置")
    parser.add_argument("--arm-ready-timeout-ms", type=int, default=30000, help="开始识别后等待 RA6M5 READY 的超时时间；0 表示不超时")
    parser.add_argument("--window-width", type=int, default=1024, help="普通窗口模式下的初始宽度")
    parser.add_argument("--window-height", type=int, default=600, help="普通窗口模式下的初始高度")
    args = parser.parse_args(argv)
    if args.stereo:
        args.camera_layout = "side-by-side"
    if args.rgbd:
        args.camera_layout = "astra-rgbd"
    if args.demo:
        args.astra_rgb_enabled = 0
        args.astra_depth_enabled = 0
    return args


def _check_display_available() -> bool:
    if not sys.platform.startswith("linux"):
        return True
    if os.environ.get("QT_QPA_PLATFORM") == "offscreen":
        return True
    return bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))


def _print_display_help() -> None:
    print(
        "Qt 界面需要图形桌面显示器，但当前终端没有 DISPLAY / WAYLAND_DISPLAY。\n"
        "如果你是在 Jetson 桌面上操作，请打开本机终端后运行本程序。\n"
        "如果你是 SSH 登录 Jetson 且 Jetson 已经接显示器并登录桌面，可以先执行：\n"
        "  export DISPLAY=:0\n"
        "然后重新运行 python3 -m app.main ...\n"
        "如果仍然失败，请在 Jetson 桌面终端执行：xhost +SI:localuser:jetson",
        file=sys.stderr,
    )


def main(argv: Optional[List[str]] = None) -> int:
    args = parse_args(argv)
    if args.continuous_safe_distance:
        print(UNSUPPORTED_CONTINUOUS_SAFE_DISTANCE_MESSAGE, file=sys.stderr)
        return 2
    if not _check_display_available():
        _print_display_help()
        return 2

    app = QApplication(sys.argv if argv is None else ["arm_mole_qt", *argv])
    app.setApplicationName("机械臂点痣装置")
    window = MainWindow(args)
    if args.fullscreen:
        window.setWindowFlags(Qt.Window | Qt.FramelessWindowHint | Qt.WindowStaysOnTopHint)

        def force_fullscreen() -> None:
            screen = app.primaryScreen()
            if screen is not None:
                window.setGeometry(screen.geometry())
            window.setWindowState(window.windowState() | Qt.WindowFullScreen)
            window.showFullScreen()
            window._update_fullscreen_button()
            window.raise_()
            window.activateWindow()

        force_fullscreen()
        QTimer.singleShot(250, force_fullscreen)
        QTimer.singleShot(800, force_fullscreen)
    else:
        window.show()
    return app.exec_()


if __name__ == "__main__":
    raise SystemExit(main())
