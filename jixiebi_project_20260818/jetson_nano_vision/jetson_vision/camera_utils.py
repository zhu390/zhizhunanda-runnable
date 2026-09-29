import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple, Union


def camera_source(value: str) -> Union[int, str]:
    return int(value) if value.isdigit() else value


def apply_flip(frame, flip: str):
    """Apply the configured image flip mode."""
    if flip == "none":
        return frame
    import cv2  # type: ignore

    return cv2.flip(frame, int(flip))


def open_camera(
    value: str,
    width: int,
    height: int,
    fps: int,
    *,
    brightness: Optional[float] = None,
    gain: Optional[float] = None,
    backlight_compensation: Optional[float] = None,
) -> Tuple[Any, str]:
    if value == "picamera2":
        return _open_picamera2(width, height, fps)
    if value in ("csi", "jetson-csi") or value.startswith("csi:"):
        sensor_id = _parse_csi_sensor_id(value)
        cap = _try_open_jetson_csi(sensor_id, width, height, fps)
        if cap is not None:
            return cap, "jetson-csi sensor={} {}fps".format(sensor_id, fps)
        raise RuntimeError("cannot open Jetson CSI camera sensor={}".format(sensor_id))
    if "!" in value:
        cap = _try_open_gstreamer(value)
        if cap is not None:
            return cap, "gstreamer pipeline"
        raise RuntimeError("cannot open GStreamer camera pipeline")

    candidates = _camera_candidates(value)
    last_label = candidates[-1] if candidates else value
    fps_candidates = _fps_candidates(fps)
    fourcc_candidates = _fourcc_candidates()

    for candidate in candidates:
        for try_fps in fps_candidates:
            for fourcc in fourcc_candidates:
                for backend_name, backend in _capture_backends():
                    opened = _try_open_camera(
                        candidate,
                        width,
                        height,
                        try_fps,
                        fourcc,
                        backend,
                        brightness=brightness,
                        gain=gain,
                        backlight_compensation=backlight_compensation,
                    )
                    if opened is not None:
                        cap, actual = opened
                        return cap, "{} {} {}".format(
                            candidate,
                            backend_name,
                            actual,
                        )

    if value == "auto":
        cap = _try_open_jetson_csi(0, width, height, fps)
        if cap is not None:
            return cap, "jetson-csi sensor=0 {}fps".format(fps)
        raise RuntimeError(
            "cannot open camera auto; V4L2 last tried {}; Jetson CSI failed".format(
                last_label
            )
        )

    raise RuntimeError("cannot open camera {}; last tried {}".format(value, last_label))


def _try_open_camera(
    value: str,
    width: int,
    height: int,
    fps: int,
    fourcc: Optional[str],
    backend: int,
    *,
    brightness: Optional[float] = None,
    gain: Optional[float] = None,
    backlight_compensation: Optional[float] = None,
) -> Optional[Tuple[Any, str]]:
    import cv2  # type: ignore

    source = camera_source(value)
    try:
        cap = cv2.VideoCapture(source, backend)
    except TypeError:
        cap = cv2.VideoCapture(source)
    if cap.isOpened():
        # UVC devices renegotiate the stream when FOURCC changes.  Set it
        # first; setting it last can silently reset an already requested
        # resolution or frame interval.
        if fourcc is not None:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_FPS, fps)
        if hasattr(cv2, "CAP_PROP_BUFFERSIZE"):
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        frame_times = []
        first_frame = None
        for _ in range(12):
            ok, frame = cap.read()
            if ok and frame is not None:
                if frame.shape[0] != int(height) or frame.shape[1] != int(width):
                    cap.release()
                    return None
                if first_frame is None:
                    first_frame = frame
                    _apply_camera_controls(
                        cap,
                        cv2,
                        brightness=brightness,
                        gain=gain,
                        backlight_compensation=backlight_compensation,
                    )
                frame_times.append(time.monotonic())
        if first_frame is not None:
            actual_fourcc = _decode_fourcc(cap.get(cv2.CAP_PROP_FOURCC))
            if (
                fourcc is not None
                and actual_fourcc
                and not _fourcc_matches(fourcc, actual_fourcc)
            ):
                cap.release()
                return None
            actual_fps = _finite_positive(cap.get(cv2.CAP_PROP_FPS))
            probe_fps = 0.0
            if len(frame_times) >= 2 and frame_times[-1] > frame_times[0]:
                probe_fps = (len(frame_times) - 1) / (frame_times[-1] - frame_times[0])
            controls = _camera_control_readback(
                cap,
                cv2,
                brightness=brightness,
                gain=gain,
                backlight_compensation=backlight_compensation,
            )
            actual = "req={}x{}@{} {} actual={}x{} {} driver_fps={} probe_fps={}{}".format(
                int(width),
                int(height),
                int(fps),
                fourcc or "native",
                int(first_frame.shape[1]),
                int(first_frame.shape[0]),
                actual_fourcc or "unknown",
                "{:.1f}".format(actual_fps) if actual_fps > 0.0 else "unknown",
                "{:.1f}".format(probe_fps) if probe_fps > 0.0 else "unknown",
                " controls=" + controls if controls else "",
            )
            return cap, actual
    cap.release()
    return None


def _finite_positive(value: Any) -> float:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return 0.0
    if parsed <= 0.0 or parsed != parsed or parsed == float("inf"):
        return 0.0
    return parsed


def _decode_fourcc(value: Any) -> str:
    try:
        code = int(round(float(value)))
    except (TypeError, ValueError):
        return ""
    if code <= 0:
        return ""
    text = "".join(chr((code >> (8 * index)) & 0xFF) for index in range(4))
    return text.rstrip("\x00 ")


def _fourcc_matches(requested: str, actual: str) -> bool:
    requested = str(requested).upper()
    actual = str(actual).upper()
    if requested == actual:
        return True
    return {requested, actual} <= {"YUYV", "YUY2"}


def _camera_control_properties(cv2) -> Dict[str, Optional[int]]:
    return {
        "brightness": getattr(cv2, "CAP_PROP_BRIGHTNESS", None),
        "gain": getattr(cv2, "CAP_PROP_GAIN", None),
        "backlight": getattr(cv2, "CAP_PROP_BACKLIGHT", None),
    }


def _requested_camera_controls(
    *,
    brightness: Optional[float],
    gain: Optional[float],
    backlight_compensation: Optional[float],
) -> Dict[str, Optional[float]]:
    return {
        "brightness": brightness,
        "gain": gain,
        "backlight": backlight_compensation,
    }


def _apply_camera_controls(
    cap,
    cv2,
    *,
    brightness: Optional[float],
    gain: Optional[float],
    backlight_compensation: Optional[float],
) -> None:
    properties = _camera_control_properties(cv2)
    requested = _requested_camera_controls(
        brightness=brightness,
        gain=gain,
        backlight_compensation=backlight_compensation,
    )
    for name, value in requested.items():
        prop = properties.get(name)
        if value is None or prop is None:
            continue
        try:
            cap.set(prop, float(value))
        except Exception:
            # Image acquisition remains useful even when a different camera
            # does not implement one of the YHTek controls.
            continue


def _camera_control_readback(
    cap,
    cv2,
    *,
    brightness: Optional[float],
    gain: Optional[float],
    backlight_compensation: Optional[float],
) -> str:
    properties = _camera_control_properties(cv2)
    requested = _requested_camera_controls(
        brightness=brightness,
        gain=gain,
        backlight_compensation=backlight_compensation,
    )
    parts = []
    for name, value in requested.items():
        prop = properties.get(name)
        if value is None:
            continue
        if prop is None:
            parts.append("{}=unsupported".format(name))
            continue
        try:
            actual = float(cap.get(prop))
        except Exception:
            parts.append("{}=?".format(name))
            continue
        if actual != actual or actual in (float("inf"), float("-inf")):
            parts.append("{}=?".format(name))
            continue
        mismatch = abs(actual - float(value)) > max(0.5, abs(float(value)) * 0.02)
        parts.append(
            "{}={:g}{}".format(
                name,
                actual,
                "(!={:g})".format(float(value)) if mismatch else "",
            )
        )
    return ",".join(parts)


def _capture_backends() -> List[Tuple[str, int]]:
    """Return preferred OpenCV capture backends for the current platform."""
    import cv2  # type: ignore

    if sys.platform == "win32":
        backends = []
        if hasattr(cv2, "CAP_DSHOW"):
            backends.append(("dshow", cv2.CAP_DSHOW))
        if hasattr(cv2, "CAP_MSMF"):
            backends.append(("msmf", cv2.CAP_MSMF))
        return backends or [("auto", cv2.CAP_ANY)]
    if sys.platform.startswith("linux"):
        backends = []
        if hasattr(cv2, "CAP_V4L2"):
            backends.append(("v4l2", cv2.CAP_V4L2))
        backends.append(("auto", cv2.CAP_ANY))
        return backends
    return [("auto", cv2.CAP_ANY)]


def _parse_csi_sensor_id(value: str) -> int:
    if ":" not in value:
        return 0
    _, raw = value.split(":", 1)
    try:
        return max(0, int(raw))
    except ValueError:
        return 0


def _try_open_gstreamer(pipeline: str) -> Optional[Any]:
    import cv2  # type: ignore

    backend = getattr(cv2, "CAP_GSTREAMER", cv2.CAP_ANY)
    try:
        cap = cv2.VideoCapture(pipeline, backend)
    except TypeError:
        cap = cv2.VideoCapture(pipeline)
    if cap.isOpened():
        for _ in range(5):
            ok, frame = cap.read()
            if ok and frame is not None:
                return cap
    cap.release()
    return None


def _try_open_jetson_csi(
    sensor_id: int, width: int, height: int, fps: int
) -> Optional[Any]:
    return _try_open_gstreamer(_jetson_csi_pipeline(sensor_id, width, height, fps))


def _jetson_csi_pipeline(sensor_id: int, width: int, height: int, fps: int) -> str:
    return (
        "nvarguscamerasrc sensor-id={} ! ".format(sensor_id)
        + "video/x-raw(memory:NVMM), width=(int){}, height=(int){}, ".format(
            width, height
        )
        + "format=(string)NV12, framerate=(fraction){}/1 ! ".format(fps)
        + "nvvidconv flip-method=0 ! "
        + "video/x-raw, format=(string)BGRx ! "
        + "videoconvert ! "
        + "video/x-raw, format=(string)BGR ! appsink drop=true sync=false"
    )


class _Picamera2Capture:
    def __init__(self, width: int, height: int, fps: int) -> None:
        from picamera2 import Picamera2  # type: ignore

        self._camera = Picamera2()
        config = self._camera.create_video_configuration(
            main={"size": (width, height), "format": "RGB888"},
            controls={"FrameRate": float(fps)},
        )
        self._camera.configure(config)
        self._camera.start()

    def read(self) -> Tuple[bool, Any]:
        import cv2  # type: ignore

        frame = self._camera.capture_array()
        if frame is None:
            return False, None
        return True, cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

    def release(self) -> None:
        self._camera.stop()
        self._camera.close()


def _open_picamera2(width: int, height: int, fps: int) -> Tuple[Any, str]:
    cap = _Picamera2Capture(width, height, fps)
    ok, frame = cap.read()
    if not ok or frame is None:
        cap.release()
        raise RuntimeError("picamera2 opened but frame read failed")
    return cap, "picamera2"


def _fps_candidates(preferred: int) -> List[int]:
    values = [int(preferred), 60, 30, 15]
    result = []  # type: List[int]
    for value in values:
        if value > 0 and value not in result:
            result.append(value)
    return result


def _fourcc_candidates() -> List[Optional[str]]:
    return ["MJPG", "YUYV", "YUY2", None]


def _camera_candidates(value: str) -> List[str]:
    if value != "auto":
        return [value]

    candidates = []  # type: List[str]
    max_index = 8 if sys.platform == "win32" else 32
    for index in range(max_index):
        candidates.append(str(index))

    if sys.platform.startswith("linux"):
        for dev in sorted(Path("/dev").glob("video*")):
            if dev.name[5:].isdigit() and str(dev) not in candidates:
                candidates.append(str(dev))

        by_id = Path("/dev/v4l/by-id")
        if by_id.exists():
            for dev in sorted(by_id.iterdir()):
                if str(dev) not in candidates:
                    candidates.append(str(dev))

    if not candidates:
        candidates = [str(index) for index in range(32)]
    return candidates
