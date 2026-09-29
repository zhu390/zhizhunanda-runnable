import argparse
import re
import time
from pathlib import Path
from typing import Optional, Tuple

import cv2
import numpy as np


BACKENDS = {
    "auto": cv2.CAP_ANY,
    "v4l2": getattr(cv2, "CAP_V4L2", cv2.CAP_ANY),
    "msmf": getattr(cv2, "CAP_MSMF", cv2.CAP_ANY),
    "dshow": getattr(cv2, "CAP_DSHOW", cv2.CAP_ANY),
}

PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_ASTRA_RGB_CALIB_IMAGES = PROJECT_ROOT / "calibration" / "astra_rgb" / "images"


def parse_pattern(value: str) -> Tuple[int, int]:
    parts = value.lower().split("x")
    if len(parts) != 2:
        raise argparse.ArgumentTypeError("pattern must be like 6x9")
    return int(parts[0]), int(parts[1])


def parse_flip(value: str) -> Optional[int]:
    if value.lower() in ("none", "no", "off"):
        return None
    if value not in ("-1", "0", "1"):
        raise argparse.ArgumentTypeError("flip must be none, -1, 0, or 1")
    return int(value)


def apply_flip(frame: np.ndarray, flip: Optional[int]) -> np.ndarray:
    if flip is None:
        return frame
    return cv2.flip(frame, flip)


def existing_count(out_dir: Path) -> int:
    pattern = re.compile(r"calib_(\d+)\.png$", re.IGNORECASE)
    numbers = []
    if not out_dir.exists():
        return 0
    for path in out_dir.glob("calib_*.png"):
        match = pattern.match(path.name)
        if match:
            numbers.append(int(match.group(1)))
    return max(numbers) if numbers else 0


def write_image(path: Path, image: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    ok, encoded = cv2.imencode(path.suffix or ".png", image)
    if not ok:
        raise RuntimeError("failed to encode image: {}".format(path))
    path.write_bytes(encoded.tobytes())


def find_chessboard(frame: np.ndarray, pattern: Tuple[int, int], fast_check: bool):
    rows, cols = pattern
    pattern_size = (cols, rows)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
    if fast_check:
        flags += cv2.CALIB_CB_FAST_CHECK
    found, corners = cv2.findChessboardCorners(gray, pattern_size, flags=flags)
    if not found:
        return False, None

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01)
    refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
    return True, refined


def open_capture(args) -> cv2.VideoCapture:
    try:
        cap = cv2.VideoCapture(args.camera, BACKENDS[args.backend])
    except TypeError:
        # Older OpenCV Python bindings, such as Jetson's OpenCV 3.2,
        # only accept the camera index.
        cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        raise RuntimeError("cannot open camera index {} with backend {}".format(args.camera, args.backend))

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    cap.set(cv2.CAP_PROP_FPS, args.fps)
    if args.fourcc != "native":
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*args.fourcc))
    if hasattr(cv2, "CAP_PROP_BUFFERSIZE"):
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return cap


def delete_last(out_dir: Path, saved: int) -> int:
    if saved <= 0:
        print("no saved image to delete", flush=True)
        return 0
    path = out_dir / "calib_{:03d}.png".format(saved)
    if path.exists():
        path.unlink()
        print("deleted {}".format(path), flush=True)
    return existing_count(out_dir)


def main() -> int:
    parser = argparse.ArgumentParser(description="Capture RGB chessboard images for Astra Pro Plus calibration.")
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--backend", choices=sorted(BACKENDS), default="v4l2")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--fourcc", choices=["MJPG", "YUY2", "YUYV", "native"], default="MJPG")
    parser.add_argument("--pattern", type=parse_pattern, default=(6, 9), help="Inner corners, rows x cols. Default: 6x9.")
    parser.add_argument("--flip", type=parse_flip, default=None, help="none, -1, 0, or 1. Default: none.")
    parser.add_argument("--out", default=str(DEFAULT_ASTRA_RGB_CALIB_IMAGES))
    parser.add_argument("--detect-every", type=int, default=5, help="Run chessboard detection every N frames. Default: 5.")
    parser.add_argument("--preview-scale", type=float, default=1.0, help="Display scale only. Saved image keeps original size.")
    parser.add_argument("--save-cooldown", type=float, default=1.0, help="Minimum seconds between two saves. Default: 1.0.")
    args = parser.parse_args()

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    saved = existing_count(out_dir)

    cap = open_capture(args)
    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH) or 0)
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT) or 0)
    actual_fps = float(cap.get(cv2.CAP_PROP_FPS) or 0.0)

    rows, cols = args.pattern
    print("camera index: {}, backend: {}".format(args.camera, args.backend), flush=True)
    print("requested: {}x{}@{}, actual: {}x{}@{:.1f}".format(
        args.width, args.height, args.fps, actual_w, actual_h, actual_fps
    ), flush=True)
    print("pattern: {}x{} inner corners, output: {}".format(rows, cols, out_dir.resolve()), flush=True)
    print("detect_every: {}, preview_scale: {}, save_cooldown: {}".format(
        args.detect_every, args.preview_scale, args.save_cooldown
    ), flush=True)
    print("keys: SPACE=save when board is detected, X=delete last, Q/Esc=quit", flush=True)

    window = "Astra Pro Plus RGB chessboard capture"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    frame_index = 0
    live_found = False
    live_corners = None
    last_save_time = 0.0

    try:
        while True:
            ok, frame = cap.read()
            if not ok or frame is None:
                print("WARN: frame read failed", flush=True)
                continue

            frame = apply_flip(frame, args.flip)
            frame_index += 1
            if frame_index % max(1, args.detect_every) == 0:
                live_found, live_corners = find_chessboard(frame, args.pattern, fast_check=True)

            shown = frame.copy()
            if live_found and live_corners is not None:
                cv2.drawChessboardCorners(shown, (cols, rows), live_corners, True)
            color = (0, 255, 0) if live_found else (0, 0, 255)
            status = "saved={} board={} SPACE=save X=delete Q=quit".format(saved, "OK" if live_found else "NO")
            cv2.putText(shown, status, (12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2, cv2.LINE_AA)
            if args.preview_scale != 1.0:
                preview_w = max(1, int(shown.shape[1] * args.preview_scale))
                preview_h = max(1, int(shown.shape[0] * args.preview_scale))
                shown = cv2.resize(shown, (preview_w, preview_h), interpolation=cv2.INTER_AREA)
            cv2.imshow(window, shown)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key in (ord("x"), ord("X")):
                saved = delete_last(out_dir, saved)
            elif key == 32:
                now = time.monotonic()
                if now - last_save_time < args.save_cooldown:
                    continue
                save_found, _ = find_chessboard(frame, args.pattern, fast_check=False)
                if not save_found:
                    print("not saved: chessboard was not detected", flush=True)
                    continue
                saved += 1
                path = out_dir / "calib_{:03d}.png".format(saved)
                write_image(path, frame)
                last_save_time = now
                print("saved {}".format(path), flush=True)
    finally:
        cap.release()
        cv2.destroyAllWindows()

    print("done. saved images: {}".format(saved), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
