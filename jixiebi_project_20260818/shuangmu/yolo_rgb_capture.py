import argparse
import csv
import select
import sys
import time
from datetime import datetime
from pathlib import Path

import cv2


DEFAULT_CAMERA = "/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera_SN0001-video-index0"


def parse_args():
    parser = argparse.ArgumentParser(description="Capture Astra RGB images for YOLO arm-tip / face training.")
    parser.add_argument("--camera", default=DEFAULT_CAMERA)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--fourcc", choices=["MJPG", "YUY2", "YUYV", "native"], default="MJPG")
    parser.add_argument("--out", default="../datasets/arm_face_yolo/raw/images")
    parser.add_argument("--prefix", default="astra")
    parser.add_argument("--auto-interval", type=float, default=0.0, help="Seconds between autosaves. 0 disables autosave.")
    parser.add_argument("--start-auto", action="store_true")
    parser.add_argument("--flip", choices=["none", "0", "1", "-1"], default="none")
    parser.add_argument("--rotate", choices=["none", "90cw", "90ccw", "180"], default="none")
    parser.add_argument("--display", type=int, choices=[0, 1], default=1)
    parser.add_argument("--max-images", type=int, default=0)
    parser.add_argument(
        "--stdin-trigger",
        type=int,
        choices=[0, 1],
        default=1,
        help="1=allow Enter or 's'+Enter in the SSH terminal to save",
    )
    return parser.parse_args()


def camera_value(value: str):
    return int(value) if str(value).isdigit() else value


def open_camera(args):
    cap = cv2.VideoCapture(camera_value(args.camera))
    if not cap.isOpened():
        raise RuntimeError("cannot open camera {}".format(args.camera))

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    cap.set(cv2.CAP_PROP_FPS, args.fps)
    if args.fourcc != "native":
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*args.fourcc))
    if hasattr(cv2, "CAP_PROP_BUFFERSIZE"):
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return cap


def transform_frame(frame, flip: str, rotate: str):
    if flip != "none":
        frame = cv2.flip(frame, int(flip))
    if rotate == "90cw":
        frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
    elif rotate == "90ccw":
        frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
    elif rotate == "180":
        frame = cv2.rotate(frame, cv2.ROTATE_180)
    return frame


def draw_text(image, text, origin, color=(255, 255, 255), scale=0.55):
    cv2.putText(image, text, origin, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(image, text, origin, cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1, cv2.LINE_AA)


def make_filename(prefix: str, count: int) -> str:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
    return "{}_{}_{:05d}.jpg".format(prefix, stamp, count)


def append_metadata(meta_path: Path, row):
    write_header = not meta_path.exists()
    with meta_path.open("a", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        if write_header:
            writer.writerow(["filename", "saved_at", "camera", "width", "height", "note"])
        writer.writerow(row)


def save_image(frame, out_dir: Path, meta_path: Path, args, count: int) -> int:
    out_dir.mkdir(parents=True, exist_ok=True)
    filename = make_filename(args.prefix, count + 1)
    path = out_dir / filename
    if not cv2.imwrite(str(path), frame):
        print("save failed:", path, flush=True)
        return count

    append_metadata(
        meta_path,
        [
            filename,
            datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            args.camera,
            frame.shape[1],
            frame.shape[0],
            "",
        ],
    )
    count += 1
    print("saved #{}: {}".format(count, path), flush=True)
    return count


def main() -> int:
    args = parse_args()
    out_dir = Path(args.out)
    meta_path = out_dir.parent / "capture_metadata.csv"
    cap = open_camera(args)

    saved_count = 0
    auto_enabled = bool(args.start_auto and args.auto_interval > 0)
    last_auto_save = 0.0
    last_fps_time = time.time()
    frame_count = 0
    fps = 0.0
    win = "YOLO RGB Capture"
    mouse_save_requested = False

    def on_mouse(event, _x, _y, _flags, _param):
        nonlocal mouse_save_requested
        if event == cv2.EVENT_LBUTTONDOWN:
            mouse_save_requested = True

    print("camera:", args.camera, flush=True)
    print("output:", out_dir, flush=True)
    print("classes to label later: 0=arm_tip, 1=face", flush=True)
    print("keys: S save | A auto | Q/Esc quit", flush=True)
    print("SSH: Enter or S+Enter save | A+Enter auto | Q+Enter quit", flush=True)
    print("touch/click the preview window to save", flush=True)

    if args.display:
        cv2.namedWindow(win, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(win, on_mouse)

    try:
        while True:
            ok, frame = cap.read()
            if not ok or frame is None:
                time.sleep(0.01)
                continue

            frame = transform_frame(frame, args.flip, args.rotate)
            frame_count += 1
            now = time.time()
            if now - last_fps_time >= 1.0:
                fps = frame_count / max(now - last_fps_time, 1e-6)
                frame_count = 0
                last_fps_time = now

            if auto_enabled and now - last_auto_save >= args.auto_interval:
                saved_count = save_image(frame, out_dir, meta_path, args, saved_count)
                last_auto_save = now

            shown = frame.copy()
            draw_text(shown, "saved={} fps={:.1f}".format(saved_count, fps), (12, 28))
            draw_text(
                shown,
                "auto={} interval={:.1f}s".format("on" if auto_enabled else "off", args.auto_interval),
                (12, 54),
            )
            draw_text(shown, "S save | A auto | Q quit", (12, shown.shape[0] - 14), (255, 255, 0), 0.5)

            key = 255
            if args.display:
                cv2.imshow(win, shown)
                key = cv2.waitKey(1) & 0xFF

            terminal_command = None
            if args.stdin_trigger and select.select([sys.stdin], [], [], 0.0)[0]:
                line = sys.stdin.readline()
                if line == "":
                    args.stdin_trigger = 0
                else:
                    terminal_command = line.strip().lower()

            if key in (ord("q"), 27) or terminal_command == "q":
                break

            save_requested = (
                key in (ord("s"), ord("S"), ord(" "))
                or mouse_save_requested
                or terminal_command in ("", "s", "save")
            )
            if save_requested:
                mouse_save_requested = False
                saved_count = save_image(frame, out_dir, meta_path, args, saved_count)

            if key in (ord("a"), ord("A")) or terminal_command == "a":
                if args.auto_interval <= 0:
                    print("autosave disabled: set --auto-interval > 0", flush=True)
                else:
                    auto_enabled = not auto_enabled
                    last_auto_save = 0.0
                    print("auto save:", "on" if auto_enabled else "off", flush=True)

            if args.max_images > 0 and saved_count >= args.max_images:
                break
    except KeyboardInterrupt:
        print("\nstopped", flush=True)
    finally:
        cap.release()
        if args.display:
            cv2.destroyAllWindows()

    print("total saved:", saved_count, flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
