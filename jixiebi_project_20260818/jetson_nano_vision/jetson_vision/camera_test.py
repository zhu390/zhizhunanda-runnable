import argparse
import time

from .camera_utils import open_camera


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="OpenCV camera test.")
    parser.add_argument("--camera", default="auto")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=120)
    parser.add_argument("--display", type=int, choices=[0, 1], default=0)
    parser.add_argument(
        "--flip",
        choices=["none", "0", "1", "-1"],
        default="none",
        help="Optional image flip: 0 vertical, 1 horizontal, -1 rotate 180.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    try:
        cap, label = open_camera(args.camera, args.width, args.height, args.fps)
    except Exception as exc:
        print(f"CAM ERR: {exc}")
        return 2

    print(f"Camera opened: {label}")

    try:
        if args.display:
            import cv2  # type: ignore

        count = 0
        start = time.time()
        while True:
            ok, frame = cap.read()
            if not ok or frame is None:
                print("read failed")
                time.sleep(0.1)
                continue
            frame = _apply_flip(frame, args.flip)

            count += 1
            if count % 30 == 0:
                elapsed = max(1e-6, time.time() - start)
                h, w = frame.shape[:2]
                print(f"frames={count} fps={count / elapsed:.1f} size={w}x{h}")

            if args.display:
                cv2.imshow("camera_test", frame)
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q") or key == 27:
                    break
    except KeyboardInterrupt:
        print("\nCamera test stopped.")
    finally:
        cap.release()
        if args.display:
            cv2.destroyAllWindows()

    return 0


def _apply_flip(frame, flip: str):
    if flip == "none":
        return frame

    import cv2  # type: ignore

    return cv2.flip(frame, int(flip))


if __name__ == "__main__":
    raise SystemExit(main())
