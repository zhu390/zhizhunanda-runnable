import argparse
import time

from .camera_utils import apply_flip, open_camera
from .undistort import Undistorter
from .vision.rect_detect import detect_rectangles, draw_detected_rect


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Jetson camera rectangle detection preview.")
    parser.add_argument("--camera", default="0")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=120)
    parser.add_argument("--display", type=int, choices=[0, 1], default=1)
    parser.add_argument(
        "--flip",
        choices=["none", "0", "1", "-1"],
        default="-1",
        help="Camera flip: none, 0 vertical, 1 horizontal, -1 rotate 180.",
    )
    parser.add_argument("--calib-params", default="", help="calib_solve.py output .npz path.")
    parser.add_argument("--min-area-ratio", type=float, default=0.005)
    parser.add_argument("--max-area-ratio", type=float, default=0.5)
    parser.add_argument("--angle-tol", type=float, default=25.0)
    parser.add_argument("--print-interval", type=float, default=0.5)
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    import cv2  # type: ignore

    cap, label = open_camera(args.camera, args.width, args.height, args.fps)
    undistorter = None
    if args.calib_params:
        undistorter = Undistorter.load(args.calib_params, (args.width, args.height))
        if undistorter is None:
            print(f"undistort disabled: cannot load {args.calib_params}", flush=True)
        else:
            print(
                f"undistort enabled: fx={undistorter.fx:.1f} fy={undistorter.fy:.1f} "
                f"cx={undistorter.cx:.1f} cy={undistorter.cy:.1f}",
                flush=True,
            )

    print(f"camera opened: {label}", flush=True)
    win_name = f"detect {label}"
    if args.display:
        cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)

    count = 0
    start = time.time()
    last_print = 0.0

    try:
        while True:
            ok, frame = cap.read()
            if not ok or frame is None:
                time.sleep(0.01)
                continue

            frame = apply_flip(frame, args.flip)
            if undistorter is not None:
                frame = undistorter.apply(frame)

            rects = detect_rectangles(
                frame,
                min_area_ratio=args.min_area_ratio,
                max_area_ratio=args.max_area_ratio,
                angle_tol=args.angle_tol,
            )
            best = rects[0] if rects else None

            count += 1
            now = time.time()
            fps = count / max(1e-6, now - start)

            if now - last_print >= args.print_interval:
                last_print = now
                if best is None:
                    print(f"fps={fps:.1f} rect=none", flush=True)
                else:
                    cx, cy = best.center
                    print(f"fps={fps:.1f} cx={cx:.1f} cy={cy:.1f} area={best.area:.0f}", flush=True)

            if args.display:
                if best is not None:
                    draw_detected_rect(frame, best)
                h, w = frame.shape[:2]
                cv2.drawMarker(
                    frame,
                    (w // 2, h // 2),
                    (255, 0, 0),
                    markerType=cv2.MARKER_CROSS,
                    markerSize=18,
                    thickness=2,
                )
                cv2.putText(frame, f"FPS {fps:.1f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                cv2.imshow(win_name, frame)
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q") or key == 27:
                    break
    except KeyboardInterrupt:
        print("\nstopped", flush=True)
    finally:
        cap.release()
        if args.display:
            cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
