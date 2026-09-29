"""Camera/laser XY-offset tuning helper.

Usage:
    python3 -m jetson_vision.laser_offset_tuner \
        --camera 0 --width 960 --height 540 --work-distance-mm 17.5

Keys:
    mouse  click actual laser spot to set the working center
    a/d    move working center left/right by 1 px
    A/D    move working center left/right by 10 px
    i/k    move working center up/down by 1 px
    I/K    move working center up/down by 10 px
    s/p    print current recommended arguments
    q/ESC  quit
"""

import argparse
import time

import cv2

from .camera_utils import apply_flip, open_camera


MIN_WORK_DISTANCE_MM = 15.0
MAX_WORK_DISTANCE_MM = 20.0


def parse_args(argv=None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Tune camera-to-laser XY offset without Qt.")
    parser.add_argument("--camera", default="0")
    parser.add_argument("--width", type=int, default=960)
    parser.add_argument("--height", type=int, default=540)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--flip", choices=["none", "0", "1", "-1"], default="none")
    parser.add_argument(
        "--work-distance-mm",
        type=float,
        required=True,
        help="Measured tool-to-target working distance; must be 15 through 20 mm.",
    )
    parser.add_argument(
        "--laser-offset-x-px",
        type=int,
        default=0,
        help="Optional measured starting X offset. Default 0 starts at image center.",
    )
    parser.add_argument("--laser-offset-y-px", type=int, default=0)
    args = parser.parse_args(argv)
    if not MIN_WORK_DISTANCE_MM <= args.work_distance_mm <= MAX_WORK_DISTANCE_MM:
        parser.error(
            "--work-distance-mm must be between "
            f"{MIN_WORK_DISTANCE_MM:g} and {MAX_WORK_DISTANCE_MM:g} mm"
        )
    return args


def reference_from_offset(
    width: int,
    height: int,
    offset_x_px: int,
    offset_y_px: int,
):
    """Return an in-frame reference pixel and its effective center offset."""

    cx = int(width) // 2
    cy = int(height) // 2
    reference_x = max(0, min(int(width) - 1, cx + int(offset_x_px)))
    reference_y = max(0, min(int(height) - 1, cy + int(offset_y_px)))
    return reference_x, reference_y, reference_x - cx, reference_y - cy


def calibration_text(
    work_distance_mm: float,
    width: int,
    height: int,
    reference_x: int,
    reference_y: int,
    offset_x_px: int,
    offset_y_px: int,
) -> str:
    return (
        f"work_distance_mm={work_distance_mm:.2f} "
        f"resolution={width}x{height} "
        f"reference_pixel=({reference_x},{reference_y}) "
        f"offset_px=({offset_x_px},{offset_y_px})"
    )


def draw_crosshair(frame, x: int, y: int, color, label: str) -> None:
    h, w = frame.shape[:2]
    cv2.line(frame, (x, 0), (x, h - 1), color, 1)
    cv2.line(frame, (0, y), (w - 1, y), color, 1)
    cv2.circle(frame, (x, y), 6, color, 2)
    cv2.putText(frame, label, (max(4, x + 8), max(18, y - 8)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)


def main() -> int:
    args = parse_args()
    cap, label = open_camera(args.camera, args.width, args.height, args.fps)
    print(f"camera opened: {label}", flush=True)
    print(
        f"measured work distance: {args.work_distance_mm:.2f} mm "
        f"(allowed {MIN_WORK_DISTANCE_MM:g}-{MAX_WORK_DISTANCE_MM:g} mm)",
        flush=True,
    )
    print("This tool only opens the camera; it never enables a laser or arm output.", flush=True)
    print("Move the red aim reference to the real low-power visible aiming spot.", flush=True)
    print("mouse: click laser spot, a/d: X +/-1 px, A/D: X +/-10 px, i/k: Y -/+1 px, I/K: Y -/+10 px, s/p: print, q: quit", flush=True)

    offset_x_px = int(args.laser_offset_x_px)
    offset_y_px = int(args.laser_offset_y_px)
    state = {
        "cx": 0,
        "cy": 0,
        "offset_x_px": offset_x_px,
        "offset_y_px": offset_y_px,
    }
    win = "Laser Offset Tuner"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)

    def on_mouse(event, x, y, flags, param) -> None:
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        state["offset_x_px"] = int(x) - int(state["cx"])
        state["offset_y_px"] = int(y) - int(state["cy"])
        print(
            f"mouse set: offset_x_px={state['offset_x_px']} "
            f"offset_y_px={state['offset_y_px']}",
            flush=True,
        )

    cv2.setMouseCallback(win, on_mouse)
    last_print = 0.0

    try:
        while True:
            ok, frame = cap.read()
            if not ok or frame is None:
                time.sleep(0.01)
                continue

            frame = apply_flip(frame, args.flip)
            h, w = frame.shape[:2]
            cx = w // 2
            cy = h // 2
            state["cx"] = cx
            state["cy"] = cy
            offset_x_px = int(state["offset_x_px"])
            offset_y_px = int(state["offset_y_px"])

            working_x, working_y, offset_x_px, offset_y_px = reference_from_offset(
                w,
                h,
                offset_x_px,
                offset_y_px,
            )
            state["offset_x_px"] = offset_x_px
            state["offset_y_px"] = offset_y_px
            shown = frame.copy()
            draw_crosshair(shown, cx, cy, (0, 255, 0), "camera center")
            draw_crosshair(shown, working_x, working_y, (0, 0, 255), "aim reference")

            record = calibration_text(
                args.work_distance_mm,
                w,
                h,
                working_x,
                working_y,
                offset_x_px,
                offset_y_px,
            )
            cv2.putText(
                shown,
                f"distance={args.work_distance_mm:.2f} mm | resolution={w}x{h} | "
                f"reference=({working_x},{working_y})",
                (10, 28),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                2,
            )
            cv2.putText(
                shown,
                f"offset=({offset_x_px},{offset_y_px}) px",
                (10, 50),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                2,
            )
            cv2.putText(shown, "Click laser spot or move red center. s/p=print q=quit",
                        (10, h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
            cv2.imshow(win, shown)

            now = time.time()
            if now - last_print > 3.0:
                last_print = now
                print(record, flush=True)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("a"):
                state["offset_x_px"] -= 1
            elif key == ord("d"):
                state["offset_x_px"] += 1
            elif key == ord("A"):
                state["offset_x_px"] -= 10
            elif key == ord("D"):
                state["offset_x_px"] += 10
            elif key == ord("i"):
                state["offset_y_px"] -= 1
            elif key == ord("k"):
                state["offset_y_px"] += 1
            elif key == ord("I"):
                state["offset_y_px"] -= 10
            elif key == ord("K"):
                state["offset_y_px"] += 10
            elif key in (ord("s"), ord("p")):
                offset_x_px = int(state["offset_x_px"])
                offset_y_px = int(state["offset_y_px"])
                reference_x, reference_y, offset_x_px, offset_y_px = reference_from_offset(
                    w,
                    h,
                    offset_x_px,
                    offset_y_px,
                )
                print("measured calibration:", flush=True)
                print(
                    "  "
                    + calibration_text(
                        args.work_distance_mm,
                        w,
                        h,
                        reference_x,
                        reference_y,
                        offset_x_px,
                        offset_y_px,
                    ),
                    flush=True,
                )
                print("recommended Qt args:", flush=True)
                print(
                    f"  --laser-offset-x-px {offset_x_px} --laser-offset-y-px {offset_y_px}",
                    flush=True,
                )
    finally:
        cap.release()
        cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
