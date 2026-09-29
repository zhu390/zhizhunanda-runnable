import argparse
import time
from collections import deque
from pathlib import Path

import cv2

from .camera_utils import apply_flip, open_camera
from .undistort import Undistorter
from .vision.reference_mole_detect import ReferenceMoleDetector
from .vision.target_stability import stable_center


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="基于无黑痣基准图的实时黑痣检测")
    parser.add_argument("--camera", default="0")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--flip", choices=["none", "0", "1", "-1"], default="none")
    parser.add_argument("--calib-params", default="calib_params.npz")
    parser.add_argument("--reference", default="reference/clean_face.png")
    parser.add_argument("--display", type=int, choices=[0, 1], default=1)
    parser.add_argument("--stable-window", type=int, default=5)
    parser.add_argument("--stable-required", type=int, default=4)
    parser.add_argument("--stable-radius", type=float, default=5.0)
    parser.add_argument("--print-interval", type=float, default=0.5)
    parser.add_argument("--max-frames", type=int, default=0,
                        help="处理指定帧数后自动退出，0 表示持续运行")
    parser.add_argument("--save-overlay", default="",
                        help="退出前保存最后一帧带检测标注的结果图")
    return parser.parse_args()


def draw_detection_overlay(frame, moles, stable, state: str):
    shown = frame.copy()
    for index, mole in enumerate(moles):
        color = (0, 255, 0) if index == 0 else (0, 165, 255)
        cv2.circle(shown, mole.center, mole.radius + 5, color, 2)
        cv2.putText(
            shown,
            f"#{index + 1} ({mole.center[0]},{mole.center[1]}) c={mole.confidence:.2f}",
            (mole.center[0] + 10, max(18, mole.center[1] - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            color,
            1,
        )
    if stable is not None:
        cv2.drawMarker(shown, stable, (0, 0, 255), cv2.MARKER_CROSS, 17, 2)
        cv2.circle(shown, stable, 10, (0, 255, 0), 2)
    state_color = (0, 255, 0) if state in ("NO_TARGET", "STABLE_TARGET") else (0, 255, 255)
    cv2.putText(shown, state, (10, 28),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, state_color, 2)
    return shown


def main() -> int:
    args = parse_args()
    reference = cv2.imread(args.reference)
    if reference is None:
        print(f"ERROR: 无法读取基准图 {args.reference}", flush=True)
        return 2
    if reference.shape[1] != args.width or reference.shape[0] != args.height:
        print(
            f"ERROR: 基准图尺寸 {reference.shape[1]}x{reference.shape[0]} "
            f"与运行尺寸 {args.width}x{args.height} 不一致",
            flush=True,
        )
        return 2

    detector = ReferenceMoleDetector(reference)
    cap, label = open_camera(args.camera, args.width, args.height, args.fps)
    undistorter = Undistorter.load(args.calib_params, (args.width, args.height))
    if undistorter is None:
        print(f"ERROR: 无法加载标定参数 {args.calib_params}", flush=True)
        cap.release()
        return 2

    print(f"摄像头已打开: {label}", flush=True)
    print("基准差分检测已启动，按 N 切换调试视图，按 Q 退出", flush=True)
    views = ["overlay", "aligned", "mask", "dark_diff", "binary"]
    view_index = 0
    history = deque(maxlen=max(1, args.stable_window))
    count = 0
    state_counts = {
        "NO_TARGET": 0,
        "CANDIDATE": 0,
        "STABLE_TARGET": 0,
        "ALIGN_FAILED": 0,
    }
    started = time.time()
    last_print = 0.0
    win = "Reference Mole Detection"
    if args.display:
        cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    last_overlay = None

    try:
        while True:
            ok, frame = cap.read()
            if not ok or frame is None:
                time.sleep(0.01)
                continue
            frame = apply_flip(frame, args.flip)
            frame = undistorter.apply(frame)
            moles, alignment, debug = detector.detect(frame, return_debug=True)

            primary = moles[0] if moles else None
            history.append(primary.center if primary is not None and alignment.ok else None)
            stable = stable_center(
                history, max(1, args.stable_required), args.stable_radius
            )

            count += 1
            if not alignment.ok:
                state = "ALIGN_FAILED"
            elif stable is not None:
                state = "STABLE_TARGET"
            elif moles:
                state = "CANDIDATE"
            else:
                state = "NO_TARGET"
            state_counts[state] += 1
            if args.save_overlay:
                last_overlay = draw_detection_overlay(debug["aligned"], moles, stable, state)
            now = time.time()
            fps = count / max(now - started, 1e-6)
            if now - last_print >= args.print_interval:
                last_print = now
                if not alignment.ok:
                    print(f"fps={fps:.1f} ALIGN_FAILED {alignment.reason}", flush=True)
                elif stable is not None:
                    print(
                        f"fps={fps:.1f} STABLE_TARGET u={stable[0]} v={stable[1]} "
                        f"candidates={len(moles)}",
                        flush=True,
                    )
                elif moles:
                    print(
                        f"fps={fps:.1f} CANDIDATE u={primary.center[0]} "
                        f"v={primary.center[1]} confidence={primary.confidence:.2f}",
                        flush=True,
                    )
                else:
                    print(f"fps={fps:.1f} NO_TARGET", flush=True)

            if args.display:
                mode = views[view_index]
                if mode == "overlay":
                    shown = draw_detection_overlay(debug["aligned"], moles, stable, state)
                elif mode == "aligned":
                    shown = debug["aligned"]
                else:
                    shown = cv2.cvtColor(debug[mode], cv2.COLOR_GRAY2BGR)
                cv2.putText(shown, f"view={mode} N=next Q=quit", (10, args.height - 12),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
                cv2.imshow(win, shown)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27):
                    break
                if key in (ord("n"), ord("N")):
                    view_index = (view_index + 1) % len(views)
            if args.max_frames > 0 and count >= args.max_frames:
                break
    except KeyboardInterrupt:
        print("\n已停止", flush=True)
    finally:
        cap.release()
        if args.display:
            cv2.destroyAllWindows()
    if args.save_overlay and last_overlay is not None:
        overlay_path = Path(args.save_overlay)
        overlay_path.parent.mkdir(parents=True, exist_ok=True)
        if not cv2.imwrite(str(overlay_path), last_overlay):
            print(f"WARN: 无法保存检测结果图 {overlay_path}", flush=True)
        else:
            print(f"检测结果图已保存: {overlay_path}", flush=True)
    print(
        "SUMMARY " + " ".join(
            f"{state}={state_counts[state]}"
            for state in ("NO_TARGET", "CANDIDATE", "STABLE_TARGET", "ALIGN_FAILED")
        ),
        flush=True,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
