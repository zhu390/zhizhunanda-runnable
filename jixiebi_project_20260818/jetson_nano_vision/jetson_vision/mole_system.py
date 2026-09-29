"""无黑痣基准采集与黑痣检测一体化程序。

可选串口仅保留给隔离的旧协议台架调试，不属于正式比赛流程，也不得与
Qt 主程序同时打开同一串口。

按键：
    R  在当前位置重新采集无黑痣基准（必须先取下全部黑痣）
    D  使用当前基准进入黑痣检测
    P  返回普通预览
    N  在检测调试视图之间切换
    S  保存当前带标注画面
    Q  退出
"""
import argparse
import json
import time
from collections import deque
from datetime import datetime, timezone
from pathlib import Path

import cv2

from .camera_utils import apply_flip, open_camera
from .reference_capture import calibration_sha256, save_median_reference
from .reference_mole_preview import draw_detection_overlay
from .undistort import Undistorter
from .vision.reference_mole_detect import ReferenceMoleDetector
from .vision.target_stability import stable_center


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="单目黑痣定位一体化程序（隔离旧协议调试工具）")
    parser.add_argument("--camera", default="0")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--flip", choices=["none", "0", "1", "-1"], default="none")
    parser.add_argument("--calib-params", default="calib_params.npz")
    parser.add_argument("--reference", default="reference/clean_face.png")
    parser.add_argument("--reference-meta", default="reference/reference_meta.json")
    parser.add_argument("--reference-frames", type=int, default=30)
    parser.add_argument("--countdown", type=float, default=3.0,
                        help="按 R 后等待多少秒再采集，方便移开手")
    parser.add_argument("--stable-window", type=int, default=5)
    parser.add_argument("--stable-required", type=int, default=4)
    parser.add_argument("--stable-radius", type=float, default=5.0)
    parser.add_argument("--save-dir", default="test_results")
    parser.add_argument("--print-interval", type=float, default=0.5)
    parser.add_argument(
        "--serial-port",
        default="",
        help="隔离的 legacy 台架调试串口；正式运行留空并由 Qt 独占串口",
    )
    parser.add_argument("--baudrate", type=int, default=115200)
    return parser.parse_args()


def _load_detector(reference_path: Path, width: int, height: int):
    if not reference_path.exists():
        return None
    reference = cv2.imread(str(reference_path))
    if reference is None or reference.shape[:2] != (height, width):
        print(f"基准图无效或尺寸不一致: {reference_path}", flush=True)
        return None
    return ReferenceMoleDetector(reference)


def _write_reference_metadata(
    path: Path,
    reference_path: Path,
    args: argparse.Namespace,
    camera_label: str,
) -> None:
    payload = {
        "schema_version": 1,
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "camera": args.camera,
        "camera_label": camera_label,
        "image_size": [args.width, args.height],
        "requested_fps": args.fps,
        "flip": args.flip,
        "frame_count": args.reference_frames,
        "undistorted": True,
        "calib_params": str(Path(args.calib_params).resolve()),
        "calib_sha256": calibration_sha256(args.calib_params),
        "reference_image": str(reference_path.resolve()),
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")


def _draw_header(frame, state: str, detail: str, color) -> None:
    cv2.putText(frame, state, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.72, color, 2)
    cv2.putText(frame, detail, (10, 54), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
    cv2.putText(
        frame,
        "R=new clean reference | D=detect | P=preview | N=view | S=save | Q=quit",
        (10, frame.shape[0] - 12),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.42,
        (255, 255, 0),
        1,
    )


def main() -> int:
    args = parse_args()
    if args.reference_frames < 3:
        print("ERROR: --reference-frames 至少为 3", flush=True)
        return 2

    cap, label = open_camera(args.camera, args.width, args.height, args.fps)
    undistorter = Undistorter.load(args.calib_params, (args.width, args.height))
    if undistorter is None:
        print(f"ERROR: 无法加载标定参数 {args.calib_params}", flush=True)
        cap.release()
        return 2

    sender = None
    if args.serial_port:
        from .serial_sender import SerialSender

        try:
            sender = SerialSender(
                args.serial_port,
                args.baudrate,
                protocol="legacy",
                auto_heartbeat=False,
            )
            print("警告：当前为隔离 legacy 串口调试，禁止同时运行正式 Qt", flush=True)
            print(
                f"串口已连接: {args.serial_port} @ {args.baudrate} protocol=legacy",
                flush=True,
            )
        except Exception as exc:
            print(f"串口打开失败: {exc}", flush=True)

    reference_path = Path(args.reference)
    meta_path = Path(args.reference_meta)
    detector = _load_detector(reference_path, args.width, args.height)
    state = "PREVIEW"
    capture_frames = []
    countdown_until = 0.0
    history = deque(maxlen=max(1, args.stable_window))
    views = ["overlay", "aligned", "mask", "dark_diff", "binary"]
    view_index = 0
    last_overlay = None
    last_print = 0.0
    last_serial_error_print = 0.0
    win = "Integrated Mole System"

    print(f"摄像头已打开: {label}", flush=True)
    print("R=重采无黑痣基准 D=检测 P=预览 N=切换视图 S=保存 Q=退出", flush=True)
    if detector is None:
        print("尚无有效基准图：取下全部黑痣后按 R", flush=True)
    else:
        print(f"已加载基准图: {reference_path}", flush=True)

    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    try:
        while True:
            ok, frame = cap.read()
            if not ok or frame is None:
                time.sleep(0.01)
                continue
            frame = apply_flip(frame, args.flip)
            frame = undistorter.apply(frame)
            shown = frame.copy()
            now = time.time()

            if sender:
                for status in sender.poll_statuses():
                    print(
                        f"RA6M5 状态: {status.label} "
                        f"func=0x{status.func:02X} value=0x{status.value:02X}",
                        flush=True,
                    )

            if state == "COUNTDOWN":
                remaining = max(0.0, countdown_until - now)
                _draw_header(
                    shown,
                    "REMOVE ALL MOLES",
                    f"capturing clean reference in {remaining:.1f}s",
                    (0, 0, 255),
                )
                if remaining <= 0:
                    capture_frames.clear()
                    state = "CAPTURE_REFERENCE"

            elif state == "CAPTURE_REFERENCE":
                capture_frames.append(frame.copy())
                _draw_header(
                    shown,
                    "CAPTURE_REFERENCE",
                    f"frames {len(capture_frames)}/{args.reference_frames}",
                    (0, 255, 255),
                )
                if len(capture_frames) >= args.reference_frames:
                    reference = save_median_reference(capture_frames, reference_path)
                    _write_reference_metadata(meta_path, reference_path, args, label)
                    detector = ReferenceMoleDetector(reference)
                    history.clear()
                    state = "PREVIEW"
                    print(f"新基准图已保存: {reference_path}", flush=True)
                    print("现在可以贴黑痣，贴好后按 D", flush=True)

            elif state == "DETECT" and detector is not None:
                moles, alignment, debug = detector.detect(frame, return_debug=True)
                primary = moles[0] if moles else None
                history.append(primary.center if primary is not None and alignment.ok else None)
                stable = stable_center(
                    history, args.stable_required, args.stable_radius
                )
                if sender and alignment.ok and primary is not None and stable is not None:
                    sent = sender.send_target_error(
                        stable[0],
                        stable[1],
                        frame.shape[1],
                        frame.shape[0],
                    )
                    if not sent and now - last_serial_error_print >= args.print_interval:
                        last_serial_error_print = now
                        print(f"串口发送失败: {sender.last_error}", flush=True)
                if not alignment.ok:
                    result_state = "ALIGN_FAILED"
                elif stable is not None:
                    result_state = "STABLE_TARGET"
                elif moles:
                    result_state = "CANDIDATE"
                else:
                    result_state = "NO_TARGET"

                last_overlay = draw_detection_overlay(
                    debug["aligned"], moles, stable, result_state
                )
                mode = views[view_index]
                if mode == "overlay":
                    shown = last_overlay.copy()
                elif mode == "aligned":
                    shown = debug["aligned"].copy()
                else:
                    shown = cv2.cvtColor(debug[mode], cv2.COLOR_GRAY2BGR)

                if not alignment.ok:
                    cv2.putText(
                        shown,
                        "CAMERA MOVED: remove mole and press R",
                        (10, 54),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.55,
                        (0, 0, 255),
                        2,
                    )
                if now - last_print >= args.print_interval:
                    last_print = now
                    if stable is not None and primary is not None:
                        print(
                            f"STABLE_TARGET u={stable[0]} v={stable[1]} "
                            f"confidence={primary.confidence:.2f} candidates={len(moles)}",
                            flush=True,
                        )
                    elif not alignment.ok:
                        print(f"ALIGN_FAILED {alignment.reason}", flush=True)
                    else:
                        print(result_state, flush=True)
                cv2.putText(
                    shown,
                    f"view={mode} | R=reference D=detect P=preview N=view S=save Q=quit",
                    (10, args.height - 12),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.42,
                    (255, 255, 0),
                    1,
                )

            else:
                detail = (
                    "reference ready: attach mole and press D"
                    if detector is not None
                    else "no reference: remove all moles and press R"
                )
                _draw_header(shown, "PREVIEW", detail, (0, 255, 0))

            cv2.imshow(win, shown)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key in (ord("r"), ord("R")):
                state = "COUNTDOWN"
                countdown_until = time.time() + max(0.0, args.countdown)
                capture_frames.clear()
                history.clear()
                print("准备重采基准：请确保脸上没有任何黑痣并移开手", flush=True)
            elif key in (ord("d"), ord("D")):
                if detector is None:
                    print("没有有效基准图，先取下黑痣并按 R", flush=True)
                else:
                    state = "DETECT"
                    history.clear()
                    print("进入黑痣检测", flush=True)
            elif key in (ord("p"), ord("P")):
                state = "PREVIEW"
                history.clear()
            elif key in (ord("n"), ord("N")):
                view_index = (view_index + 1) % len(views)
            elif key in (ord("s"), ord("S")):
                image = last_overlay if last_overlay is not None else shown
                save_dir = Path(args.save_dir)
                save_dir.mkdir(parents=True, exist_ok=True)
                stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                save_path = save_dir / f"mole_result_{stamp}.png"
                if cv2.imwrite(str(save_path), image):
                    print(f"结果图已保存: {save_path}", flush=True)
    except KeyboardInterrupt:
        print("\n已停止", flush=True)
    finally:
        cap.release()
        if sender:
            sender.close()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
