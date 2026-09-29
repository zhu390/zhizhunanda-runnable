"""Single-camera mole preview with an isolated legacy bench output.

The optional serial path is not part of the protocol-v5 application.  Never
run it while the formal Qt process owns the RA6M5 port.
"""

import argparse
import json
import time
from pathlib import Path
from typing import List

from .camera_utils import apply_flip, open_camera
from .undistort import Undistorter
from .vision.mole_detect import DetectParams, DetectedMole, detect_moles

import cv2
import numpy as np


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="单目摄像头黑痣检测")
    parser.add_argument("--camera", default="0", help="摄像头源：设备号/csi/picamera2")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--display", type=int, choices=[0, 1], default=1)
    parser.add_argument("--flip", choices=["none", "0", "1", "-1"], default="none")
    parser.add_argument("--calib-params", default="", help="标定参数 .npz 路径")

    parser.add_argument("--params-json", default="",
                        help="mole_image_test.py 保存的参数 JSON 文件")
    parser.add_argument("--tune", action="store_true",
                        help="开启 trackbar 实时调参（按 s 保存到 --params-json）")

    parser.add_argument(
        "--serial-port",
        default="",
        help="隔离的 legacy 台架调试串口；正式运行留空并由 Qt 独占串口",
    )
    parser.add_argument("--baudrate", type=int, default=115200)

    parser.add_argument("--print-interval", type=float, default=0.5)
    return parser.parse_args()


def draw_moles(frame: np.ndarray, moles: List[DetectedMole]) -> None:
    for i, mole in enumerate(moles):
        cx, cy = mole.center
        cv2.circle(frame, (cx, cy), mole.radius + 5, (0, 255, 0), 2)
        cv2.circle(frame, (cx, cy), 3, (0, 0, 255), -1)
        label = f"#{i+1} ({cx},{cy})"
        cv2.putText(frame, label, (cx + 10, cy - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)


def load_params(path: str) -> DetectParams:
    if not path:
        return DetectParams()
    p = Path(path)
    if not p.exists():
        print(f"参数文件不存在，使用默认参数：{path}", flush=True)
        return DetectParams()
    data = json.loads(p.read_text(encoding="utf-8"))
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


def save_params(path: str, params: DetectParams) -> None:
    if not path:
        path = "mole_params.json"
    payload = {
        "min_area": params.min_area,
        "max_area": params.max_area,
        "min_circularity": params.min_circularity,
        "min_solidity": params.min_solidity,
        "max_aspect_ratio": params.max_aspect_ratio,
        "blackhat_ksize": params.blackhat_ksize,
        "min_contrast": params.min_contrast,
        "skin_dilate": params.skin_dilate,
        "use_skin_mask": params.use_skin_mask,
    }
    Path(path).write_text(json.dumps(payload, ensure_ascii=False, indent=2),
                          encoding="utf-8")
    print(f"参数已保存到 {path}", flush=True)


def build_trackbars(window: str, params: DetectParams) -> None:
    cv2.createTrackbar("min_area", window, params.min_area, 2000, lambda v: None)
    cv2.createTrackbar("max_area", window, params.max_area, 5000, lambda v: None)
    cv2.createTrackbar("circ x100", window, int(params.min_circularity * 100), 100, lambda v: None)
    cv2.createTrackbar("solid x100", window, int(params.min_solidity * 100), 100, lambda v: None)
    cv2.createTrackbar("aspect x10", window, int(params.max_aspect_ratio * 10), 60, lambda v: None)
    cv2.createTrackbar("bh_ksize", window, params.blackhat_ksize, 81, lambda v: None)
    cv2.createTrackbar("contrast", window, params.min_contrast, 120, lambda v: None)
    cv2.createTrackbar("skin_dilate", window, params.skin_dilate, 51, lambda v: None)
    cv2.createTrackbar("skin_mask", window, 1 if params.use_skin_mask else 0, 1, lambda v: None)


def read_trackbars(window: str) -> DetectParams:
    bh = cv2.getTrackbarPos("bh_ksize", window)
    if bh < 3:
        bh = 3
    if bh % 2 == 0:
        bh += 1
    skin_dilate = cv2.getTrackbarPos("skin_dilate", window)
    if skin_dilate < 1:
        skin_dilate = 1
    if skin_dilate % 2 == 0:
        skin_dilate += 1
    return DetectParams(
        min_area=max(1, cv2.getTrackbarPos("min_area", window)),
        max_area=max(10, cv2.getTrackbarPos("max_area", window)),
        min_circularity=cv2.getTrackbarPos("circ x100", window) / 100.0,
        min_solidity=cv2.getTrackbarPos("solid x100", window) / 100.0,
        max_aspect_ratio=max(1.0, cv2.getTrackbarPos("aspect x10", window) / 10.0),
        blackhat_ksize=bh,
        min_contrast=cv2.getTrackbarPos("contrast", window),
        skin_dilate=skin_dilate,
        use_skin_mask=bool(cv2.getTrackbarPos("skin_mask", window)),
    )


def main() -> int:
    args = parse_args()

    cap, label = open_camera(args.camera, args.width, args.height, args.fps)
    print(f"摄像头已打开: {label}", flush=True)

    undistorter = None
    if args.calib_params:
        undistorter = Undistorter.load(args.calib_params, (args.width, args.height))
        if undistorter:
            print("去畸变已启用", flush=True)

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
        except Exception as e:
            print(f"串口打开失败: {e}", flush=True)

    params = load_params(args.params_json)
    print(
        f"检测参数: area=[{params.min_area},{params.max_area}] "
        f"circ>={params.min_circularity:.2f} solid>={params.min_solidity:.2f} "
        f"aspect<={params.max_aspect_ratio:.1f} bh={params.blackhat_ksize} "
        f"contrast>={params.min_contrast} skin_mask={params.use_skin_mask}",
        flush=True,
    )

    win_name = "Mole Detection"
    if args.display:
        cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)
        if args.tune:
            build_trackbars(win_name, params)

    count = 0
    start = time.time()
    last_print = 0.0
    last_serial_error_print = 0.0

    try:
        while True:
            ok, frame = cap.read()
            if not ok or frame is None:
                time.sleep(0.01)
                continue

            frame = apply_flip(frame, args.flip)
            if undistorter is not None:
                frame = undistorter.apply(frame)

            if args.tune and args.display:
                params = read_trackbars(win_name)

            moles = detect_moles(frame, params=params)
            now = time.time()

            if sender:
                for status in sender.poll_statuses():
                    print(
                        f"RA6M5 状态: {status.label} "
                        f"func=0x{status.func:02X} value=0x{status.value:02X}",
                        flush=True,
                    )
                if moles:
                    primary = moles[0]
                    sent = sender.send_target_error(
                        primary.center[0],
                        primary.center[1],
                        frame.shape[1],
                        frame.shape[0],
                    )
                    if not sent and now - last_serial_error_print >= args.print_interval:
                        last_serial_error_print = now
                        print(f"串口发送失败: {sender.last_error}", flush=True)
                # The archived legacy frame has no valid=0 representation;
                # loss of target remains silent in this isolated legacy tool.

            count += 1
            fps = count / max(1e-6, now - start)

            if now - last_print >= args.print_interval:
                last_print = now
                if moles:
                    info = " | ".join(f"({m.center[0]},{m.center[1]})" for m in moles[:5])
                    print(f"fps={fps:.1f} 检测到{len(moles)}个痣: {info}", flush=True)
                else:
                    print(f"fps={fps:.1f} 未检测到黑痣", flush=True)

            if args.display:
                draw_moles(frame, moles)
                cv2.putText(frame, f"FPS {fps:.1f} | Moles: {len(moles)}",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                if args.tune:
                    cv2.putText(frame, "tune mode (s save / q quit)",
                                (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                cv2.imshow(win_name, frame)
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q") or key == 27:
                    break
                if key == ord("s") and args.tune:
                    save_params(args.params_json or "mole_params.json", params)
    except KeyboardInterrupt:
        print("\n已停止", flush=True)
    finally:
        cap.release()
        if sender:
            sender.close()
        if args.display:
            cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
