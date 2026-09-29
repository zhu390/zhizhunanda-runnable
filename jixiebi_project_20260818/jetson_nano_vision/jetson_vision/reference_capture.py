"""采集无黑痣人脸基准图。

程序在相机预热后连续采集多帧，对每个像素取中值，生成低噪声基准图。
显示模式下按 SPACE 开始采集，按 Q/ESC 退出。

可选串口控制仅用于隔离的旧协议台架调试，不属于正式 Qt 流程。
"""
import argparse
import hashlib
import json
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional

from .camera_utils import apply_flip, open_camera
from .serial_sender import SerialSender
from .undistort import Undistorter
from .vision_protocol import RA6_TO_JETSON_ERROR, RA6_TO_JETSON_READY


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="采集无黑痣人脸多帧中值基准图")
    parser.add_argument("--camera", default="0")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--flip", choices=["none", "0", "1", "-1"], default="none")
    parser.add_argument("--calib-params", default="")
    parser.add_argument("--frames", type=int, default=30,
                        help="用于生成中值图的帧数，默认 30")
    parser.add_argument("--warmup", type=int, default=30,
                        help="开始前丢弃的预热帧数，默认 30")
    parser.add_argument("--display", type=int, choices=[0, 1], default=1,
                        help="1=按 SPACE 开始采集；0=预热后自动采集")
    parser.add_argument("--out", default="reference/clean_face.png")
    parser.add_argument("--meta-out", default="reference/reference_meta.json")
    parser.add_argument(
        "--serial-port",
        default="",
        help="Isolated legacy bench port; leave empty for formal Qt operation.",
    )
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--start-target-ctrl", type=int, choices=[0, 1], default=0,
                        help="Send AA 01 01 BB before capturing the reference.")
    parser.add_argument("--stop-target-ctrl-on-exit", type=int, choices=[0, 1], default=0,
                        help="Send AA 01 00 BB before exit when target ctrl was started.")
    parser.add_argument("--wait-ready", type=int, choices=[0, 1], default=1,
                        help="Wait for RA6M5 READY after starting target ctrl.")
    parser.add_argument("--ready-timeout-ms", type=int, default=30000,
                        help="Timeout for waiting READY. 0 disables timeout.")
    return parser.parse_args()


def open_serial(args: argparse.Namespace) -> Optional[SerialSender]:
    if not args.serial_port:
        return None
    try:
        sender = SerialSender(
            args.serial_port,
            args.baudrate,
            protocol="legacy",
            auto_heartbeat=False,
        )
    except Exception as exc:
        print("serial open failed: {}".format(exc), flush=True)
        return None
    print(
        "WARNING: isolated LEGACY serial debug only; do not run the formal Qt serial session",
        flush=True,
    )
    print(
        "serial opened: {} @ {} protocol=legacy".format(args.serial_port, args.baudrate),
        flush=True,
    )
    return sender


def wait_ready(sender: SerialSender, timeout_ms: int) -> bool:
    start = time.monotonic()
    last_print = start
    print("waiting RA6M5 READY before reference capture", flush=True)
    while True:
        for status in sender.poll_statuses():
            if status.func == RA6_TO_JETSON_READY and status.value == 0x01:
                print("READY received: reference capture can start", flush=True)
                return True
            if status.func == RA6_TO_JETSON_ERROR and status.value == 0x01:
                print("RA6M5 ERROR received: reference capture aborted", flush=True)
                return False

        if timeout_ms > 0 and (time.monotonic() - start) * 1000.0 >= timeout_ms:
            print("READY timeout: reference capture aborted", flush=True)
            return False
        now = time.monotonic()
        if now - last_print >= 1.0:
            last_print = now
            elapsed_ms = int((now - start) * 1000.0)
            print("still waiting READY... {} ms".format(elapsed_ms), flush=True)
        time.sleep(0.02)


def calibration_sha256(path: str) -> Optional[str]:
    if not path:
        return None
    p = Path(path)
    if not p.exists():
        return None
    digest = hashlib.sha256()
    with p.open("rb") as file:
        for chunk in iter(lambda: file.read(65536), b""):
            digest.update(chunk)
    return digest.hexdigest()


def save_median_reference(frames, out_path: Path):
    import cv2  # type: ignore
    import numpy as np  # type: ignore

    stack = np.stack(frames, axis=0)
    reference = np.median(stack, axis=0).astype(np.uint8)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(out_path), reference):
        raise RuntimeError(f"无法写入基准图: {out_path}")
    return reference


def main() -> int:
    args = parse_args()
    if args.frames < 3:
        print("ERROR: --frames 至少为 3", flush=True)
        return 2

    sender = open_serial(args)
    if args.serial_port and sender is None:
        return 2
    target_ctrl_started = False
    if sender is not None and args.start_target_ctrl:
        target_ctrl_started = sender.send_target_ctrl(True)
        if not target_ctrl_started:
            print("target ctrl start failed: {}".format(sender.last_error), flush=True)
            sender.close()
            return 2
        if args.wait_ready and not wait_ready(sender, args.ready_timeout_ms):
            if args.stop_target_ctrl_on_exit:
                sender.send_target_ctrl(False)
            sender.close()
            return 2

    import cv2  # type: ignore

    cap, label = open_camera(args.camera, args.width, args.height, args.fps)
    print(f"摄像头已打开: {label}", flush=True)

    undistorter = None
    if args.calib_params:
        undistorter = Undistorter.load(args.calib_params, (args.width, args.height))
        if undistorter is None:
            print(f"ERROR: 无法加载标定参数 {args.calib_params}", flush=True)
            cap.release()
            if sender is not None:
                if target_ctrl_started and args.stop_target_ctrl_on_exit:
                    sender.send_target_ctrl(False)
                sender.close()
            return 2
        print("去畸变已启用", flush=True)

    out_path = Path(args.out)
    meta_path = Path(args.meta_out)
    frames = []
    capturing = not bool(args.display)
    reference = None
    win = "clean face reference (SPACE=start, Q=quit)"

    try:
        if args.warmup > 0:
            print("camera warmup: dropping {} frames".format(args.warmup), flush=True)
        for _ in range(max(0, args.warmup)):
            cap.read()
            time.sleep(0.01)
        if args.warmup > 0:
            print("camera warmup done", flush=True)

        if args.display:
            cv2.namedWindow(win, cv2.WINDOW_NORMAL)
        else:
            print("display disabled: auto capturing {} frames".format(args.frames), flush=True)

        while True:
            ok, frame = cap.read()
            if not ok or frame is None:
                time.sleep(0.01)
                continue

            frame = apply_flip(frame, args.flip)
            if undistorter is not None:
                frame = undistorter.apply(frame)

            if capturing:
                frames.append(frame.copy())
                if len(frames) == 1 or len(frames) == args.frames or len(frames) % 10 == 0:
                    print("capturing reference frame {}/{}".format(len(frames), args.frames), flush=True)
                if len(frames) >= args.frames:
                    reference = save_median_reference(frames, out_path)
                    print(f"基准图已保存: {out_path} ({len(frames)} 帧中值)", flush=True)
                    break

            if args.display:
                preview = frame.copy()
                if capturing:
                    message = f"capturing {len(frames)}/{args.frames}"
                    color = (0, 255, 255)
                else:
                    message = "NO MOLE | SPACE=start | Q=quit"
                    color = (0, 255, 0)
                cv2.putText(preview, message, (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2)
                cv2.imshow(win, preview)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27):
                    return 1
                if key == 32 and not capturing:
                    frames.clear()
                    capturing = True
                    print("开始采集无黑痣基准帧", flush=True)
    finally:
        cap.release()
        if sender is not None:
            if target_ctrl_started and args.stop_target_ctrl_on_exit:
                sender.send_target_ctrl(False)
            sender.close()
        if args.display:
            cv2.destroyAllWindows()

    if reference is None:
        return 2

    height, width = reference.shape[:2]
    metadata = {
        "schema_version": 1,
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "camera": args.camera,
        "camera_label": label,
        "image_size": [width, height],
        "requested_fps": args.fps,
        "flip": args.flip,
        "frame_count": len(frames),
        "undistorted": undistorter is not None,
        "calib_params": str(Path(args.calib_params).resolve()) if args.calib_params else None,
        "calib_sha256": calibration_sha256(args.calib_params),
        "reference_image": str(out_path.resolve()),
        "serial_port": args.serial_port or None,
        "target_ctrl_started": bool(target_ctrl_started),
        "waited_ready": bool(sender is not None and args.start_target_ctrl and args.wait_ready),
    }
    meta_path.parent.mkdir(parents=True, exist_ok=True)
    meta_path.write_text(json.dumps(metadata, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"基准元数据已保存: {meta_path}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
