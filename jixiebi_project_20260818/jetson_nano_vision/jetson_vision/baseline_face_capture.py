"""Capture no-mole face baseline frames for robust mole detection.

Usage:
    DISPLAY=:0 python3 -m jetson_vision.baseline_face_capture --view front

Keys:
    Space / s  Save current raw frame
    Left click  Save current raw frame
    Terminal Enter  Save current raw frame
    q / Esc    Quit
"""

import argparse
import json
import select
import sys
import time
from datetime import datetime
from pathlib import Path

from .camera_utils import apply_flip, open_camera


DEFAULT_CAMERA = "/dev/v4l/by-id/usb-Generic_USB_Camera_YHTek-video-index0"
DEFAULT_OUT_DIR = "/home/jetson/jixiebi/calibration/baseline_face/raw"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="无黑痣人脸基准图采集工具：预览窗口中按空格保存"
    )
    parser.add_argument("--camera", default=DEFAULT_CAMERA)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--flip", choices=["none", "0", "1", "-1"], default="none")
    parser.add_argument("--brightness", type=float, default=0.0)
    parser.add_argument("--gain", type=float, default=64.0)
    parser.add_argument("--backlight-compensation", type=float, default=0.0)
    parser.add_argument(
        "--view",
        choices=["left", "front", "right", "scan", "other"],
        default="front",
        help="当前采集视角；会作为保存子文件夹名",
    )
    parser.add_argument(
        "--out-dir",
        default=DEFAULT_OUT_DIR,
        help="基准图根目录；实际保存到 out-dir/view/",
    )
    parser.add_argument(
        "--target-count",
        type=int,
        default=40,
        help="建议采集张数，只用于窗口提示；达到后仍可继续拍",
    )
    parser.add_argument("--prefix", default="baseline")
    parser.add_argument(
        "--stdin-trigger",
        type=int,
        choices=[0, 1],
        default=1,
        help="1=允许在启动脚本的 SSH/终端里按 Enter 保存一张",
    )
    return parser.parse_args()


def _next_index(directory: Path, prefix: str, view: str) -> int:
    pattern = "{}_{}_[0-9][0-9][0-9].png".format(prefix, view)
    indices = []
    for path in directory.glob(pattern):
        stem = path.stem
        try:
            indices.append(int(stem.rsplit("_", 1)[-1]))
        except ValueError:
            continue
    return max(indices, default=0) + 1


def _draw_overlay(frame, *, view: str, saved: int, target_count: int):
    import cv2  # type: ignore

    shown = frame.copy()
    lines = [
        "NO-MOLE BASELINE CAPTURE",
        "view={}  saved={}/{}".format(view, saved, target_count),
        "SPACE/s/click=save   q/Esc=quit",
        "SSH terminal Enter also saves",
        "Please remove mole sticker before capture",
    ]
    y = 28
    for line in lines:
        cv2.putText(
            shown,
            line,
            (12, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.62,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )
        y += 28
    return shown


def _write_metadata(
    directory: Path,
    *,
    args: argparse.Namespace,
    camera_label: str,
    saved_count: int,
) -> None:
    meta_path = directory / "metadata.json"
    payload = {
        "updated_at": datetime.now().isoformat(timespec="seconds"),
        "purpose": "no_mole_face_baseline",
        "view": args.view,
        "camera": args.camera,
        "camera_label": camera_label,
        "resolution": [int(args.width), int(args.height)],
        "fps": int(args.fps),
        "flip": args.flip,
        "brightness": float(args.brightness),
        "gain": float(args.gain),
        "backlight_compensation": float(args.backlight_compensation),
        "saved_count_this_run": int(saved_count),
    }
    meta_path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )


def main() -> int:
    args = parse_args()
    import cv2  # type: ignore

    save_dir = Path(args.out_dir) / args.view
    save_dir.mkdir(parents=True, exist_ok=True)
    next_index = _next_index(save_dir, args.prefix, args.view)

    cap, camera_label = open_camera(
        args.camera,
        args.width,
        args.height,
        args.fps,
        brightness=args.brightness,
        gain=args.gain,
        backlight_compensation=args.backlight_compensation,
    )
    print("摄像头已打开: {}".format(camera_label), flush=True)
    print("保存目录: {}".format(save_dir), flush=True)
    print(
        "按 空格/s、触屏点击窗口、或在 SSH 终端按 Enter 保存；按 q 或 Esc 退出。",
        flush=True,
    )

    saved = 0
    window = "baseline face capture - space/save, q/quit"
    mouse_save_requested = False

    def on_mouse(event, _x, _y, _flags, _param):
        nonlocal mouse_save_requested
        if event == cv2.EVENT_LBUTTONDOWN:
            mouse_save_requested = True

    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(window, on_mouse)
    try:
        while True:
            ok, frame = cap.read()
            if not ok or frame is None:
                time.sleep(0.01)
                continue
            frame = apply_flip(frame, args.flip)
            shown = _draw_overlay(
                frame,
                view=args.view,
                saved=saved,
                target_count=max(1, int(args.target_count)),
            )
            cv2.imshow(window, shown)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            stdin_save_requested = False
            if args.stdin_trigger and select.select([sys.stdin], [], [], 0.0)[0]:
                line = sys.stdin.readline()
                stdin_save_requested = line != ""
            if (
                key in (ord(" "), ord("s"))
                or mouse_save_requested
                or stdin_save_requested
            ):
                mouse_save_requested = False
                path = save_dir / "{}_{}_{:03d}.png".format(
                    args.prefix,
                    args.view,
                    next_index,
                )
                if cv2.imwrite(str(path), frame):
                    saved += 1
                    next_index += 1
                    print("已保存: {}".format(path), flush=True)
                else:
                    print("保存失败: {}".format(path), flush=True)
    finally:
        cap.release()
        cv2.destroyAllWindows()
        _write_metadata(
            save_dir,
            args=args,
            camera_label=camera_label,
            saved_count=saved,
        )

    print("本次保存 {} 张。".format(saved), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
