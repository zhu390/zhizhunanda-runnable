"""从摄像头抓拍一张/几张快照保存到磁盘，方便后续 mole_image_test.py 离线调参。

用法：
    python -m jetson_vision.snapshot --camera 0 --out face.jpg
    # 显示窗口模式下按空格连拍，按 q 退出
    python -m jetson_vision.snapshot --camera 0 --out face --display 1
"""
import argparse
import time
from pathlib import Path

from .camera_utils import apply_flip, open_camera
from .undistort import Undistorter


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="单帧/连续抓拍快照工具")
    parser.add_argument("--camera", default="0")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--flip", choices=["none", "0", "1", "-1"], default="none")
    parser.add_argument("--calib-params", default="")
    parser.add_argument("--out", default="snapshot.jpg",
                        help="单帧模式：直接当作输出路径；连拍模式：作为前缀（自动加 _001.jpg 等）")
    parser.add_argument("--display", type=int, choices=[0, 1], default=0,
                        help="1=开窗显示，按空格抓拍，q/Esc 退出；0=直接抓一帧后退出")
    parser.add_argument("--warmup", type=int, default=10,
                        help="非显示模式下，在抓帧前先丢掉前几帧让相机自动曝光稳定")
    return parser.parse_args()


def make_output_path(out: str, index: int, multi: bool) -> Path:
    p = Path(out)
    if not multi:
        return p
    if p.suffix:
        return p.with_name(f"{p.stem}_{index:03d}{p.suffix}")
    return p.with_name(f"{p.name}_{index:03d}.jpg")


def main() -> int:
    args = parse_args()
    import cv2  # type: ignore

    cap, label = open_camera(args.camera, args.width, args.height, args.fps)
    print(f"摄像头已打开: {label}", flush=True)

    undistorter = None
    if args.calib_params:
        undistorter = Undistorter.load(args.calib_params, (args.width, args.height))
        if undistorter:
            print("去畸变已启用", flush=True)

    saved = 0
    try:
        if args.display:
            win = "snapshot (space=save, q=quit)"
            cv2.namedWindow(win, cv2.WINDOW_NORMAL)
            while True:
                ok, frame = cap.read()
                if not ok or frame is None:
                    time.sleep(0.01)
                    continue
                frame = apply_flip(frame, args.flip)
                if undistorter is not None:
                    frame = undistorter.apply(frame)

                preview = frame.copy()
                cv2.putText(preview, f"saved={saved}", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.imshow(win, preview)
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q") or key == 27:
                    break
                if key == ord(" "):
                    out_path = make_output_path(args.out, saved + 1, multi=True)
                    cv2.imwrite(str(out_path), frame)
                    saved += 1
                    print(f"已保存: {out_path}", flush=True)
            cv2.destroyAllWindows()
        else:
            for _ in range(max(0, args.warmup)):
                cap.read()
                time.sleep(0.03)
            ok, frame = cap.read()
            if not ok or frame is None:
                print("抓帧失败", flush=True)
                return 2
            frame = apply_flip(frame, args.flip)
            if undistorter is not None:
                frame = undistorter.apply(frame)
            out_path = Path(args.out)
            cv2.imwrite(str(out_path), frame)
            saved = 1
            print(f"已保存: {out_path}", flush=True)
    finally:
        cap.release()

    return 0 if saved > 0 else 2


if __name__ == "__main__":
    raise SystemExit(main())
