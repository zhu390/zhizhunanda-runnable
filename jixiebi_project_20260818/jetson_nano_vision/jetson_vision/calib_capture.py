"""棋盘格采集脚本 — 用于相机内参标定第一步。

用法：
    python3 -m competition_app.calib_capture --camera 0 --flip -1 --pattern 6x9

按键：
    SPACE  保存当前帧（仅当检测到完整棋盘格时）
    R      重拍最后一张
    Q/ESC  退出

每次成功保存会在 calib_images/ 下生成 calib_001.png ... 文件,
并在窗口右上角实时显示已保存张数。建议拍 15-20 张：
- 不同角度（左右倾斜、上下倾斜各几张）
- 不同距离（近、中、远）
- 棋盘格在画面不同位置（中央、四角各一张）
- 棋盘格保持平整,不要弯曲
- 全程靶子距离改变, 但**不要换摄像头/镜头**

完成后跑 calib_solve.py 算出畸变参数。
"""
import argparse
import sys
from pathlib import Path
from typing import Optional, Tuple

from .camera_utils import apply_flip, open_camera


def parse_pattern(s: str) -> Tuple[int, int]:
    parts = s.lower().split("x")
    if len(parts) != 2:
        raise argparse.ArgumentTypeError("pattern must be like 6x9 (rows x cols of inner corners)")
    return (int(parts[0]), int(parts[1]))


def main() -> int:
    parser = argparse.ArgumentParser(description="Capture chessboard images for camera calibration.")
    parser.add_argument("--camera", default="0")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=120)
    parser.add_argument("--flip", default="-1", help="跟比赛保持一致, 默认 -1")
    parser.add_argument("--pattern", type=parse_pattern, default=(6, 9),
                        help="棋盘内角点数, 行 x 列。默认 6x9 (= 7x10 方格的棋盘)")
    parser.add_argument("--out", default="calib_images")
    args = parser.parse_args()

    import cv2  # type: ignore

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    # 累计已存图片数（支持继续拍）
    existing = sorted(out_dir.glob("calib_*.png"))
    saved = len(existing)
    last_path = existing[-1] if existing else None  # type: Optional[Path]
    if saved > 0:
        print(f"continue from {saved} existing images in {out_dir}")

    cap, label = open_camera(args.camera, args.width, args.height, args.fps)
    rows, cols = args.pattern
    win = f"calib capture [{label}] pattern={rows}x{cols}"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)

    detect_flags = (
        cv2.CALIB_CB_ADAPTIVE_THRESH
        + cv2.CALIB_CB_NORMALIZE_IMAGE
        + cv2.CALIB_CB_FAST_CHECK
    )

    try:
        while True:
            ok, frame = cap.read()
            if not ok or frame is None:
                continue
            frame = apply_flip(frame, args.flip)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            found, corners = cv2.findChessboardCorners(gray, (cols, rows), flags=detect_flags)

            disp = frame.copy()
            if found:
                cv2.drawChessboardCorners(disp, (cols, rows), corners, found)
                color = (0, 255, 0)
                hint = "SPACE=save  R=retry  Q=quit"
            else:
                color = (0, 0, 255)
                hint = "no pattern  R=retry  Q=quit"
            cv2.putText(disp, f"saved {saved}", (10, 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2, cv2.LINE_AA)
            cv2.putText(disp, hint, (10, 56),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
            cv2.imshow(win, disp)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == 32 and found:    # SPACE
                saved += 1
                path = out_dir / f"calib_{saved:03d}.png"
                cv2.imwrite(str(path), frame)
                last_path = path
                print(f"saved {path}")
            elif key in (ord("r"), ord("R")) and last_path is not None and last_path.exists():
                last_path.unlink()
                saved -= 1
                print(f"removed {last_path}")
                last_path = None
    finally:
        cap.release()
        cv2.destroyAllWindows()

    print(f"\ndone. {saved} images in {out_dir}")
    if saved < 10:
        print("WARN: 建议至少 15 张, 太少标定误差大")
    return 0


if __name__ == "__main__":
    sys.exit(main())
