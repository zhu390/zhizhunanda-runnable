"""单张图片离线调参工具：在一张人脸照片上拖滑动条调整黑痣检测参数。

用法：
    python -m jetson_vision.mole_image_test --image path/to/face.jpg

按键：
    s  保存当前参数到 mole_params.json
    n  在 [原图+检测] / [肤色掩膜] / [blackhat] / [二值图] 之间循环切换
    q / Esc  退出
"""
import argparse
import json
from pathlib import Path
from typing import List

import cv2
import numpy as np

from .vision.mole_detect import DetectParams, DetectedMole, detect_moles


VIEW_NAMES = ["overlay", "skin", "blackhat", "binary"]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="单图黑痣检测调参工具")
    parser.add_argument("--image", required=True, help="待测试图片路径")
    parser.add_argument("--max-side", type=int, default=900,
                        help="显示前若长边超过此值会按比例缩放，避免开窗过大")
    parser.add_argument("--out-params", default="mole_params.json",
                        help="按 s 保存参数的目标文件")
    return parser.parse_args()


def load_image(path: str, max_side: int) -> np.ndarray:
    img = cv2.imread(path, cv2.IMREAD_COLOR)
    if img is None:
        raise SystemExit(f"无法读取图片：{path}")
    h, w = img.shape[:2]
    long_side = max(h, w)
    if long_side > max_side:
        scale = max_side / long_side
        img = cv2.resize(img, (int(w * scale), int(h * scale)),
                         interpolation=cv2.INTER_AREA)
    return img


def make_default_params() -> DetectParams:
    return DetectParams()


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


def draw_moles(frame: np.ndarray, moles: List[DetectedMole]) -> None:
    for i, mole in enumerate(moles):
        cx, cy = mole.center
        cv2.circle(frame, (cx, cy), mole.radius + 4, (0, 255, 0), 2)
        cv2.circle(frame, (cx, cy), 2, (0, 0, 255), -1)
        label = f"#{i+1} c={mole.contrast:.0f} a={int(mole.area)}"
        cv2.putText(frame, label, (cx + 8, cy - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1, cv2.LINE_AA)


def to_bgr(gray: np.ndarray) -> np.ndarray:
    return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)


def save_params(path: str, params: DetectParams, image_path: str) -> None:
    payload = {
        "image": image_path,
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


def main() -> int:
    args = parse_args()
    img = load_image(args.image, args.max_side)
    print(f"加载图片：{args.image}  尺寸 {img.shape[1]}x{img.shape[0]}", flush=True)

    win = "mole tuner"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win, img.shape[1], img.shape[0] + 40)

    build_trackbars(win, make_default_params())
    view_idx = 0

    while True:
        params = read_trackbars(win)
        moles, dbg = detect_moles(img, params=params, return_debug=True)

        view = VIEW_NAMES[view_idx]
        if view == "overlay":
            display = img.copy()
            draw_moles(display, moles)
        elif view == "skin":
            display = to_bgr(dbg["skin_eroded"])
        elif view == "blackhat":
            bh = dbg["blackhat"]
            bh_norm = cv2.normalize(bh, None, 0, 255, cv2.NORM_MINMAX)
            display = to_bgr(bh_norm.astype(np.uint8))
        else:
            display = to_bgr(dbg["binary"])

        header = f"view={view}  moles={len(moles)}  (s save / n next view / q quit)"
        cv2.putText(display, header, (8, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1, cv2.LINE_AA)

        cv2.imshow(win, display)
        key = cv2.waitKey(20) & 0xFF
        if key == ord("q") or key == 27:
            break
        if key == ord("s"):
            save_params(args.out_params, params, args.image)
        if key == ord("n"):
            view_idx = (view_idx + 1) % len(VIEW_NAMES)

    cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
