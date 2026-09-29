"""相机内参 + 畸变标定脚本。

用法：
    python3 -m competition_app.calib_solve --images calib_images --pattern 6x9 --square-mm 20

读取 calib_images/calib_*.png, 找出每张棋盘格内角点, 跑 cv2.calibrateCamera,
打印每张图的重投影误差 (RMS), 输出最终参数到 calib_params.npz。

关键输出：
    camera_matrix (3x3)         — 内参矩阵 [fx, 0, cx; 0, fy, cy; 0, 0, 1]
    dist_coeffs   (1x5 / 1x8)   — 畸变系数 [k1, k2, p1, p2, k3, ...]
    rms                          — 全局重投影误差 (像素), <0.5 优秀, <1.0 可用, >1.5 差

每张图的单帧误差列出来, 误差 > 1.5px 的图建议删了重拍。
"""
import argparse
import sys
from pathlib import Path
from typing import List, Optional, Tuple


def parse_pattern(s: str) -> Tuple[int, int]:
    parts = s.lower().split("x")
    if len(parts) != 2:
        raise argparse.ArgumentTypeError("pattern must be like 6x9")
    return (int(parts[0]), int(parts[1]))


def main() -> int:
    parser = argparse.ArgumentParser(description="Solve camera intrinsics + distortion.")
    parser.add_argument("--images", default="calib_images")
    parser.add_argument("--pattern", type=parse_pattern, default=(6, 9),
                        help="内角点行 x 列, 默认 6x9 (= 7x10 方格棋盘)")
    parser.add_argument("--square-mm", type=float, default=20.0,
                        help="棋盘格每个方格的物理边长 (mm), 默认 20")
    parser.add_argument("--out", default="calib_params.npz")
    parser.add_argument("--max-rms-px", type=float, default=1.5,
                        help="单帧重投影误差超此值会被列为'建议剔除'")
    args = parser.parse_args()

    import cv2  # type: ignore
    import numpy as np  # type: ignore

    image_dir = Path(args.images)
    image_paths = sorted(image_dir.glob("calib_*.png"))
    if not image_paths:
        print(f"ERROR: no calib_*.png in {image_dir}", file=sys.stderr)
        return 1

    rows, cols = args.pattern
    # OpenCV 内角点排布顺序：先列后行
    pattern_size = (cols, rows)
    print(f"using pattern={rows}x{cols} (inner corners), square={args.square_mm}mm")
    print(f"loading {len(image_paths)} images from {image_dir}")

    # 棋盘格物理坐标 (Z=0 平面), 单位 mm
    # 顺序与 findChessboardCorners 返回顺序一致：(0,0), (1,0)... (cols-1, 0), (0,1)...
    objp = np.zeros((rows * cols, 3), dtype=np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    objp *= float(args.square_mm)

    obj_points: list = []
    img_points: list = []
    used_paths = []  # type: List[Path]
    img_size = None  # type: Optional[Tuple[int, int]]

    detect_flags = (
        cv2.CALIB_CB_ADAPTIVE_THRESH
        + cv2.CALIB_CB_NORMALIZE_IMAGE
        + cv2.CALIB_CB_FAST_CHECK
    )
    subpix_criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01)

    for path in image_paths:
        img = cv2.imread(str(path))
        if img is None:
            print(f"  {path.name}: SKIP (cannot read)")
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        if img_size is None:
            img_size = (gray.shape[1], gray.shape[0])

        found, corners = cv2.findChessboardCorners(gray, pattern_size, flags=detect_flags)
        if not found:
            print(f"  {path.name}: SKIP (no pattern found)")
            continue
        # 亚像素细化, 提高角点定位精度到 0.01 像素
        corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), subpix_criteria)
        obj_points.append(objp.copy())
        img_points.append(corners_refined)
        used_paths.append(path)
        print(f"  {path.name}: OK ({len(corners_refined)} corners)")

    if len(obj_points) < 5:
        print(f"\nERROR: only {len(obj_points)} valid images, need >= 5", file=sys.stderr)
        return 2

    print(f"\ncalibrating with {len(obj_points)} images, image size = {img_size}")
    rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(
        obj_points, img_points, img_size, None, None
    )
    print(f"\nGLOBAL RMS reprojection error = {rms:.4f} px")
    if rms < 0.5:
        print("  -> 优秀")
    elif rms < 1.0:
        print("  -> 可用, 推荐继续")
    elif rms < 1.5:
        print("  -> 边界, 看单帧误差表决定要不要重拍")
    else:
        print("  -> 差, 建议剔除高误差帧后重标")

    # 算每张图单独的重投影误差
    print(f"\nper-image RMS (threshold {args.max_rms_px:.2f} px):")
    bad = []  # type: List[Path]
    for i, path in enumerate(used_paths):
        proj, _ = cv2.projectPoints(obj_points[i], rvecs[i], tvecs[i], K, dist)
        proj = proj.reshape(-1, 2)
        err = np.linalg.norm(img_points[i].reshape(-1, 2) - proj, axis=1)
        per_rms = float(np.sqrt(np.mean(err ** 2)))
        flag = "  " if per_rms <= args.max_rms_px else "!!"
        print(f"  {flag} {path.name}: {per_rms:.3f} px")
        if per_rms > args.max_rms_px:
            bad.append(path)

    print("\ncamera_matrix K =")
    print(K)
    print(f"\ndist_coeffs = {dist.ravel()}")
    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]
    print(f"\n  fx = {fx:.2f}  fy = {fy:.2f}")
    print(f"  cx = {cx:.2f}  cy = {cy:.2f}")
    print(f"  画面中心理论 = ({img_size[0]/2:.1f}, {img_size[1]/2:.1f})")
    if abs(cx - img_size[0] / 2) > 30 or abs(cy - img_size[1] / 2) > 30:
        print("  WARN: 主点偏离画面中心 > 30px, 通常意味着标定数据不够 (画面四角覆盖不够)")

    out_path = Path(args.out)
    np.savez(
        out_path,
        camera_matrix=K,
        dist_coeffs=dist,
        image_size=np.array(img_size, dtype=np.int32),
        rms=rms,
    )
    print(f"\nsaved to {out_path}")

    if bad:
        print(f"\n建议处理 (单帧误差 > {args.max_rms_px}):")
        for p in bad:
            print(f"  rm {p}")
        print("删后重跑此脚本即可。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
