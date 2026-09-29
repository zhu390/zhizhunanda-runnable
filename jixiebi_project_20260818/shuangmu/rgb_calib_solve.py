import argparse
from pathlib import Path
from typing import List, Optional, Tuple

import cv2
import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_ASTRA_RGB_CALIB_IMAGES = PROJECT_ROOT / "calibration" / "astra_rgb" / "images"
DEFAULT_ASTRA_RGB_CALIB_PARAMS = PROJECT_ROOT / "calibration" / "astra_rgb" / "rgb_calib_params.npz"


def parse_pattern(value: str) -> Tuple[int, int]:
    parts = value.lower().split("x")
    if len(parts) != 2:
        raise argparse.ArgumentTypeError("pattern must be like 6x9")
    return int(parts[0]), int(parts[1])


def main() -> int:
    parser = argparse.ArgumentParser(description="Solve RGB camera intrinsics and distortion from chessboard images.")
    parser.add_argument("--images", default=str(DEFAULT_ASTRA_RGB_CALIB_IMAGES))
    parser.add_argument("--pattern", type=parse_pattern, default=(6, 9), help="Inner corners, rows x cols. Default: 6x9.")
    parser.add_argument("--square-mm", type=float, default=20.0)
    parser.add_argument("--out", default=str(DEFAULT_ASTRA_RGB_CALIB_PARAMS))
    parser.add_argument("--max-rms-px", type=float, default=1.5)
    args = parser.parse_args()

    image_dir = Path(args.images)
    image_paths = sorted(image_dir.glob("calib_*.png"))
    if not image_paths:
        print("ERROR: no calib_*.png in {}".format(image_dir))
        return 1

    rows, cols = args.pattern
    pattern_size = (cols, rows)
    print("using pattern={}x{} inner corners, square={} mm".format(rows, cols, args.square_mm))
    print("loading {} images from {}".format(len(image_paths), image_dir))

    objp = np.zeros((rows * cols, 3), dtype=np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    objp *= float(args.square_mm)

    obj_points = []
    img_points = []
    used_paths = []  # type: List[Path]
    img_size = None  # type: Optional[Tuple[int, int]]

    flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01)

    for path in image_paths:
        image = cv2.imread(str(path))
        if image is None:
            print("  {}: SKIP cannot read".format(path.name))
            continue
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        if img_size is None:
            img_size = (gray.shape[1], gray.shape[0])

        found, corners = cv2.findChessboardCorners(gray, pattern_size, flags=flags)
        if not found:
            print("  {}: SKIP no chessboard".format(path.name))
            continue

        refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        obj_points.append(objp.copy())
        img_points.append(refined)
        used_paths.append(path)
        print("  {}: OK {} corners".format(path.name, len(refined)))

    if len(obj_points) < 8:
        print("ERROR: only {} valid images, need at least 8, recommend 20".format(len(obj_points)))
        return 2

    print("")
    print("calibrating with {} images, image size={}".format(len(obj_points), img_size))
    rms, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        obj_points, img_points, img_size, None, None
    )

    print("")
    print("GLOBAL RMS reprojection error = {:.4f} px".format(rms))
    if rms < 0.5:
        print("quality: excellent")
    elif rms < 1.0:
        print("quality: usable")
    elif rms < 1.5:
        print("quality: borderline, check per-image errors")
    else:
        print("quality: poor, delete bad images and reshoot")

    print("")
    print("per-image RMS:")
    bad = []
    for index, path in enumerate(used_paths):
        projected, _ = cv2.projectPoints(obj_points[index], rvecs[index], tvecs[index], camera_matrix, dist_coeffs)
        projected = projected.reshape(-1, 2)
        measured = img_points[index].reshape(-1, 2)
        err = np.linalg.norm(measured - projected, axis=1)
        per_rms = float(np.sqrt(np.mean(err ** 2)))
        mark = "OK"
        if per_rms > args.max_rms_px:
            mark = "BAD"
            bad.append(path)
        print("  {}  {}: {:.3f} px".format(mark, path.name, per_rms))

    print("")
    print("camera_matrix =")
    print(camera_matrix)
    print("")
    print("dist_coeffs =")
    print(dist_coeffs.ravel())

    np.savez(
        args.out,
        camera_matrix=camera_matrix,
        dist_coeffs=dist_coeffs,
        image_size=np.array(img_size, dtype=np.int32),
        rms=np.array(rms, dtype=np.float32),
    )
    print("")
    print("saved to {}".format(args.out))

    if bad:
        print("")
        print("suggest deleting these high-error images and running solve again:")
        for path in bad:
            print("  rm {}".format(path))

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
