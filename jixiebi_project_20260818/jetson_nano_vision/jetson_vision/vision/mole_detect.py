from typing import List, Optional, Tuple

import cv2
import numpy as np

from .._compat import dataclass


@dataclass(frozen=True)
class DetectedMole:
    center: Tuple[int, int]
    radius: int
    area: float
    circularity: float
    contrast: float
    # Tracking metadata is populated by CandidateStabilityTracker.  Raw detector
    # results keep the neutral defaults so existing positional construction and
    # callers that only need the five detection fields remain compatible.
    track_id: int = 0
    stable_count: int = 0
    required_count: int = 0
    track_age_s: float = 0.0


@dataclass(frozen=True)
class DetectParams:
    min_area: int = 8
    max_area: int = 600
    min_circularity: float = 0.55
    min_solidity: float = 0.80
    max_aspect_ratio: float = 2.2
    blackhat_ksize: int = 21
    min_contrast: int = 18
    skin_dilate: int = 9
    use_skin_mask: bool = True

    def max_area_diameter(self) -> int:
        diameter = int(2 * np.sqrt(max(self.max_area, 1) / np.pi)) + 4
        return max(diameter, 7)


def build_skin_mask(frame_bgr: np.ndarray) -> np.ndarray:
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    ycrcb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2YCrCb)

    mask_hsv = cv2.inRange(hsv, (0, 20, 70), (25, 180, 255))
    mask_ycrcb = cv2.inRange(ycrcb, (50, 130, 80), (255, 180, 130))
    mask = cv2.bitwise_and(mask_hsv, mask_ycrcb)

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    return mask


def detect_moles(
    frame: np.ndarray,
    params: Optional[DetectParams] = None,
    roi: Optional[Tuple[int, int, int, int]] = None,
    valid_mask: Optional[np.ndarray] = None,
    return_debug: bool = False,
):
    if params is None:
        params = DetectParams()

    if roi is not None:
        rx, ry, rw, rh = roi
        work = frame[ry:ry + rh, rx:rx + rw]
        work_valid_mask = (
            valid_mask[ry:ry + rh, rx:rx + rw]
            if valid_mask is not None
            else None
        )
    else:
        work = frame
        rx, ry = 0, 0
        work_valid_mask = valid_mask

    if work.size == 0:
        return ([], {}) if return_debug else []
    if work_valid_mask is not None:
        if work_valid_mask.shape[:2] != work.shape[:2]:
            raise ValueError(
                "valid_mask shape {} does not match image shape {}".format(
                    work_valid_mask.shape[:2], work.shape[:2]
                )
            )
        if work_valid_mask.ndim == 3:
            work_valid_mask = cv2.cvtColor(work_valid_mask, cv2.COLOR_BGR2GRAY)
        work_valid_mask = np.where(work_valid_mask > 0, 255, 0).astype(np.uint8)

    gray = cv2.cvtColor(work, cv2.COLOR_BGR2GRAY)

    if params.use_skin_mask:
        skin = build_skin_mask(work)
        fill_k = max(3, params.max_area_diameter() | 1)
        fill_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (fill_k, fill_k))
        skin_filled = cv2.morphologyEx(skin, cv2.MORPH_CLOSE, fill_kernel, iterations=1)
        kernel_skin = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (params.skin_dilate, params.skin_dilate)
        )
        skin_eroded = cv2.erode(skin_filled, kernel_skin, iterations=1)
    else:
        skin = np.full(gray.shape, 255, dtype=np.uint8)
        skin_eroded = skin

    if work_valid_mask is not None:
        skin_eroded = cv2.bitwise_and(skin_eroded, work_valid_mask)

    bh_k = max(3, params.blackhat_ksize | 1)
    bh_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (bh_k, bh_k))
    blackhat = cv2.morphologyEx(gray, cv2.MORPH_BLACKHAT, bh_kernel)
    blackhat = cv2.bitwise_and(blackhat, blackhat, mask=skin_eroded)

    _, binary = cv2.threshold(blackhat, params.min_contrast, 255, cv2.THRESH_BINARY)
    morph_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, morph_kernel, iterations=1)
    binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, morph_kernel, iterations=1)

    contours = _find_external_contours(binary)

    moles = []  # type: List[DetectedMole]
    for cnt in contours:
        area = float(cv2.contourArea(cnt))
        if area < params.min_area or area > params.max_area:
            continue

        perimeter = float(cv2.arcLength(cnt, True))
        if perimeter <= 0:
            continue
        circularity = 4.0 * np.pi * area / (perimeter * perimeter)
        if circularity < params.min_circularity:
            continue

        hull = cv2.convexHull(cnt)
        hull_area = float(cv2.contourArea(hull))
        solidity = area / hull_area if hull_area > 0 else 0.0
        if solidity < params.min_solidity:
            continue

        rect = cv2.minAreaRect(cnt)
        (_, _), (rw_, rh_), _ = rect
        long_side = max(rw_, rh_)
        short_side = max(min(rw_, rh_), 1.0)
        aspect = long_side / short_side
        if aspect > params.max_aspect_ratio:
            continue

        (cx, cy), radius = cv2.minEnclosingCircle(cnt)
        cx_i = int(cx)
        cy_i = int(cy)
        radius_i = max(int(radius), 1)

        contrast = _local_contrast(gray, cx_i, cy_i, radius_i)
        if contrast < params.min_contrast:
            continue

        moles.append(
            DetectedMole(
                center=(cx_i + rx, cy_i + ry),
                radius=radius_i,
                area=area,
                circularity=circularity,
                contrast=contrast,
            )
        )

    moles.sort(key=lambda m: m.contrast * m.area, reverse=True)

    if return_debug:
        debug = {
            "skin": skin,
            "skin_eroded": skin_eroded,
            "valid_mask": work_valid_mask,
            "blackhat": blackhat,
            "binary": binary,
        }
        return moles, debug
    return moles


def _local_contrast(gray: np.ndarray, cx: int, cy: int, radius: int) -> float:
    h, w = gray.shape[:2]
    inner_r = max(radius, 2)
    outer_r = inner_r * 3

    x0 = max(cx - outer_r, 0)
    y0 = max(cy - outer_r, 0)
    x1 = min(cx + outer_r + 1, w)
    y1 = min(cy + outer_r + 1, h)
    if x1 <= x0 or y1 <= y0:
        return 0.0

    patch = gray[y0:y1, x0:x1]
    yy, xx = np.ogrid[y0:y1, x0:x1]
    dist2 = (xx - cx) ** 2 + (yy - cy) ** 2
    inner_mask = dist2 <= inner_r * inner_r
    outer_mask = (dist2 > inner_r * inner_r) & (dist2 <= outer_r * outer_r)

    if not inner_mask.any() or not outer_mask.any():
        return 0.0

    inner_mean = float(patch[inner_mask].mean())
    outer_mean = float(patch[outer_mask].mean())
    return outer_mean - inner_mean


def _find_external_contours(image: np.ndarray):
    result = cv2.findContours(
        image.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    return result[-2]
