from typing import List, Optional, Tuple

import cv2
import numpy as np

from .._compat import dataclass
from .mole_detect import build_skin_mask


@dataclass(frozen=True)
class ReferenceMole:
    center: Tuple[int, int]
    radius: int
    area: float
    circularity: float
    solidity: float
    mean_darkening: float
    confidence: float


@dataclass(frozen=True)
class AlignmentResult:
    ok: bool
    score: float
    translation_px: float
    rotation_deg: float
    reason: str = ""


@dataclass(frozen=True)
class ReferenceDetectParams:
    min_area: int = 8
    max_area: int = 500
    min_darkening: int = 8
    min_mean_darkening: float = 8.0
    min_circularity: float = 0.45
    min_solidity: float = 0.75
    min_confidence: float = 0.45
    max_aspect_ratio: float = 2.5
    mask_erode: int = 5
    max_translation_px: float = 80.0
    max_rotation_deg: float = 15.0
    min_alignment_score: float = 0.30


class ReferenceMoleDetector:
    """Detect new dark circular marks relative to a clean-face reference."""

    def __init__(
        self,
        reference_bgr: np.ndarray,
        params: Optional[ReferenceDetectParams] = None,
    ) -> None:
        if reference_bgr is None or reference_bgr.ndim != 3:
            raise ValueError("reference_bgr must be a valid BGR image")
        self.reference = reference_bgr.copy()
        self.params = params or ReferenceDetectParams()
        self.height, self.width = self.reference.shape[:2]
        self.detect_mask, self.alignment_mask = _build_reference_masks(
            self.reference, self.params.mask_erode
        )
        self.reference_gray = cv2.cvtColor(self.reference, cv2.COLOR_BGR2GRAY)
        self.reference_l = cv2.cvtColor(self.reference, cv2.COLOR_BGR2LAB)[:, :, 0]

    def detect(self, current_bgr: np.ndarray, return_debug: bool = False):
        if current_bgr.shape[:2] != (self.height, self.width):
            raise ValueError(
                "current image size {}x{} does not match reference {}x{}".format(
                    current_bgr.shape[1],
                    current_bgr.shape[0],
                    self.width,
                    self.height,
                )
            )

        aligned, alignment = self._align(current_bgr)
        if not alignment.ok:
            if return_debug:
                empty = np.zeros((self.height, self.width), dtype=np.uint8)
                return [], alignment, {
                    "aligned": aligned,
                    "mask": self.detect_mask,
                    "dark_diff": empty,
                    "binary": empty,
                }
            return [], alignment

        current_l = cv2.cvtColor(aligned, cv2.COLOR_BGR2LAB)[:, :, 0]
        ref_values = self.reference_l[self.detect_mask > 0]
        cur_values = current_l[self.detect_mask > 0]
        brightness_offset = float(np.median(ref_values) - np.median(cur_values))
        current_adjusted = np.clip(
            current_l.astype(np.float32) + brightness_offset, 0, 255
        ).astype(np.uint8)

        ref_blur = cv2.GaussianBlur(self.reference_l, (3, 3), 0)
        cur_blur = cv2.GaussianBlur(current_adjusted, (3, 3), 0)
        dark_diff = cv2.subtract(ref_blur, cur_blur)
        dark_diff = cv2.bitwise_and(dark_diff, dark_diff, mask=self.detect_mask)

        _, binary = cv2.threshold(
            dark_diff, self.params.min_darkening, 255, cv2.THRESH_BINARY
        )
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel, iterations=1)
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel, iterations=1)

        contours = _find_external_contours(binary)
        moles = []  # type: List[ReferenceMole]
        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < self.params.min_area or area > self.params.max_area:
                continue

            perimeter = float(cv2.arcLength(contour, True))
            if perimeter <= 0:
                continue
            circularity = 4.0 * np.pi * area / (perimeter * perimeter)
            if circularity < self.params.min_circularity:
                continue

            hull = cv2.convexHull(contour)
            hull_area = float(cv2.contourArea(hull))
            solidity = area / hull_area if hull_area > 0 else 0.0
            if solidity < self.params.min_solidity:
                continue

            (_, _), (rect_w, rect_h), _ = cv2.minAreaRect(contour)
            aspect = max(rect_w, rect_h) / max(min(rect_w, rect_h), 1.0)
            if aspect > self.params.max_aspect_ratio:
                continue

            candidate_mask = np.zeros(binary.shape, dtype=np.uint8)
            cv2.drawContours(candidate_mask, [contour], -1, 255, thickness=-1)
            mean_darkening = float(cv2.mean(dark_diff, mask=candidate_mask)[0])
            if mean_darkening < self.params.min_mean_darkening:
                continue

            (cx, cy), radius = cv2.minEnclosingCircle(contour)
            darkness_score = min(1.0, mean_darkening / 50.0)
            confidence = float(
                np.clip(
                    0.5 * darkness_score + 0.3 * circularity + 0.2 * solidity,
                    0.0,
                    1.0,
                )
            )
            if confidence < self.params.min_confidence:
                continue
            moles.append(
                ReferenceMole(
                    center=(int(round(cx)), int(round(cy))),
                    radius=max(1, int(round(radius))),
                    area=area,
                    circularity=float(circularity),
                    solidity=float(solidity),
                    mean_darkening=mean_darkening,
                    confidence=confidence,
                )
            )

        moles.sort(key=lambda mole: mole.confidence * mole.area, reverse=True)
        if return_debug:
            return moles, alignment, {
                "aligned": aligned,
                "mask": self.detect_mask,
                "dark_diff": dark_diff,
                "binary": binary,
            }
        return moles, alignment

    def _align(self, current_bgr: np.ndarray) -> Tuple[np.ndarray, AlignmentResult]:
        current_gray = cv2.cvtColor(current_bgr, cv2.COLOR_BGR2GRAY)
        warp = np.eye(2, 3, dtype=np.float32)
        criteria = (
            cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT,
            60,
            1e-5,
        )
        try:
            score, warp = cv2.findTransformECC(
                self.reference_gray,
                current_gray,
                warp,
                cv2.MOTION_AFFINE,
                criteria,
                self.alignment_mask,
                5,
            )
        except cv2.error as exc:
            return current_bgr.copy(), AlignmentResult(
                False, 0.0, 0.0, 0.0, "ECC align failed: {}".format(exc.code)
            )

        tx = float(warp[0, 2])
        ty = float(warp[1, 2])
        translation = float(np.hypot(tx, ty))
        rotation = float(np.degrees(np.arctan2(warp[1, 0], warp[0, 0])))
        aligned = cv2.warpAffine(
            current_bgr,
            warp,
            (self.width, self.height),
            flags=cv2.INTER_LINEAR | cv2.WARP_INVERSE_MAP,
            borderMode=cv2.BORDER_REFLECT101,
        )

        reason = ""
        ok = True
        if score < self.params.min_alignment_score:
            ok = False
            reason = "alignment score too low: {:.3f}".format(score)
        elif translation > self.params.max_translation_px:
            ok = False
            reason = "translation too large: {:.1f}px".format(translation)
        elif abs(rotation) > self.params.max_rotation_deg:
            ok = False
            reason = "rotation too large: {:.2f}deg".format(rotation)

        return aligned, AlignmentResult(
            ok=ok,
            score=float(score),
            translation_px=translation,
            rotation_deg=rotation,
            reason=reason,
        )


def _build_reference_masks(
    reference_bgr: np.ndarray, erode_size: int
) -> Tuple[np.ndarray, np.ndarray]:
    skin = build_skin_mask(reference_bgr)

    count, labels, stats, _ = cv2.connectedComponentsWithStats(skin, connectivity=8)
    if count <= 1:
        raise ValueError("cannot extract a valid skin region from reference image")
    largest_label = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    component = np.where(labels == largest_label, 255, 0).astype(np.uint8)

    erode_size = max(1, int(erode_size) | 1)
    erode_kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE, (erode_size, erode_size)
    )
    detect_mask = cv2.erode(component, erode_kernel, iterations=1)

    gray = cv2.cvtColor(reference_bgr, cv2.COLOR_BGR2GRAY)
    feature_edges = cv2.Canny(gray, 40, 100)
    edge_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    feature_edges = cv2.dilate(feature_edges, edge_kernel, iterations=1)
    detect_mask[feature_edges > 0] = 0

    contours = _find_external_contours(component)
    alignment_mask = np.zeros(component.shape, dtype=np.uint8)
    if contours:
        largest = max(contours, key=cv2.contourArea)
        cv2.drawContours(alignment_mask, [largest], -1, 255, thickness=-1)
        alignment_mask = cv2.erode(alignment_mask, erode_kernel, iterations=1)
    else:
        alignment_mask = detect_mask.copy()
    return detect_mask, alignment_mask


def _find_external_contours(image: np.ndarray):
    result = cv2.findContours(
        image.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    return result[-2]
