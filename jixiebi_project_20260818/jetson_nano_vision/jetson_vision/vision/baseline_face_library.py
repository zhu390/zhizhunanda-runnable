"""No-mole baseline face library for candidate scoring.

This module intentionally does not replace the normal mole detector.  It uses
captured no-mole frames to answer a narrower question for each candidate:
"is this spot newly darker than similar no-mole frames?"
"""

from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import cv2
import numpy as np

from .._compat import dataclass
from .mole_detect import DetectedMole


VIEW_TO_POINT = {"left": 1, "front": 2, "right": 3}
POINT_TO_VIEW = {value: key for key, value in VIEW_TO_POINT.items()}


@dataclass(frozen=True)
class BaselineMatchScore:
    darker_score: float
    reference_darkness: float
    match_count: int


class BaselineFaceLibrary:
    def __init__(
        self,
        root: str,
        *,
        enabled: bool = True,
        max_images_per_view: int = 48,
        thumb_size: Tuple[int, int] = (96, 72),
        top_k: int = 5,
        patch_radius: int = 8,
    ) -> None:
        self.root = Path(root)
        self.enabled = bool(enabled)
        self.max_images_per_view = max(1, int(max_images_per_view))
        self.thumb_size = (
            max(16, int(thumb_size[0])),
            max(16, int(thumb_size[1])),
        )
        self.top_k = max(1, int(top_k))
        self.patch_radius = max(2, int(patch_radius))
        self._entries: Dict[str, List[Tuple[Path, np.ndarray, np.ndarray]]] = {}
        if self.enabled:
            self._load()

    @property
    def available(self) -> bool:
        return any(self._entries.values())

    def count(self, view: str) -> int:
        return len(self._entries.get(_normalize_view(view), []))

    def views(self) -> List[str]:
        return sorted(view for view, entries in self._entries.items() if entries)

    def score_candidate(
        self,
        frame_bgr: np.ndarray,
        candidate: DetectedMole,
        *,
        view: str,
    ) -> Optional[BaselineMatchScore]:
        view = _normalize_view(view)
        entries = self._entries.get(view, [])
        if not entries or frame_bgr is None or frame_bgr.size == 0:
            return None
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        thumb = _make_thumb(gray, self.thumb_size)
        ranked = sorted(
            entries,
            key=lambda item: float(np.mean(np.abs(thumb.astype(np.float32) - item[1]))),
        )[: self.top_k]
        x, y = candidate.center
        x = int(round(x))
        y = int(round(y))
        r = max(self.patch_radius, int(candidate.radius) + 3)
        h, w = gray.shape[:2]
        if x < 0 or y < 0 or x >= w or y >= h:
            return None
        x0 = max(0, x - r)
        y0 = max(0, y - r)
        x1 = min(w, x + r + 1)
        y1 = min(h, y + r + 1)
        if x1 <= x0 or y1 <= y0:
            return None

        current_patch = gray[y0:y1, x0:x1]
        current_darkness = 255.0 - float(np.percentile(current_patch, 20))
        darker_values = []
        reference_darkness = []
        for _path, _thumb, baseline_gray in ranked:
            if baseline_gray.shape[:2] != gray.shape[:2]:
                continue
            base_patch = baseline_gray[y0:y1, x0:x1]
            base_darkness = 255.0 - float(np.percentile(base_patch, 20))
            darker_values.append(max(0.0, current_darkness - base_darkness))
            reference_darkness.append(base_darkness)
        if not darker_values:
            return None
        return BaselineMatchScore(
            darker_score=float(np.median(darker_values)),
            reference_darkness=float(np.median(reference_darkness)),
            match_count=len(darker_values),
        )

    def score_candidates(
        self,
        frame_bgr: np.ndarray,
        candidates: Sequence[DetectedMole],
        *,
        view: str,
        min_darker_score: float = 18.0,
        max_reference_darkness: float = 95.0,
        reject_static_dark: bool = False,
        static_dark_min_reference: float = 88.0,
        static_dark_max_new_darkening: float = 12.0,
    ) -> List[DetectedMole]:
        if not candidates:
            return []
        scored = []
        for candidate in candidates:
            score = self.score_candidate(frame_bgr, candidate, view=view)
            if score is None:
                scored.append((0.0, candidate))
                continue
            if (
                bool(reject_static_dark)
                and score.reference_darkness >= float(static_dark_min_reference)
                and score.darker_score <= float(static_dark_max_new_darkening)
            ):
                continue
            # A strong new dark spot is good; a region that was already dark in
            # the no-mole library is likely an eye/eyebrow/lip feature.
            baseline_bonus = max(0.0, score.darker_score - float(min_darker_score))
            static_dark_penalty = max(
                0.0,
                score.reference_darkness - float(max_reference_darkness),
            )
            base_quality = max(0.0, float(candidate.area)) * max(
                0.0,
                float(candidate.contrast),
            )
            fused = base_quality * (1.0 + baseline_bonus / 32.0) - (
                static_dark_penalty * max(1.0, float(candidate.area))
            )
            scored.append((float(fused), candidate))
        scored.sort(key=lambda item: item[0], reverse=True)
        return [candidate for _score, candidate in scored]

    def detect_darkening_candidates(
        self,
        frame_bgr: np.ndarray,
        *,
        view: str,
        valid_mask: Optional[np.ndarray] = None,
        min_darkening: float = 22.0,
        min_area: float = 8.0,
        max_area: float = 600.0,
        min_circularity: float = 0.45,
        min_solidity: float = 0.60,
        max_aspect_ratio: float = 2.8,
        max_candidates: int = 12,
    ) -> List[DetectedMole]:
        """Find newly dark blobs relative to similar no-mole baseline frames."""

        view = _normalize_view(view)
        entries = self._entries.get(view, [])
        if not entries or frame_bgr is None or frame_bgr.size == 0:
            return []
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        thumb = _make_thumb(gray, self.thumb_size)
        ranked = sorted(
            entries,
            key=lambda item: float(np.mean(np.abs(thumb.astype(np.float32) - item[1]))),
        )[: self.top_k]
        references = [
            item[2].astype(np.float32)
            for item in ranked
            if item[2].shape[:2] == gray.shape[:2]
        ]
        if not references:
            return []
        reference = np.median(np.stack(references, axis=0), axis=0)
        diff = np.clip(reference - gray.astype(np.float32), 0, 255).astype(np.uint8)

        if valid_mask is not None:
            if valid_mask.ndim == 3:
                valid = cv2.cvtColor(valid_mask, cv2.COLOR_BGR2GRAY)
            else:
                valid = valid_mask
            if valid.shape[:2] == diff.shape[:2]:
                diff = cv2.bitwise_and(diff, diff, mask=np.where(valid > 0, 255, 0).astype(np.uint8))

        _, binary = cv2.threshold(diff, float(min_darkening), 255, cv2.THRESH_BINARY)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel, iterations=1)
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel, iterations=1)

        contours = cv2.findContours(binary.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        candidates: List[DetectedMole] = []
        for cnt in contours:
            area = float(cv2.contourArea(cnt))
            if area < float(min_area) or area > float(max_area):
                continue
            perimeter = float(cv2.arcLength(cnt, True))
            if perimeter <= 0.0:
                continue
            circularity = 4.0 * np.pi * area / (perimeter * perimeter)
            if circularity < float(min_circularity):
                continue
            hull_area = float(cv2.contourArea(cv2.convexHull(cnt)))
            solidity = area / hull_area if hull_area > 0.0 else 0.0
            if solidity < float(min_solidity):
                continue
            rect = cv2.minAreaRect(cnt)
            (_cx, _cy), (rw, rh), _angle = rect
            aspect = max(rw, rh) / max(1.0, min(rw, rh))
            if aspect > float(max_aspect_ratio):
                continue
            (cx, cy), radius = cv2.minEnclosingCircle(cnt)
            cx_i, cy_i = int(round(cx)), int(round(cy))
            radius_i = max(1, int(round(radius)))
            x0 = max(0, cx_i - radius_i - 2)
            y0 = max(0, cy_i - radius_i - 2)
            x1 = min(diff.shape[1], cx_i + radius_i + 3)
            y1 = min(diff.shape[0], cy_i + radius_i + 3)
            contrast = float(np.percentile(diff[y0:y1, x0:x1], 90)) if x1 > x0 and y1 > y0 else 0.0
            if contrast < float(min_darkening):
                continue
            candidates.append(
                DetectedMole(
                    center=(cx_i, cy_i),
                    radius=radius_i,
                    area=area,
                    circularity=float(circularity),
                    contrast=contrast,
                )
            )
        candidates.sort(key=lambda item: float(item.area) * float(item.contrast), reverse=True)
        return candidates[: max(1, int(max_candidates))]

    def _load(self) -> None:
        for view in ("front", "left", "right"):
            view_dir = self.root / view
            files = sorted(view_dir.glob("baseline_{}_*.png".format(view)))
            if len(files) > self.max_images_per_view:
                files = _evenly_sample(files, self.max_images_per_view)
            entries = []
            for path in files:
                frame = cv2.imread(str(path), cv2.IMREAD_COLOR)
                if frame is None:
                    continue
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                entries.append((path, _make_thumb(gray, self.thumb_size), gray))
            self._entries[view] = entries


def _normalize_view(view: object) -> str:
    if isinstance(view, int):
        return POINT_TO_VIEW.get(int(view), "front")
    text = str(view).strip().lower()
    return {"center": "front", "middle": "front", "mid": "front"}.get(text, text)


def _make_thumb(gray: np.ndarray, thumb_size: Tuple[int, int]) -> np.ndarray:
    small = cv2.resize(gray, thumb_size, interpolation=cv2.INTER_AREA)
    return cv2.equalizeHist(small)


def _evenly_sample(items: Sequence[Path], count: int) -> List[Path]:
    if len(items) <= count:
        return list(items)
    indices = np.linspace(0, len(items) - 1, num=count)
    return [items[int(round(index))] for index in indices]
