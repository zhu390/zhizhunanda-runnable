import math
from typing import List, Optional, Tuple

import cv2
import numpy as np

from .._compat import dataclass


@dataclass(frozen=True)
class DetectedRect:
    """A detected (possibly rotated) rectangle in image coordinates."""

    center: Tuple[float, float]
    box: np.ndarray
    area: float


def angle_between(v1: np.ndarray, v2: np.ndarray) -> float:
    """Return the angle (in degrees) between 2 vectors."""
    dot = float(v1.dot(v2))
    n1 = float(np.linalg.norm(v1))
    n2 = float(np.linalg.norm(v2))
    if n1 * n2 == 0:
        return 0.0
    cos = max(-1.0, min(1.0, dot / (n1 * n2)))
    return math.degrees(math.acos(cos))


def detect_rectangles(
    frame: np.ndarray,
    min_area_ratio: float = 0.005,
    max_area_ratio: float = 0.5,
    angle_tol: float = 25.0,
) -> List[DetectedRect]:
    h, w = frame.shape[:2]
    img_area = h * w
    min_area = img_area * float(min_area_ratio)
    max_area = img_area * float(max_area_ratio)

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (3, 3), 0)
    _, thresh = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))

    opened = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel, iterations=1)
    opened = cv2.erode(opened, kernel, iterations=1)
    edges = cv2.Canny(opened, 25, 75)
    contours = _find_external_contours(edges)

    rects = []  # type: List[DetectedRect]
    for cnt in contours:
        area = float(cv2.contourArea(cnt))
        if area < min_area or area > max_area:
            continue

        peri = cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)

        if len(approx) == 4 and cv2.isContourConvex(approx):
            pts = approx.reshape(4, 2).astype(np.float32)
            angles = []
            for i in range(4):
                p0 = pts[i]
                p1 = pts[(i + 1) % 4]
                p2 = pts[(i + 2) % 4]
                angles.append(angle_between(p0 - p1, p2 - p1))
            if all(abs(a - 90) < angle_tol for a in angles):
                dists = [float(np.linalg.norm(pts[i] - pts[(i + 1) % 4])) for i in range(4)]
                min_dist = min(dists)
                max_dist = max(dists)
                if min_dist <= 0:
                    continue
                if max_dist / min_dist > 3:
                    continue
                cx_val, cy_val = _diagonal_intersection(pts)
                rects.append(
                    DetectedRect(
                        center=(cx_val, cy_val),
                        box=pts,
                        area=area,
                    )
                )

    rects.sort(key=lambda r: r.area, reverse=True)
    return rects


def _diagonal_intersection(pts: np.ndarray) -> Tuple[float, float]:
    p0, p1, p2, p3 = pts[0], pts[1], pts[2], pts[3]
    x1, y1 = float(p0[0]), float(p0[1])
    x2, y2 = float(p2[0]), float(p2[1])
    x3, y3 = float(p1[0]), float(p1[1])
    x4, y4 = float(p3[0]), float(p3[1])
    denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if abs(denom) < 1e-6:
        c = np.mean(pts, axis=0)
        return float(c[0]), float(c[1])
    t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom
    return x1 + t * (x2 - x1), y1 + t * (y2 - y1)


def draw_detected_rect(
    frame: np.ndarray,
    rect: Optional[DetectedRect],
    color: Tuple[int, int, int] = (0, 255, 0),
    thickness: int = 3,
    draw_center: bool = True,
    draw_text: bool = True,
) -> None:
    if rect is None:
        return

    box = rect.box.astype(np.int32)
    cv2.polylines(frame, [box], isClosed=True, color=color, thickness=thickness)

    cx, cy = rect.center

    if draw_text:
        label = "A:{}".format(int(rect.area))
        label_coord = "X:{} Y:{}".format(int(cx), int(cy))
        cv2.putText(
            frame,
            label,
            (int(cx) - 80, int(cy) - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            color,
            2,
        )
        cv2.putText(
            frame,
            label_coord,
            (int(cx) - 80, int(cy) + 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            color,
            2,
        )

    if draw_center:
        cv2.circle(frame, (int(cx), int(cy)), 5, (0, 0, 255), -1)


def _find_external_contours(image: np.ndarray):
    result = cv2.findContours(
        image.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    return result[-2]
