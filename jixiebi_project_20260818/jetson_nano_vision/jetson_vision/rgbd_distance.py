"""RGB detection-box to depth-ROI safety-distance estimation."""

from dataclasses import dataclass, replace
from typing import Dict, Optional, Sequence, Tuple

import numpy as np

try:
    from scipy.spatial import cKDTree
except Exception:  # pragma: no cover - the exact NumPy fallback stays supported
    cKDTree = None


@dataclass(frozen=True)
class RgbdDistanceResult:
    distance_mm: float
    arm_points: int
    face_points: int
    arm_valid_ratio: float
    face_valid_ratio: float
    raw_distance_mm: float = 0.0
    filtered_distance_mm: float = 0.0
    sampled_minimum_mm: float = 0.0
    arm_cover_radius_mm: float = 0.0
    face_cover_radius_mm: float = 0.0


class ScalarDistanceFilter:
    """Small random-walk Kalman filter with safety-biased jump handling."""

    def __init__(self, measurement_noise: float = 20.0, process_noise: float = 6.0, jump_mm: float = 80.0):
        self.r = max(1e-6, float(measurement_noise) ** 2)
        self.q = max(1e-6, float(process_noise) ** 2)
        self.jump_mm = max(1.0, float(jump_mm))
        self.value: Optional[float] = None
        self.variance = self.r

    def reset(self) -> None:
        self.value = None
        self.variance = self.r

    def update(self, measurement: float) -> float:
        measurement = float(measurement)
        if self.value is None:
            self.value = measurement
            self.variance = self.r
            return measurement
        # A sudden decrease can mean the person/arm moved closer.  Accept it
        # immediately so filtering can never hide a newly dangerous distance.
        if measurement < self.value - self.jump_mm:
            self.value = measurement
            self.variance = self.r
            return measurement
        # A sudden increase is not allowed to reset the state upward.  It is
        # incorporated through the normal Kalman update, so more clearance is
        # only reported gradually after repeated supporting measurements.
        self.variance += self.q
        gain = self.variance / (self.variance + self.r)
        self.value += gain * (measurement - self.value)
        self.variance *= 1.0 - gain
        return float(self.value)


class RgbdDistanceEstimator:
    """Estimate conservative 3-D surface clearance between two detected ROIs."""

    def __init__(
        self,
        *,
        arm_scale: float = 1.0,
        face_scale: float = 1.0,
        arm_offset: Tuple[int, int] = (0, 0),
        face_offset: Tuple[int, int] = (0, 0),
        min_depth_mm: float = 20.0,
        max_depth_mm: float = 10000.0,
        max_points: int = 600,
        measurement_noise_mm: float = 20.0,
        process_noise_mm: float = 6.0,
        jump_reset_mm: float = 80.0,
        max_sample_cover_radius_mm: float = 30.0,
    ) -> None:
        self.arm_scale = max(0.05, float(arm_scale))
        self.face_scale = max(0.05, float(face_scale))
        self.arm_offset = (int(arm_offset[0]), int(arm_offset[1]))
        self.face_offset = (int(face_offset[0]), int(face_offset[1]))
        self.min_depth_mm = float(min_depth_mm)
        self.max_depth_mm = float(max_depth_mm)
        self.max_points = max(20, int(max_points))
        self.max_sample_cover_radius_mm = max(
            1.0,
            float(max_sample_cover_radius_mm),
        )
        self.surface_tolerance_mm = 30.0
        self.surface_tolerance_ratio = 0.03
        self.arm_background_separation_mm = 15.0
        self.min_surface_points = 8
        self._depth_ray_cache_key = None
        self._depth_ray_cache = None
        self.filter = ScalarDistanceFilter(
            measurement_noise=measurement_noise_mm,
            process_noise=process_noise_mm,
            jump_mm=jump_reset_mm,
        )

    def reset(self) -> None:
        self.filter.reset()

    def estimate(
        self,
        depth_mm: np.ndarray,
        rgb_size: Tuple[int, int],
        detections: Dict[str, dict],
        intrinsics: Sequence[float],
        rgb_intrinsics: Optional[Sequence[float]] = None,
        depth_to_rgb_rotation: Optional[np.ndarray] = None,
        depth_to_rgb_translation: Optional[np.ndarray] = None,
        rgb_flip: str = "none",
        depth_distortion: Optional[Sequence[float]] = None,
        rgb_distortion: Optional[Sequence[float]] = None,
        update_filter: bool = True,
    ) -> Optional[RgbdDistanceResult]:
        if depth_mm is None or depth_mm.ndim != 2:
            return None
        if "arm" not in detections or "face" not in detections:
            return None
        if len(intrinsics) != 4:
            return None
        fx, fy, cx, cy = [float(v) for v in intrinsics]
        if fx <= 0.0 or fy <= 0.0:
            return None

        has_extrinsics = (
            rgb_intrinsics is not None
            and depth_to_rgb_rotation is not None
            and depth_to_rgb_translation is not None
        )
        if has_extrinsics:
            try:
                arm_points, arm_valid_ratio, face_points, face_valid_ratio = self._projected_roi_points(
                    depth_mm,
                    rgb_size,
                    detections,
                    (fx, fy, cx, cy),
                    rgb_intrinsics,
                    depth_to_rgb_rotation,
                    depth_to_rgb_translation,
                    rgb_flip,
                    depth_distortion,
                    rgb_distortion,
                )
            except (TypeError, ValueError):
                return None
        else:
            arm_roi = self._box_to_depth_roi(
                detections["arm"]["box"], depth_mm.shape, rgb_size, self.arm_scale, self.arm_offset
            )
            arm_outer_roi = self._box_to_depth_roi(
                detections["arm"]["box"],
                depth_mm.shape,
                rgb_size,
                self.arm_scale * 1.5,
                self.arm_offset,
            )
            face_roi = self._box_to_depth_roi(
                detections["face"]["box"], depth_mm.shape, rgb_size, self.face_scale, self.face_offset
            )
            face_outer_roi = self._box_to_depth_roi(
                detections["face"]["box"],
                depth_mm.shape,
                rgb_size,
                self.face_scale * 1.5,
                self.face_offset,
            )
            arm_points, arm_valid_ratio = self._roi_points_with_quality(
                depth_mm,
                arm_roi,
                fx,
                fy,
                cx,
                cy,
                require_foreground=True,
                outer_roi=arm_outer_roi,
            )
            face_points, face_valid_ratio = self._roi_points_with_quality(
                depth_mm,
                face_roi,
                fx,
                fy,
                cx,
                cy,
                require_foreground=True,
                outer_roi=face_outer_roi,
            )
        if arm_points.shape[0] < 8 or face_points.shape[0] < 8:
            return None

        clearance = self._conservative_min_clearance(arm_points, face_points)
        if clearance is None:
            return None
        raw_distance, sampled_minimum, arm_cover, face_cover = clearance
        result = RgbdDistanceResult(
            float(raw_distance),
            int(arm_points.shape[0]),
            int(face_points.shape[0]),
            float(arm_valid_ratio),
            float(face_valid_ratio),
            float(raw_distance),
            float(raw_distance),
            float(sampled_minimum),
            float(arm_cover),
            float(face_cover),
        )
        return self.commit_result(result) if update_filter else result

    def commit_result(self, result: RgbdDistanceResult) -> RgbdDistanceResult:
        """Commit one already quality-approved raw observation to the filter."""

        raw_distance = float(result.raw_distance_mm)
        filtered = self.filter.update(raw_distance)
        # Clearance is a safety quantity: smoothing may delay an increase, but
        # it must never delay a decrease and report more room than the current
        # raw observation actually contains.
        conservative_distance = min(raw_distance, filtered)
        return replace(
            result,
            distance_mm=float(conservative_distance),
            filtered_distance_mm=float(filtered),
        )

    def _projected_roi_points(
        self,
        depth_mm: np.ndarray,
        rgb_size: Tuple[int, int],
        detections: Dict[str, dict],
        depth_intrinsics: Sequence[float],
        rgb_intrinsics: Sequence[float],
        rotation,
        translation,
        rgb_flip: str,
        depth_distortion: Optional[Sequence[float]] = None,
        rgb_distortion: Optional[Sequence[float]] = None,
    ) -> Tuple[np.ndarray, float, np.ndarray, float]:
        if len(rgb_intrinsics) != 4:
            raise ValueError("rgb intrinsics must contain fx, fy, cx, cy")
        dfx, dfy, dcx, dcy = [float(v) for v in depth_intrinsics]
        rfx, rfy, rcx, rcy = [float(v) for v in rgb_intrinsics]
        if min(dfx, dfy, rfx, rfy) <= 0.0:
            raise ValueError("camera focal length must be positive")
        rotation = np.asarray(rotation, dtype=np.float64).reshape(3, 3)
        translation = np.asarray(translation, dtype=np.float64).reshape(3)
        if not np.all(np.isfinite(rotation)) or not np.all(np.isfinite(translation)):
            raise ValueError("invalid depth-to-rgb transform")
        depth_distortion = self._coerce_distortion(depth_distortion)
        rgb_distortion = self._coerce_distortion(rgb_distortion)

        valid = (
            np.isfinite(depth_mm)
            & (depth_mm >= self.min_depth_mm)
            & (depth_mm <= self.max_depth_mm)
        )
        ys, xs = np.nonzero(valid)
        if xs.size == 0:
            empty = np.empty((0, 3), dtype=np.float32)
            return empty, 0.0, empty.copy(), 0.0
        z = depth_mm[ys, xs].astype(np.float64)
        ray_x, ray_y = self._depth_normalized_rays(
            depth_mm.shape,
            (dfx, dfy, dcx, dcy),
            depth_distortion,
        )
        depth_points = np.column_stack(
            (
                ray_x[ys, xs] * z,
                ray_y[ys, xs] * z,
                z,
            )
        )
        rgb_points = depth_points @ rotation.T + translation[None, :]
        in_front = rgb_points[:, 2] > 1e-6
        depth_points = depth_points[in_front]
        rgb_points = rgb_points[in_front]
        if rgb_points.size == 0:
            empty = np.empty((0, 3), dtype=np.float32)
            return empty, 0.0, empty.copy(), 0.0
        rgb_x = rgb_points[:, 0] / rgb_points[:, 2]
        rgb_y = rgb_points[:, 1] / rgb_points[:, 2]
        rgb_x, rgb_y = self._distort_normalized(rgb_x, rgb_y, rgb_distortion)
        u = rfx * rgb_x + rcx
        v = rfy * rgb_y + rcy
        rgb_w, rgb_h = [max(1, int(value)) for value in rgb_size]
        if rgb_flip in ("1", "-1"):
            u = float(rgb_w - 1) - u
        if rgb_flip in ("0", "-1"):
            v = float(rgb_h - 1) - v

        def points_in_box(
            key: str,
            scale: float,
            offset: Tuple[int, int],
        ) -> Tuple[np.ndarray, float]:
            x0, y0, x1, y1 = self._scaled_box(
                detections[key]["box"], scale
            )
            depth_h, depth_w = depth_mm.shape[:2]
            x_shift = float(offset[0]) * rgb_w / max(1, depth_w)
            y_shift = float(offset[1]) * rgb_h / max(1, depth_h)
            x0 += x_shift
            x1 += x_shift
            y0 += y_shift
            y1 += y_shift
            mask = (u >= x0) & (u <= x1) & (v >= y0) & (v <= y1)
            points = depth_points[mask]
            center_x = (x0 + x1) * 0.5
            center_y = (y0 + y1) * 0.5
            inner_x0 = center_x - (x1 - x0) * 0.25
            inner_x1 = center_x + (x1 - x0) * 0.25
            inner_y0 = center_y - (y1 - y0) * 0.25
            inner_y1 = center_y + (y1 - y0) * 0.25
            inner_mask = (
                (u >= inner_x0)
                & (u <= inner_x1)
                & (v >= inner_y0)
                & (v <= inner_y1)
            )
            outer_x0 = center_x - (x1 - x0) * 0.75
            outer_x1 = center_x + (x1 - x0) * 0.75
            outer_y0 = center_y - (y1 - y0) * 0.75
            outer_y1 = center_y + (y1 - y0) * 0.75
            outer_mask = (
                (u >= outer_x0)
                & (u <= outer_x1)
                & (v >= outer_y0)
                & (v <= outer_y1)
            )
            ring_points = depth_points[outer_mask & ~mask]
            depth_density = float(depth_mm.size) / float(max(1, rgb_w * rgb_h))
            box_area = self._clipped_box_area((x0, y0, x1, y1), (rgb_w, rgb_h))
            inner_area = self._clipped_box_area(
                (inner_x0, inner_y0, inner_x1, inner_y1),
                (rgb_w, rgb_h),
            )
            expected = max(1.0, box_area * depth_density)
            expected_inner = max(1.0, inner_area * depth_density)
            valid_ratio = min(1.0, float(np.count_nonzero(mask)) / expected)
            central_ratio = min(1.0, float(np.count_nonzero(inner_mask)) / expected_inner)
            quality_ratio = min(valid_ratio, central_ratio)
            points = self._select_surface_points(
                points,
                depth_points[inner_mask],
                ring_points,
                require_foreground=True,
            )
            quality_ratio = min(
                quality_ratio,
                min(1.0, float(points.shape[0]) / expected),
            )
            return points.astype(np.float32, copy=False), quality_ratio

        arm_points, arm_ratio = points_in_box("arm", self.arm_scale, self.arm_offset)
        face_points, face_ratio = points_in_box("face", self.face_scale, self.face_offset)
        return arm_points, arm_ratio, face_points, face_ratio

    @staticmethod
    def _coerce_distortion(
        distortion: Optional[Sequence[float]],
    ) -> Tuple[float, float, float, float, float, float, float, float]:
        """Return OpenCV-order Brown/rational coefficients.

        The order is ``k1, k2, p1, p2, k3, k4, k5, k6``.  Older callers
        which do not provide calibration retain the pinhole behaviour, while
        an explicitly malformed SDK calibration is rejected.
        """

        if distortion is None:
            return (0.0,) * 8
        values = np.asarray(distortion, dtype=np.float64).reshape(-1)
        if values.size != 8 or not np.all(np.isfinite(values)):
            raise ValueError("distortion must contain eight finite coefficients")
        return tuple(float(value) for value in values)

    @staticmethod
    def _distort_normalized(
        x: np.ndarray,
        y: np.ndarray,
        distortion: Sequence[float],
    ) -> Tuple[np.ndarray, np.ndarray]:
        k1, k2, p1, p2, k3, k4, k5, k6 = distortion
        if not any(abs(float(value)) > 0.0 for value in distortion):
            return x, y
        r2 = x * x + y * y
        r4 = r2 * r2
        r6 = r4 * r2
        numerator = 1.0 + k1 * r2 + k2 * r4 + k3 * r6
        denominator = 1.0 + k4 * r2 + k5 * r4 + k6 * r6
        if np.any(np.abs(denominator) < 1e-12):
            raise ValueError("invalid rational distortion denominator")
        radial = numerator / denominator
        delta_x = 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x)
        delta_y = p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y
        distorted_x = x * radial + delta_x
        distorted_y = y * radial + delta_y
        if not np.all(np.isfinite(distorted_x)) or not np.all(np.isfinite(distorted_y)):
            raise ValueError("distortion produced non-finite coordinates")
        return distorted_x, distorted_y

    @classmethod
    def _undistort_normalized(
        cls,
        distorted_x: np.ndarray,
        distorted_y: np.ndarray,
        distortion: Sequence[float],
    ) -> Tuple[np.ndarray, np.ndarray]:
        if not any(abs(float(value)) > 0.0 for value in distortion):
            return distorted_x, distorted_y
        k1, k2, p1, p2, k3, k4, k5, k6 = distortion
        x = distorted_x.copy()
        y = distorted_y.copy()
        for _ in range(10):
            r2 = x * x + y * y
            r4 = r2 * r2
            r6 = r4 * r2
            numerator = 1.0 + k1 * r2 + k2 * r4 + k3 * r6
            denominator = 1.0 + k4 * r2 + k5 * r4 + k6 * r6
            if np.any(np.abs(numerator) < 1e-12) or np.any(np.abs(denominator) < 1e-12):
                raise ValueError("invalid rational distortion during inversion")
            radial = numerator / denominator
            delta_x = 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x)
            delta_y = p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y
            x = (distorted_x - delta_x) / radial
            y = (distorted_y - delta_y) / radial
        check_x, check_y = cls._distort_normalized(x, y, distortion)
        residual = max(
            float(np.max(np.abs(check_x - distorted_x))),
            float(np.max(np.abs(check_y - distorted_y))),
        )
        if not np.isfinite(residual) or residual > 1e-5:
            raise ValueError("distortion inversion did not converge")
        return x, y

    def _depth_normalized_rays(
        self,
        depth_shape: Tuple[int, int],
        intrinsics: Sequence[float],
        distortion: Sequence[float],
    ) -> Tuple[np.ndarray, np.ndarray]:
        height, width = [int(value) for value in depth_shape[:2]]
        fx, fy, cx, cy = [float(value) for value in intrinsics]
        key = (height, width, fx, fy, cx, cy) + tuple(float(value) for value in distortion)
        if key == self._depth_ray_cache_key and self._depth_ray_cache is not None:
            return self._depth_ray_cache
        grid_x, grid_y = np.meshgrid(
            np.arange(width, dtype=np.float64),
            np.arange(height, dtype=np.float64),
        )
        distorted_x = (grid_x - cx) / fx
        distorted_y = (grid_y - cy) / fy
        rays = self._undistort_normalized(distorted_x, distorted_y, distortion)
        self._depth_ray_cache_key = key
        self._depth_ray_cache = rays
        return rays

    def _select_surface_points(
        self,
        points: np.ndarray,
        inner_points: np.ndarray,
        ring_points: np.ndarray,
        *,
        require_foreground: bool,
    ) -> np.ndarray:
        """Attribute depth to the detected surface, not merely its RGB box.

        Both detected objects need a center-supported surface in front of a
        valid outer ring.  Once that attribution is established, retain every
        point that is credibly in front of the ring.  This deliberately keeps
        small arm protrusions which may be closest to the face.
        """

        if (
            points.shape[0] < self.min_surface_points
            or inner_points.shape[0] < self.min_surface_points
        ):
            return np.empty((0, 3), dtype=np.float32)
        center_depth = float(np.median(inner_points[:, 2]))
        tolerance = max(
            self.surface_tolerance_mm,
            abs(center_depth) * self.surface_tolerance_ratio,
        )
        if not require_foreground:
            surface = points[np.abs(points[:, 2] - center_depth) <= tolerance]
            if surface.shape[0] < self.min_surface_points:
                return np.empty((0, 3), dtype=np.float32)
            return surface.astype(np.float32, copy=False)
        if ring_points.shape[0] < self.min_surface_points:
            return np.empty((0, 3), dtype=np.float32)
        background_depth = float(np.median(ring_points[:, 2]))
        foreground_limit = background_depth - self.arm_background_separation_mm
        center_surface = inner_points[
            (inner_points[:, 2] <= foreground_limit)
            & (np.abs(inner_points[:, 2] - center_depth) <= tolerance)
        ]
        if center_surface.shape[0] < self.min_surface_points:
            return np.empty((0, 3), dtype=np.float32)
        surface = points[points[:, 2] <= foreground_limit]
        if surface.shape[0] < self.min_surface_points:
            return np.empty((0, 3), dtype=np.float32)
        return surface.astype(np.float32, copy=False)

    @staticmethod
    def _clipped_box_area(
        box: Tuple[float, float, float, float],
        image_size: Tuple[int, int],
    ) -> float:
        x0, y0, x1, y1 = box
        width, height = image_size
        x0 = max(0.0, min(float(width), x0))
        x1 = max(0.0, min(float(width), x1))
        y0 = max(0.0, min(float(height), y0))
        y1 = max(0.0, min(float(height), y1))
        return max(0.0, x1 - x0) * max(0.0, y1 - y0)

    @staticmethod
    def _scaled_box(box, scale: float) -> Tuple[float, float, float, float]:
        x0, y0, x1, y1 = [float(v) for v in box]
        center_x = (x0 + x1) * 0.5
        center_y = (y0 + y1) * 0.5
        width = max(2.0, (x1 - x0) * max(0.05, float(scale)))
        height = max(2.0, (y1 - y0) * max(0.05, float(scale)))
        return (
            center_x - width * 0.5,
            center_y - height * 0.5,
            center_x + width * 0.5,
            center_y + height * 0.5,
        )

    def _box_to_depth_roi(
        self,
        box,
        depth_shape: Tuple[int, int],
        rgb_size: Tuple[int, int],
        scale: float,
        offset: Tuple[int, int],
    ) -> Tuple[int, int, int, int]:
        depth_h, depth_w = depth_shape[:2]
        rgb_w, rgb_h = [max(1, int(v)) for v in rgb_size]
        x0, y0, x1, y1 = [float(v) for v in box]
        box_cx = (x0 + x1) * 0.5
        box_cy = (y0 + y1) * 0.5
        box_w = max(2.0, (x1 - x0) * scale)
        box_h = max(2.0, (y1 - y0) * scale)
        x0 = (box_cx - box_w * 0.5) * depth_w / rgb_w + offset[0]
        x1 = (box_cx + box_w * 0.5) * depth_w / rgb_w + offset[0]
        y0 = (box_cy - box_h * 0.5) * depth_h / rgb_h + offset[1]
        y1 = (box_cy + box_h * 0.5) * depth_h / rgb_h + offset[1]
        ix0 = max(0, min(depth_w - 1, int(round(min(x0, x1)))))
        ix1 = max(ix0 + 1, min(depth_w, int(round(max(x0, x1))) + 1))
        iy0 = max(0, min(depth_h - 1, int(round(min(y0, y1)))))
        iy1 = max(iy0 + 1, min(depth_h, int(round(max(y0, y1))) + 1))
        return ix0, iy0, ix1, iy1

    def _roi_points_with_quality(
        self,
        depth_mm,
        roi,
        fx,
        fy,
        cx,
        cy,
        *,
        require_foreground: bool = False,
        outer_roi: Optional[Tuple[int, int, int, int]] = None,
    ):
        x0, y0, x1, y1 = roi
        region = depth_mm[y0:y1, x0:x1]
        valid = (
            np.isfinite(region)
            & (region >= self.min_depth_mm)
            & (region <= self.max_depth_mm)
        )
        valid_ratio = float(np.count_nonzero(valid)) / float(max(1, valid.size))
        region_h, region_w = region.shape[:2]
        inner_x0 = max(0, region_w // 4)
        inner_x1 = max(inner_x0 + 1, region_w - region_w // 4)
        inner_y0 = max(0, region_h // 4)
        inner_y1 = max(inner_y0 + 1, region_h - region_h // 4)
        inner_valid = valid[inner_y0:inner_y1, inner_x0:inner_x1]
        central_ratio = float(np.count_nonzero(inner_valid)) / float(max(1, inner_valid.size))
        quality_ratio = min(valid_ratio, central_ratio)
        ys, xs = np.nonzero(valid)
        if xs.size == 0:
            return np.empty((0, 3), dtype=np.float32), 0.0
        z = region[ys, xs].astype(np.float64)
        u = xs.astype(np.float64) + float(x0)
        v = ys.astype(np.float64) + float(y0)
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy
        points = np.column_stack((x, y, z)).astype(np.float32)
        inner_point_mask = (
            (xs >= inner_x0)
            & (xs < inner_x1)
            & (ys >= inner_y0)
            & (ys < inner_y1)
        )
        inner_points = points[inner_point_mask]

        ring_points = np.empty((0, 3), dtype=np.float32)
        if outer_roi is not None:
            ox0, oy0, ox1, oy1 = outer_roi
            outer_region = depth_mm[oy0:oy1, ox0:ox1]
            outer_valid = (
                np.isfinite(outer_region)
                & (outer_region >= self.min_depth_mm)
                & (outer_region <= self.max_depth_mm)
            )
            outer_ys, outer_xs = np.nonzero(outer_valid)
            if outer_xs.size:
                outer_u = outer_xs.astype(np.float64) + float(ox0)
                outer_v = outer_ys.astype(np.float64) + float(oy0)
                outside_inner_roi = (
                    (outer_u < float(x0))
                    | (outer_u >= float(x1))
                    | (outer_v < float(y0))
                    | (outer_v >= float(y1))
                )
                outer_u = outer_u[outside_inner_roi]
                outer_v = outer_v[outside_inner_roi]
                outer_z = outer_region[outer_ys, outer_xs].astype(np.float64)[outside_inner_roi]
                if outer_z.size:
                    outer_x = (outer_u - cx) * outer_z / fx
                    outer_y = (outer_v - cy) * outer_z / fy
                    ring_points = np.column_stack(
                        (outer_x, outer_y, outer_z)
                    ).astype(np.float32)

        points = self._select_surface_points(
            points,
            inner_points,
            ring_points,
            require_foreground=require_foreground,
        )
        quality_ratio = min(
            quality_ratio,
            float(points.shape[0]) / float(max(1, valid.size)),
        )
        return points.astype(np.float32, copy=False), quality_ratio

    def _roi_points(self, depth_mm, roi, fx, fy, cx, cy) -> np.ndarray:
        """Compatibility wrapper returning only points for debug callers."""

        points, _ = self._roi_points_with_quality(depth_mm, roi, fx, fy, cx, cy)
        return points

    def _conservative_min_clearance(
        self,
        arm_points: np.ndarray,
        face_points: np.ndarray,
    ) -> Optional[Tuple[float, float, float, float]]:
        """Return a lower bound on the full-cloud minimum distance.

        With SciPy available, query the complete point clouds using a KD-tree;
        this avoids the large false drops previously caused by subtracting one
        isolated omitted point's covering radius.  The deterministic sampled
        fallback remains available for minimal installations, but fails closed
        when either covering radius is too large to support a useful bound.
        """

        if arm_points.size == 0 or face_points.size == 0:
            return None

        if cKDTree is not None:
            full_distances = self._nearest_distances(arm_points, face_points)
            if full_distances.size == 0:
                return None
            exact_minimum = float(np.min(full_distances))
            if not np.isfinite(exact_minimum):
                return None
            return exact_minimum, exact_minimum, 0.0, 0.0

        def sample_with_cover(points: np.ndarray) -> Tuple[np.ndarray, float]:
            if points.shape[0] <= self.max_points:
                return points, 0.0
            indices = np.linspace(
                0,
                points.shape[0] - 1,
                self.max_points,
                dtype=np.int64,
            )
            sampled = points[indices]
            cover_distances = self._nearest_distances(points, sampled)
            cover_radius = (
                float(np.max(cover_distances))
                if cover_distances.size
                else float("inf")
            )
            return sampled, cover_radius

        arm_sample, arm_cover = sample_with_cover(arm_points)
        face_sample, face_cover = sample_with_cover(face_points)
        if (
            not np.isfinite(arm_cover)
            or not np.isfinite(face_cover)
            or arm_cover > self.max_sample_cover_radius_mm
            or face_cover > self.max_sample_cover_radius_mm
        ):
            # Never discard a poorly covered point and continue with a larger
            # reported clearance.  Reject the whole frame instead.
            return None
        sampled_distances = self._nearest_distances(arm_sample, face_sample)
        if sampled_distances.size == 0:
            return None
        sampled_minimum = float(np.min(sampled_distances))
        lower_bound = max(0.0, sampled_minimum - arm_cover - face_cover)
        return lower_bound, sampled_minimum, arm_cover, face_cover

    @staticmethod
    def _nearest_distances(a: np.ndarray, b: np.ndarray, chunk_size: int = 128) -> np.ndarray:
        if a.size == 0 or b.size == 0:
            return np.empty((0,), dtype=np.float32)
        if cKDTree is not None:
            distances, _ = cKDTree(np.asarray(b, dtype=np.float64)).query(
                np.asarray(a, dtype=np.float64),
                k=1,
            )
            return np.asarray(distances, dtype=np.float32)
        return RgbdDistanceEstimator._nearest_distances_numpy(a, b, chunk_size)

    @staticmethod
    def _nearest_distances_numpy(
        a: np.ndarray,
        b: np.ndarray,
        chunk_size: int = 128,
    ) -> np.ndarray:
        """Exact NumPy fallback used when SciPy is unavailable."""

        if a.size == 0 or b.size == 0:
            return np.empty((0,), dtype=np.float32)
        result = np.empty((a.shape[0],), dtype=np.float32)
        b64 = b.astype(np.float64, copy=False)
        for start in range(0, a.shape[0], chunk_size):
            stop = min(start + chunk_size, a.shape[0])
            diff = a[start:stop, None, :].astype(np.float64, copy=False) - b64[None, :, :]
            result[start:stop] = np.sqrt(np.min(np.sum(diff * diff, axis=2), axis=1))
        return result
