import unittest
from unittest import mock

import numpy as np

from jetson_nano_vision.jetson_vision import rgbd_distance as rgbd_distance_module
from jetson_nano_vision.jetson_vision.rgbd_distance import (
    RgbdDistanceEstimator,
    ScalarDistanceFilter,
)


def _foreground_scene(shape, detections, foreground_mm=1000.0, background_mm=1100.0):
    """Put a nearer, depth-supported surface inside each synthetic RGB box."""

    depth = np.full(shape, background_mm, dtype=np.float32)
    for detection in detections.values():
        x0, y0, x1, y1 = [int(value) for value in detection["box"]]
        depth[y0 : y1 + 1, x0 : x1 + 1] = foreground_mm
    return depth


class RgbdDistanceEstimatorTest(unittest.TestCase):
    def test_estimates_three_dimensional_clearance_between_rois(self):
        detections = {
            "arm": {"box": [1, 5, 6, 10]},
            "face": {"box": [14, 5, 19, 10]},
        }
        depth = _foreground_scene((20, 20), detections)
        estimator = RgbdDistanceEstimator(max_points=200)

        result = estimator.estimate(
            depth,
            (20, 20),
            detections,
            (100.0, 100.0, 0.0, 0.0),
        )

        self.assertIsNotNone(result)
        # At Z=1000 mm and fx=100 px, the nearest ROI columns are 8 px
        # apart, so their lateral 3-D clearance is about 80 mm.
        self.assertAlmostEqual(result.distance_mm, 80.0, delta=1.0)
        self.assertGreaterEqual(result.arm_points, 8)
        self.assertGreaterEqual(result.face_points, 8)
        self.assertGreater(result.arm_valid_ratio, 0.9)
        self.assertGreater(result.face_valid_ratio, 0.9)

    def test_missing_detection_is_invalid_but_preserves_conservative_filter(self):
        estimator = RgbdDistanceEstimator()
        valid = {
            "arm": {"box": [1, 5, 6, 10]},
            "face": {"box": [14, 5, 19, 10]},
        }
        depth = _foreground_scene((20, 20), valid)
        self.assertIsNotNone(estimator.estimate(depth, (20, 20), valid, (100, 100, 0, 0)))

        self.assertIsNone(
            estimator.estimate(
                depth,
                (20, 20),
                {"arm": valid["arm"]},
                (100, 100, 0, 0),
            )
        )
        self.assertIsNotNone(estimator.filter.value)

    def test_kdtree_nearest_distances_match_exact_numpy_fallback(self):
        rng = np.random.default_rng(20260716)
        points_a = rng.normal(size=(127, 3)).astype(np.float32) * 100.0
        points_b = rng.normal(size=(211, 3)).astype(np.float32) * 100.0

        accelerated = RgbdDistanceEstimator._nearest_distances(points_a, points_b)
        reference = RgbdDistanceEstimator._nearest_distances_numpy(points_a, points_b)

        np.testing.assert_allclose(accelerated, reference, rtol=1e-6, atol=1e-5)

    def test_kdtree_clearance_uses_full_cloud_without_cover_penalty(self):
        estimator = RgbdDistanceEstimator(max_points=20)
        arm = np.column_stack(
            (
                np.linspace(0.0, 200.0, 1000),
                np.zeros(1000),
                np.full(1000, 900.0),
            )
        ).astype(np.float32)
        face = np.column_stack(
            (
                np.linspace(0.0, 200.0, 1200),
                np.full(1200, 75.0),
                np.full(1200, 900.0),
            )
        ).astype(np.float32)

        result = estimator._conservative_min_clearance(arm, face)

        self.assertIsNotNone(result)
        raw, exact_minimum, arm_cover, face_cover = result
        self.assertAlmostEqual(raw, 75.0, delta=1e-4)
        self.assertAlmostEqual(exact_minimum, 75.0, delta=1e-4)
        self.assertEqual(arm_cover, 0.0)
        self.assertEqual(face_cover, 0.0)

    def test_sampled_fallback_fails_closed_when_cover_is_too_large(self):
        estimator = RgbdDistanceEstimator(
            max_points=20,
            max_sample_cover_radius_mm=30.0,
        )
        arm = np.column_stack(
            (
                np.linspace(0.0, 2000.0, 1000),
                np.zeros(1000),
                np.full(1000, 900.0),
            )
        ).astype(np.float32)
        face = arm + np.asarray([0.0, 100.0, 0.0], dtype=np.float32)

        with mock.patch.object(rgbd_distance_module, "cKDTree", None):
            result = estimator._conservative_min_clearance(arm, face)

        self.assertIsNone(result)

    def test_sdk_extrinsic_projection_path(self):
        detections = {
            "arm": {"box": [2, 8, 9, 15]},
            "face": {"box": [17, 8, 24, 15]},
        }
        depth = _foreground_scene((30, 30), detections)
        estimator = RgbdDistanceEstimator(max_points=200)

        result = estimator.estimate(
            depth,
            (30, 30),
            detections,
            (100.0, 100.0, 0.0, 0.0),
            (100.0, 100.0, 0.0, 0.0),
            np.eye(3),
            np.zeros(3),
        )

        self.assertIsNotNone(result)
        self.assertAlmostEqual(result.distance_mm, 80.0, delta=1.0)

    def test_brown_rational_distortion_round_trip(self):
        estimator = RgbdDistanceEstimator()
        x = np.asarray([-0.42, -0.15, 0.0, 0.19, 0.44], dtype=np.float64)
        y = np.asarray([0.31, -0.22, 0.0, 0.17, -0.29], dtype=np.float64)
        distortion = (0.098941, -0.124452, -0.000376, 0.001148, -0.124766, 0.0, 0.0, 0.0)

        distorted_x, distorted_y = estimator._distort_normalized(x, y, distortion)
        recovered_x, recovered_y = estimator._undistort_normalized(
            distorted_x,
            distorted_y,
            distortion,
        )

        np.testing.assert_allclose(recovered_x, x, rtol=0.0, atol=1e-8)
        np.testing.assert_allclose(recovered_y, y, rtol=0.0, atol=1e-8)

    def test_depth_ray_cache_applies_nonzero_distortion(self):
        estimator = RgbdDistanceEstimator()
        intrinsics = (100.0, 100.0, 2.0, 2.0)
        zero = (0.0,) * 8
        distorted = (0.2, -0.03, 0.001, -0.002, 0.01, 0.0, 0.0, 0.0)

        zero_x, zero_y = estimator._depth_normalized_rays((5, 5), intrinsics, zero)
        corrected_x, corrected_y = estimator._depth_normalized_rays(
            (5, 5),
            intrinsics,
            distorted,
        )

        self.assertNotAlmostEqual(float(zero_x[0, 0]), float(corrected_x[0, 0]))
        self.assertNotAlmostEqual(float(zero_y[0, 0]), float(corrected_y[0, 0]))
        cached_x, cached_y = estimator._depth_normalized_rays((5, 5), intrinsics, distorted)
        self.assertIs(cached_x, corrected_x)
        self.assertIs(cached_y, corrected_y)

    def test_non_finite_distortion_rejects_projected_estimate(self):
        detections = {
            "arm": {"box": [2, 8, 9, 15]},
            "face": {"box": [17, 8, 24, 15]},
        }
        depth = _foreground_scene((30, 30), detections)
        estimator = RgbdDistanceEstimator(max_points=200)

        result = estimator.estimate(
            depth,
            (30, 30),
            detections,
            (100.0, 100.0, 0.0, 0.0),
            (100.0, 100.0, 0.0, 0.0),
            np.eye(3),
            np.zeros(3),
            "none",
            (float("nan"), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            (0.0,) * 8,
        )

        self.assertIsNone(result)

    def test_depth_roi_offsets_are_applied_after_rgb_scaling(self):
        estimator = RgbdDistanceEstimator(arm_offset=(2, -1))
        roi = estimator._box_to_depth_roi(
            [10, 20, 30, 40],
            (50, 100),
            (200, 100),
            1.0,
            estimator.arm_offset,
        )
        self.assertEqual(roi, (7, 9, 18, 20))

    def test_filter_never_reports_more_clearance_than_current_sample(self):
        estimator = RgbdDistanceEstimator(max_points=200, jump_reset_mm=1000.0)
        detections_far = {
            "arm": {"box": [2, 8, 7, 13]},
            "face": {"box": [22, 8, 27, 13]},
        }
        detections_near = {
            "arm": {"box": [2, 8, 7, 13]},
            "face": {"box": [15, 8, 20, 13]},
        }
        intrinsics = (100.0, 100.0, 0.0, 0.0)
        depth_far = _foreground_scene((30, 30), detections_far)
        depth_near = _foreground_scene((30, 30), detections_near)
        self.assertIsNotNone(estimator.estimate(depth_far, (30, 30), detections_far, intrinsics))

        near = estimator.estimate(depth_near, (30, 30), detections_near, intrinsics)

        fresh_estimator = RgbdDistanceEstimator(max_points=200)
        raw_near = fresh_estimator.estimate(depth_near, (30, 30), detections_near, intrinsics)
        self.assertIsNotNone(near)
        self.assertIsNotNone(raw_near)
        self.assertLessEqual(near.distance_mm, raw_near.distance_mm + 1e-6)

    def test_filter_is_committed_only_after_explicit_quality_approval(self):
        estimator = RgbdDistanceEstimator(max_points=200)
        detections = {
            "arm": {"box": [1, 5, 6, 10]},
            "face": {"box": [14, 5, 19, 10]},
        }
        depth = _foreground_scene((20, 20), detections)

        pending = estimator.estimate(
            depth,
            (20, 20),
            detections,
            (100.0, 100.0, 0.0, 0.0),
            update_filter=False,
        )

        self.assertIsNotNone(pending)
        self.assertIsNone(estimator.filter.value)
        committed = estimator.commit_result(pending)
        self.assertIsNotNone(estimator.filter.value)
        self.assertAlmostEqual(committed.distance_mm, pending.raw_distance_mm)

    def test_sparse_edge_depth_cannot_claim_high_roi_validity(self):
        depth = np.zeros((40, 40), dtype=np.float32)
        depth[1:4, 1:4] = 1000.0
        depth[25:28, 25:28] = 1000.0
        detections = {
            "arm": {"box": [0, 0, 15, 15]},
            "face": {"box": [24, 24, 39, 39]},
        }
        estimator = RgbdDistanceEstimator(max_points=200)

        result = estimator.estimate(
            depth,
            (40, 40),
            detections,
            (100.0, 100.0, 20.0, 20.0),
        )

        self.assertIsNone(result)

    def test_background_visible_through_missing_arm_is_rejected(self):
        depth = np.full((20, 20), 1000.0, dtype=np.float32)
        detections = {
            "arm": {"box": [1, 5, 6, 10]},
            "face": {"box": [14, 5, 19, 10]},
        }

        result = RgbdDistanceEstimator(max_points=200).estimate(
            depth,
            (20, 20),
            detections,
            (100.0, 100.0, 0.0, 0.0),
        )

        self.assertIsNone(result)

    def test_background_visible_through_missing_face_is_rejected(self):
        detections = {
            "arm": {"box": [1, 5, 6, 10]},
            "face": {"box": [14, 5, 19, 10]},
        }
        depth = np.full((20, 20), 1100.0, dtype=np.float32)
        depth[5:11, 1:7] = 1000.0

        result = RgbdDistanceEstimator(max_points=200).estimate(
            depth,
            (20, 20),
            detections,
            (100.0, 100.0, 0.0, 0.0),
        )

        self.assertIsNone(result)

    def test_projected_background_visible_through_missing_face_is_rejected(self):
        detections = {
            "arm": {"box": [2, 8, 9, 15]},
            "face": {"box": [17, 8, 24, 15]},
        }
        depth = np.full((30, 30), 1100.0, dtype=np.float32)
        depth[8:16, 2:10] = 1000.0

        result = RgbdDistanceEstimator(max_points=200).estimate(
            depth,
            (30, 30),
            detections,
            (100.0, 100.0, 0.0, 0.0),
            (100.0, 100.0, 0.0, 0.0),
            np.eye(3),
            np.zeros(3),
        )

        self.assertIsNone(result)

    def test_small_arm_protrusion_controls_minimum_clearance(self):
        detections = {
            "arm": {"box": [2, 10, 13, 21]},
            "face": {"box": [20, 10, 31, 21]},
        }
        depth = np.full((40, 40), 1500.0, dtype=np.float32)
        depth[10:22, 2:14] = 900.0
        depth[10:22, 12:14] = 1080.0
        depth[10:22, 20:32] = 1100.0

        # Force aggressive point sampling; the covering-radius lower bound
        # must still prevent a small protrusion from being hidden.
        result = RgbdDistanceEstimator(max_points=20).estimate(
            depth,
            (40, 40),
            detections,
            (10000.0, 10000.0, 0.0, 0.0),
        )

        self.assertIsNotNone(result)
        self.assertLess(result.distance_mm, 25.0)


class ScalarDistanceFilterTest(unittest.TestCase):
    def test_large_upward_jump_is_delayed_but_downward_jump_is_immediate(self):
        filt = ScalarDistanceFilter(jump_mm=50.0)
        self.assertEqual(filt.update(100.0), 100.0)
        self.assertLess(filt.update(110.0), 110.0)
        self.assertLess(filt.update(220.0), 220.0)
        self.assertEqual(filt.update(20.0), 20.0)


if __name__ == "__main__":
    unittest.main()
