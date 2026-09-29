import unittest

from jetson_vision.vision.facial_candidate import FaceBox
from jetson_vision.vision.mole_detect import DetectedMole
from jetson_vision.vision.scan_candidate_fusion import ScanCandidateFusion
from jetson_vision.vision.scan_candidate_fusion import ScanReferenceBoxSmoother


def mole_for_face(
    box,
    relative_x,
    relative_y,
    radius=7,
    area=150.0,
    contrast=58.0,
):
    x, y, width, height = box
    return DetectedMole(
        center=(
            int(round(x + width * relative_x)),
            int(round(y + height * relative_y)),
        ),
        radius=radius,
        area=area,
        circularity=0.82,
        contrast=contrast,
    )


class ScanCandidateFusionTest(unittest.TestCase):
    def test_scan_reference_box_smoother_rejects_one_frame_width_jump(self):
        smoother = ScanReferenceBoxSmoother(alpha=0.10)

        first = smoother.update((100, 40, 300, 360))
        jumped = smoother.update((170, 40, 180, 360))

        self.assertEqual(first, (100.0, 40.0, 300.0, 360.0))
        self.assertAlmostEqual(jumped[0], 107.0)
        self.assertAlmostEqual(jumped[2], 288.0)
        self.assertIsNone(smoother.update(None))
        # A missing observation is fail-closed but does not erase the filter.
        resumed = smoother.update((100, 40, 300, 360))
        self.assertAlmostEqual(resumed[0], 106.3)

    def test_face_relative_track_survives_lateral_motion_and_short_miss(self):
        fusion = ScanCandidateFusion(
            min_hits=5,
            min_view_span_face_widths=0.5,
        )
        boxes = [
            (100, 40, 200, 300),
            (130, 40, 200, 300),
            (160, 40, 200, 300),
            (190, 40, 200, 300),
            (220, 40, 200, 300),
            (250, 40, 200, 300),
        ]
        for index, box in enumerate(boxes):
            candidates = []
            if index != 2:
                candidates = [
                    mole_for_face(
                        box,
                        0.72,
                        0.56,
                        radius=7 + (index % 2),
                        area=150.0 + index * 2.0,
                        contrast=58.0 + index,
                    )
                ]
            self.assertIsNone(
                fusion.update(candidates, box, 1.0 + index * 0.1)
            )

        result = fusion.finish()

        self.assertTrue(result.unique, result)
        self.assertEqual(result.reason, "unique_candidate")
        self.assertEqual(result.selected.hits, 5)
        self.assertAlmostEqual(result.selected.relative_center[0], 0.72, places=2)
        self.assertAlmostEqual(result.selected.relative_center[1], 0.56, places=2)
        self.assertGreaterEqual(result.selected.view_span_face_widths, 0.74)
        self.assertGreater(result.selected.area_consistency, 0.9)

    def test_yaw_relative_drift_uses_velocity_prediction_across_miss(self):
        fusion = ScanCandidateFusion(
            min_hits=5,
            min_view_span_face_widths=0.45,
        )
        relative_positions = [0.55, 0.61, 0.68, None, 0.82, 0.90]
        for index, relative_x in enumerate(relative_positions):
            box = (80 + index * 24, 35, 200, 300)
            candidates = []
            if relative_x is not None:
                candidates = [
                    mole_for_face(
                        box,
                        relative_x,
                        0.56,
                        area=155.0 + index,
                        contrast=62.0,
                    )
                ]
            fusion.update(candidates, box, 1.0 + index * 0.1)

        result = fusion.finish()

        self.assertTrue(result.unique, result)
        self.assertEqual(len(result.candidates), 1)
        self.assertEqual(result.selected.hits, 5)
        self.assertAlmostEqual(result.selected.relative_center[0], 0.68, places=2)

    def test_motion_prediction_does_not_attach_remote_or_reverse_candidate(self):
        fusion = ScanCandidateFusion(
            min_hits=5,
            min_view_span_face_widths=0.45,
        )
        # The true mark moves steadily right in face coordinates.  At frame 3,
        # a dark false candidate appears near the old position while the true
        # mark is near the velocity prediction.  The old-position candidate
        # must start a separate track, not hijack the scan track.
        positions = [0.55, 0.62, 0.69]
        for index, relative_x in enumerate(positions):
            box = (90 + index * 22, 30, 200, 300)
            fusion.update(
                [mole_for_face(box, relative_x, 0.55, contrast=65.0)],
                box,
                2.0 + index * 0.1,
            )

        box = (90 + 3 * 22, 30, 200, 300)
        fusion.update(
            [
                mole_for_face(box, 0.76, 0.55, contrast=65.0),
                mole_for_face(box, 0.55, 0.55, contrast=65.0),
            ],
            box,
            2.3,
        )
        for index, relative_x in enumerate((0.83, 0.90), start=4):
            box = (90 + index * 22, 30, 200, 300)
            fusion.update(
                [mole_for_face(box, relative_x, 0.55, contrast=65.0)],
                box,
                2.0 + index * 0.1,
            )

        result = fusion.finish()

        self.assertTrue(result.unique, result)
        self.assertEqual(result.selected.hits, 6)
        self.assertEqual(result.selected.track_id, 1)
        self.assertEqual(len(result.candidates), 2)
        remote = [
            candidate
            for candidate in result.candidates
            if candidate.track_id != result.selected.track_id
        ][0]
        self.assertEqual(remote.hits, 1)

    def test_multiple_targets_selects_only_stronger_scan_supported_track(self):
        fusion = ScanCandidateFusion(
            min_hits=5,
            min_view_span_face_widths=0.45,
            unique_confidence_margin=0.10,
        )
        for index in range(8):
            box = (80 + index * 20, 30, 200, 300)
            candidates = [
                mole_for_face(
                    box,
                    0.68,
                    0.58,
                    area=160.0 + (index % 2) * 3.0,
                    contrast=65.0,
                )
            ]
            if index < 3:
                candidates.append(
                    mole_for_face(
                        box,
                        0.30,
                        0.42,
                        area=80.0 + index * 20.0,
                        contrast=22.0,
                    )
                )
            fusion.update(candidates, box, 2.0 + index * 0.1)

        result = fusion.finish()

        self.assertTrue(result.unique, result)
        self.assertGreaterEqual(len(result.candidates), 2)
        self.assertAlmostEqual(result.selected.relative_center[0], 0.68, places=2)
        self.assertEqual(result.selected.hits, 8)

    def test_equal_well_supported_candidates_fail_closed(self):
        fusion = ScanCandidateFusion(
            min_hits=5,
            min_view_span_face_widths=0.4,
            unique_confidence_margin=0.12,
        )
        for index in range(7):
            box = (100 + index * 20, 20, 200, 300)
            fusion.update(
                [
                    mole_for_face(box, 0.30, 0.55, contrast=60.0),
                    mole_for_face(box, 0.72, 0.55, contrast=60.0),
                ],
                box,
                3.0 + index * 0.1,
            )

        result = fusion.finish()

        self.assertFalse(result.unique)
        self.assertIsNone(result.selected)
        self.assertEqual(result.reason, "ambiguous_candidates")
        self.assertEqual(len(result.candidates), 2)

    def test_non_overlapping_tracklets_are_stitched_after_scan(self):
        fusion = ScanCandidateFusion(
            min_hits=5,
            min_view_span_face_widths=0.25,
            max_missed_frames=1,
            max_gap_s=0.25,
            stitch_max_gap_s=2.0,
        )
        boxes = [
            (100, 30, 240, 330),
            (120, 30, 240, 330),
            (140, 30, 240, 330),
        ]
        for index, box in enumerate(boxes):
            fusion.update(
                [mole_for_face(box, 0.82, 0.58, contrast=66.0)],
                box,
                10.0 + index * 0.1,
            )

        # The gap deliberately expires the live track.  Scan-only stitching
        # may join it later, but no historical pixel is published here.
        fusion.update([], None, 10.4)
        fusion.update([], None, 10.7)
        later_boxes = [
            (200, 30, 240, 330),
            (220, 30, 240, 330),
            (240, 30, 240, 330),
        ]
        for index, box in enumerate(later_boxes):
            fusion.update(
                [mole_for_face(box, 0.86, 0.60, contrast=64.0)],
                box,
                10.9 + index * 0.1,
            )

        result = fusion.finish()

        self.assertTrue(result.unique, result)
        self.assertEqual(len(result.candidates), 1)
        self.assertEqual(result.selected.hits, 6)
        self.assertGreaterEqual(result.selected.view_span_face_widths, 0.25)

    def test_sequential_distant_tracklets_are_not_stitched(self):
        fusion = ScanCandidateFusion(
            min_hits=3,
            min_view_span_face_widths=0.0,
            max_missed_frames=0,
            max_gap_s=0.15,
            stitch_max_gap_s=2.0,
            unique_confidence_margin=0.12,
        )
        box = (100, 20, 200, 300)
        for index in range(3):
            fusion.update(
                [mole_for_face(box, 0.25, 0.35, contrast=65.0)],
                box,
                20.0 + index * 0.1,
            )
        fusion.update([], box, 20.4)
        for index in range(3):
            fusion.update(
                [mole_for_face(box, 0.78, 0.70, contrast=65.0)],
                box,
                20.6 + index * 0.1,
            )

        result = fusion.finish()

        self.assertFalse(result.unique)
        self.assertEqual(result.reason, "ambiguous_candidates")
        self.assertEqual(len(result.candidates), 2)

    def test_many_hits_without_scan_view_span_are_ineligible(self):
        fusion = ScanCandidateFusion(
            min_hits=4,
            min_view_span_face_widths=0.3,
        )
        box = (100, 20, 200, 300)
        for index in range(8):
            fusion.update(
                [mole_for_face(box, 0.62, 0.50, contrast=70.0)],
                box,
                4.0 + index * 0.1,
            )

        result = fusion.finish()

        self.assertFalse(result.unique)
        self.assertEqual(result.reason, "no_eligible_candidate")
        self.assertIn("view_span", result.candidates[0].rejection_reasons)

    def test_missing_face_reference_is_a_miss_not_a_pixel_fallback(self):
        fusion = ScanCandidateFusion(
            min_hits=4,
            min_view_span_face_widths=0.3,
            max_missed_frames=2,
        )
        first_box = (100, 20, 200, 300)
        second_box = (180, 20, 200, 300)
        fusion.update([mole_for_face(first_box, 0.65, 0.5)], first_box, 5.0)
        fusion.update([], None, 5.1)
        fusion.update([mole_for_face(second_box, 0.65, 0.5)], second_box, 5.2)
        third_box = (220, 20, 200, 300)
        fourth_box = (260, 20, 200, 300)
        fusion.update([mole_for_face(third_box, 0.65, 0.5)], third_box, 5.3)
        fusion.update([mole_for_face(fourth_box, 0.65, 0.5)], fourth_box, 5.4)

        result = fusion.finish()

        self.assertTrue(result.unique, result)
        self.assertEqual(result.selected.hits, 4)

    def test_facebox_object_is_supported(self):
        fusion = ScanCandidateFusion(
            min_hits=3,
            min_view_span_face_widths=0.2,
        )
        for index in range(3):
            raw_box = (100 + index * 30, 20, 200, 300)
            face_box = FaceBox(*raw_box, confidence=0.9, source="test")
            fusion.update(
                [mole_for_face(raw_box, 0.6, 0.5, contrast=70.0)],
                face_box,
                6.0 + index * 0.1,
            )

        self.assertTrue(fusion.finish().unique)

    def test_non_monotonic_sample_time_is_rejected(self):
        fusion = ScanCandidateFusion()
        box = (100, 20, 200, 300)
        fusion.update([], box, 7.0)

        with self.assertRaisesRegex(ValueError, "strictly increasing"):
            fusion.update([], box, 7.0)

    def test_public_result_has_no_realtime_servo_fields(self):
        fusion = ScanCandidateFusion(
            min_hits=3,
            min_view_span_face_widths=0.2,
        )
        for index in range(3):
            box = (100 + index * 30, 20, 200, 300)
            fusion.update(
                [mole_for_face(box, 0.6, 0.5, contrast=70.0)],
                box,
                8.0 + index * 0.1,
            )

        result = fusion.finish()
        summary = result.selected

        self.assertIsNotNone(summary)
        for forbidden in ("center", "dcx", "dcy", "valid", "vision_error"):
            self.assertFalse(hasattr(summary, forbidden), forbidden)
            self.assertFalse(hasattr(result, forbidden), forbidden)


if __name__ == "__main__":
    unittest.main()
