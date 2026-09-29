import unittest
from unittest import mock

import cv2
import numpy as np

from jetson_nano_vision.jetson_vision.vision.facial_candidate import (
    FaceBox,
    FaceLocator,
    FacialCandidateDetector,
    FacialCandidateParams,
    build_facial_masks,
    build_privacy_view,
)
from jetson_nano_vision.jetson_vision.vision.mole_detect import DetectParams
from jetson_nano_vision.jetson_vision.vision.mole_detect import DetectedMole
from jetson_nano_vision.jetson_vision.vision.target_stability import (
    CandidateStabilityTracker,
)


class FixedFaceLocator:
    def __init__(self, face):
        self.face = face

    def locate(self, _frame):
        return self.face


class FacialCandidateTests(unittest.TestCase):
    def setUp(self):
        self.face = FaceBox(170, 60, 300, 360, 1.0, "test")

    def test_face_locator_initializes_without_cv2_data_attribute(self):
        with mock.patch.object(cv2, "data", None, create=True):
            locator = FaceLocator(model_path=None)

        self.assertFalse(locator._haar.empty())
        self.assertFalse(locator._profile_haar.empty())

    def test_face_locator_scales_haar_input_to_configured_imgsz(self):
        class RecordingCascade:
            def __init__(self, faces):
                self.faces = faces
                self.shapes = []

            def empty(self):
                return False

            def detectMultiScale(self, image, **_kwargs):
                self.shapes.append(image.shape)
                return self.faces

        locator = FaceLocator(model_path=None, imgsz=416)
        frontal = RecordingCascade([(100, 40, 160, 220)])
        locator._haar = frontal
        locator._profile_haar = RecordingCascade([])

        face = locator._locate_haar(np.zeros((480, 640, 3), dtype=np.uint8))

        self.assertIsNotNone(face)
        self.assertEqual(frontal.shapes[0][1], 416)
        self.assertEqual(frontal.shapes[0][0], 312)

    def test_small_profile_eye_box_yields_to_containing_full_skin_head(self):
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        profile = FaceBox(136, 209, 170, 170, 0.0, "haar-profile")
        skin = FaceBox(160, 0, 352, 480, 0.0, "skin")
        locator = FaceLocator(model_path=None)

        with mock.patch.object(locator, "_locate_haar", return_value=profile), mock.patch.object(
            locator,
            "_locate_skin",
            return_value=skin,
        ):
            face = locator.locate(frame)

        self.assertEqual(face, skin)

    def test_small_frontal_eye_box_yields_to_containing_full_skin_head(self):
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        frontal = FaceBox(330, 205, 145, 150, 0.0, "haar")
        skin = FaceBox(160, 0, 352, 480, 0.0, "skin")
        locator = FaceLocator(model_path=None)

        with mock.patch.object(locator, "_locate_haar", return_value=frontal), mock.patch.object(
            locator,
            "_locate_skin",
            return_value=skin,
        ):
            face = locator.locate(frame)

        self.assertEqual(face, skin)

    def test_small_yolo_eye_box_yields_to_containing_full_skin_head(self):
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        yolo_eye = FaceBox(330, 205, 131, 176, 0.91, "yolo")
        skin = FaceBox(160, 0, 354, 480, 0.0, "skin")
        locator = FaceLocator(model_path=None)

        with mock.patch.object(locator, "_locate_haar", return_value=None), mock.patch.object(
            locator,
            "_locate_yolo",
            return_value=yolo_eye,
        ), mock.patch.object(
            locator,
            "_locate_skin",
            return_value=skin,
        ):
            face = locator.locate(frame)

        self.assertEqual(face, skin)

    def test_normal_frontal_face_is_not_replaced_by_similar_skin_head(self):
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        frontal = FaceBox(105, 45, 405, 405, 0.0, "haar")
        skin = FaceBox(90, 15, 430, 465, 0.0, "skin")
        locator = FaceLocator(model_path=None)

        with mock.patch.object(locator, "_locate_haar", return_value=frontal), mock.patch.object(
            locator,
            "_locate_skin",
            return_value=skin,
        ):
            face = locator.locate(frame)

        self.assertEqual(face, frontal)

    def test_genuine_noncontained_profile_box_is_not_replaced_by_skin(self):
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        profile = FaceBox(350, 80, 220, 300, 0.0, "haar-profile")
        unrelated_skin = FaceBox(20, 20, 250, 360, 0.0, "skin")
        locator = FaceLocator(model_path=None)

        with mock.patch.object(locator, "_locate_haar", return_value=profile), mock.patch.object(
            locator,
            "_locate_skin",
            return_value=unrelated_skin,
        ):
            face = locator.locate(frame)

        self.assertEqual(face, profile)

    def test_face_box_is_cached_for_six_frames_while_candidates_are_present(self):
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        locator = mock.Mock()
        locator.locate.return_value = self.face
        spot = DetectedMole((255, 285), 5, 70.0, 0.8, 30.0)
        detector = FacialCandidateDetector(
            locator,
            params=FacialCandidateParams(
                face_refresh_frames=6,
                candidate_recovery_enabled=False,
            ),
        )

        with mock.patch(
            "jetson_nano_vision.jetson_vision.vision.facial_candidate.detect_moles",
            return_value=[spot],
        ):
            for _ in range(6):
                detector.detect(frame, render=False)
            self.assertEqual(locator.locate.call_count, 1)

            detector.detect(frame, render=False)

        self.assertEqual(locator.locate.call_count, 2)

    def test_single_candidate_miss_keeps_trusted_face_cache(self):
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        locator = mock.Mock()
        locator.locate.return_value = self.face
        spot = DetectedMole((255, 285), 5, 70.0, 0.8, 30.0)
        detector = FacialCandidateDetector(
            locator,
            params=FacialCandidateParams(
                face_refresh_frames=6,
                candidate_recovery_enabled=False,
            ),
        )

        with mock.patch(
            "jetson_nano_vision.jetson_vision.vision.facial_candidate.detect_moles",
            side_effect=([spot], [], [spot]),
        ):
            detector.detect(frame, render=False)
            detector.detect(frame, render=False)
            self.assertEqual(locator.locate.call_count, 1)

            detector.detect(frame, render=False)

        self.assertEqual(locator.locate.call_count, 1)

    def test_two_consecutive_candidate_misses_force_face_refresh(self):
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        locator = mock.Mock()
        locator.locate.return_value = self.face
        spot = DetectedMole((255, 285), 5, 70.0, 0.8, 30.0)
        detector = FacialCandidateDetector(
            locator,
            params=FacialCandidateParams(
                face_refresh_frames=6,
                candidate_recovery_enabled=False,
            ),
        )

        with mock.patch(
            "jetson_nano_vision.jetson_vision.vision.facial_candidate.detect_moles",
            side_effect=([spot], [], [], [spot]),
        ):
            detector.detect(frame, render=False)
            detector.detect(frame, render=False)
            detector.detect(frame, render=False)
            self.assertEqual(locator.locate.call_count, 1)

            detector.detect(frame, render=False)

        self.assertEqual(locator.locate.call_count, 2)

    def test_local_recovery_replaces_fragments_with_one_fresh_candidate(self):
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        face_mask = np.zeros(frame.shape[:2], dtype=np.uint8)
        cv2.ellipse(face_mask, (320, 240), (170, 215), 0, 0, 360, 255, -1)
        weak_top = DetectedMole((389, 269), 3, 16.0, 0.74, 45.0)
        weak_bottom = DetectedMole((395, 279), 3, 18.0, 0.75, 31.0)
        recovered = DetectedMole((393, 273), 8, 78.0, 0.46, 65.0)
        detector = FacialCandidateDetector(
            FixedFaceLocator(self.face),
            params=FacialCandidateParams(candidate_recovery_enabled=True),
        )

        with mock.patch(
            "jetson_nano_vision.jetson_vision.vision.facial_candidate.build_facial_masks",
            return_value=(face_mask, face_mask.copy(), {}),
        ), mock.patch(
            "jetson_nano_vision.jetson_vision.vision.facial_candidate.detect_moles",
            side_effect=([weak_top, weak_bottom], [recovered, weak_top]),
        ):
            result = detector.detect(frame, render=False)

        self.assertEqual([item.center for item in result.candidates], [(393, 273)])

    def test_local_recovery_never_reuses_stale_coordinate_on_pixel_miss(self):
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        face_mask = np.zeros(frame.shape[:2], dtype=np.uint8)
        cv2.ellipse(face_mask, (320, 240), (170, 215), 0, 0, 360, 255, -1)
        seed = DetectedMole((390, 270), 4, 30.0, 0.72, 58.0)
        recovered = DetectedMole((393, 273), 8, 78.0, 0.46, 65.0)
        detector = FacialCandidateDetector(
            FixedFaceLocator(self.face),
            params=FacialCandidateParams(candidate_recovery_enabled=True),
        )

        with mock.patch(
            "jetson_nano_vision.jetson_vision.vision.facial_candidate.build_facial_masks",
            return_value=(face_mask, face_mask.copy(), {}),
        ), mock.patch(
            "jetson_nano_vision.jetson_vision.vision.facial_candidate.detect_moles",
            side_effect=([seed], [recovered], [], []),
        ):
            first = detector.detect(frame, render=False)
            missing = detector.detect(frame, render=False)

        self.assertEqual([item.center for item in first.candidates], [(393, 273)])
        self.assertEqual(missing.candidates, [])

    def test_missing_face_forces_locator_retry_on_the_next_frame(self):
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        locator = mock.Mock()
        locator.locate.side_effect = (None, self.face)
        spot = DetectedMole((255, 285), 5, 70.0, 0.8, 30.0)
        detector = FacialCandidateDetector(
            locator,
            params=FacialCandidateParams(face_refresh_frames=6),
        )

        first = detector.detect(frame, render=False)
        with mock.patch(
            "jetson_nano_vision.jetson_vision.vision.facial_candidate.detect_moles",
            return_value=[spot],
        ):
            second = detector.detect(frame, render=False)

        self.assertIsNone(first.face)
        self.assertEqual(second.face, self.face)
        self.assertEqual(locator.locate.call_count, 2)

    def test_partial_face_mask_is_rejected_against_selected_view_baseline(self):
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        locator = mock.Mock()
        locator.locate.side_effect = (
            FaceBox(150, 20, 340, 430, 1.0, "skin"),
            FaceBox(330, 160, 150, 180, 1.0, "haar"),
        )
        detector = FacialCandidateDetector(
            locator,
            params=FacialCandidateParams(
                face_refresh_frames=1,
                partial_face_guard=True,
                candidate_recovery_enabled=False,
            ),
        )
        full_mask = np.zeros(frame.shape[:2], dtype=np.uint8)
        cv2.ellipse(full_mask, (320, 240), (170, 215), 0, 0, 360, 255, -1)
        partial_mask = np.zeros(frame.shape[:2], dtype=np.uint8)
        cv2.ellipse(partial_mask, (405, 235), (70, 85), 0, 0, 360, 255, -1)
        empty_excluded = {}

        with mock.patch(
            "jetson_nano_vision.jetson_vision.vision.facial_candidate.build_facial_masks",
            side_effect=(
                (full_mask, full_mask.copy(), empty_excluded),
                (partial_mask, partial_mask.copy(), empty_excluded),
            ),
        ), mock.patch(
            "jetson_nano_vision.jetson_vision.vision.facial_candidate.detect_moles",
            return_value=[],
        ) as detect_spy:
            first = detector.detect(frame, render=False)
            second = detector.detect(frame, render=False)

        self.assertIsNotNone(first.face)
        self.assertIsNone(second.face)
        self.assertEqual(second.candidates, [])
        self.assertTrue(second.message.startswith("PARTIAL_FACE collapsed"))
        self.assertEqual(detect_spy.call_count, 1)

    def test_eye_sized_first_mask_cannot_establish_full_face_baseline(self):
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        detector = FacialCandidateDetector(
            FixedFaceLocator(FaceBox(330, 120, 170, 282, 1.0, "haar")),
            params=FacialCandidateParams(partial_face_guard=True),
        )
        eye_mask = np.zeros(frame.shape[:2], dtype=np.uint8)
        cv2.ellipse(eye_mask, (415, 260), (84, 140), 0, 0, 360, 255, -1)

        with mock.patch(
            "jetson_nano_vision.jetson_vision.vision.facial_candidate.build_facial_masks",
            return_value=(eye_mask, eye_mask.copy(), {}),
        ), mock.patch(
            "jetson_nano_vision.jetson_vision.vision.facial_candidate.detect_moles",
            return_value=[],
        ) as detect_spy:
            result = detector.detect(frame, render=False)

        self.assertIsNone(result.face)
        self.assertTrue(result.message.startswith("PARTIAL_FACE baseline rejected"))
        self.assertIsNone(detector._face_mask_baseline)
        self.assertEqual(detect_spy.call_count, 0)

    def test_live_baseline_requires_two_full_face_frames_when_configured(self):
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        detector = FacialCandidateDetector(
            FixedFaceLocator(self.face),
            params=FacialCandidateParams(
                face_refresh_frames=6,
                face_baseline_frames=2,
                partial_face_guard=True,
            ),
        )
        full_mask = np.zeros(frame.shape[:2], dtype=np.uint8)
        cv2.ellipse(full_mask, (320, 240), (170, 215), 0, 0, 360, 255, -1)

        with mock.patch(
            "jetson_nano_vision.jetson_vision.vision.facial_candidate.build_facial_masks",
            return_value=(full_mask, full_mask.copy(), {}),
        ), mock.patch(
            "jetson_nano_vision.jetson_vision.vision.facial_candidate.detect_moles",
            return_value=[],
        ) as detect_spy:
            first = detector.detect(frame, render=False)
            second = detector.detect(frame, render=False)

        self.assertIsNone(first.face)
        self.assertEqual(first.message, "FACE_BASELINE_PENDING 1/2")
        self.assertIsNotNone(second.face)
        self.assertIsNotNone(detector._face_mask_baseline)
        self.assertEqual(detect_spy.call_count, 1)

    def test_closeup_frame_does_not_poison_frozen_selected_view_baseline(self):
        detector = FacialCandidateDetector(
            FixedFaceLocator(self.face),
            params=FacialCandidateParams(partial_face_guard=True),
        )
        full_mask = np.zeros((480, 640), dtype=np.uint8)
        cv2.ellipse(full_mask, (320, 240), (170, 215), 0, 0, 360, 255, -1)
        closeup_mask = np.zeros((480, 640), dtype=np.uint8)
        cv2.ellipse(closeup_mask, (320, 240), (225, 235), 0, 0, 360, 255, -1)

        self.assertEqual(
            detector._check_face_mask_quality(full_mask, (480, 640, 3)),
            (True, ""),
        )
        baseline = detector._face_mask_baseline
        self.assertEqual(
            detector._check_face_mask_quality(closeup_mask, (480, 640, 3)),
            (True, ""),
        )
        self.assertEqual(detector._face_mask_baseline, baseline)
        self.assertEqual(
            detector._check_face_mask_quality(full_mask, (480, 640, 3)),
            (True, ""),
        )

    def test_full_face_recovers_after_partial_mask_without_losing_baseline(self):
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        locator = mock.Mock()
        locator.locate.side_effect = (
            FaceBox(150, 20, 340, 430, 1.0, "skin"),
            FaceBox(330, 160, 150, 180, 1.0, "haar"),
            FaceBox(165, 20, 340, 430, 1.0, "skin"),
        )
        detector = FacialCandidateDetector(
            locator,
            params=FacialCandidateParams(
                face_refresh_frames=1,
                partial_face_guard=True,
                candidate_recovery_enabled=False,
            ),
        )
        full_mask = np.zeros(frame.shape[:2], dtype=np.uint8)
        cv2.ellipse(full_mask, (320, 240), (170, 215), 0, 0, 360, 255, -1)
        partial_mask = np.zeros(frame.shape[:2], dtype=np.uint8)
        cv2.ellipse(partial_mask, (405, 235), (70, 85), 0, 0, 360, 255, -1)
        translated_full_mask = np.zeros(frame.shape[:2], dtype=np.uint8)
        cv2.ellipse(
            translated_full_mask,
            (335, 240),
            (170, 215),
            0,
            0,
            360,
            255,
            -1,
        )
        spot = DetectedMole((390, 285), 6, 90.0, 0.9, 45.0)

        with mock.patch(
            "jetson_nano_vision.jetson_vision.vision.facial_candidate.build_facial_masks",
            side_effect=(
                (full_mask, full_mask.copy(), {}),
                (partial_mask, partial_mask.copy(), {}),
                (translated_full_mask, translated_full_mask.copy(), {}),
            ),
        ), mock.patch(
            "jetson_nano_vision.jetson_vision.vision.facial_candidate.detect_moles",
            side_effect=([], [spot]),
        ):
            detector.detect(frame, render=False)
            rejected = detector.detect(frame, render=False)
            recovered = detector.detect(frame, render=False)

        self.assertIsNone(rejected.face)
        self.assertIsNotNone(recovered.face)
        self.assertEqual([item.center for item in recovered.candidates], [spot.center])

    def test_detector_reset_clears_face_cache_and_view_baseline(self):
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        locator = mock.Mock()
        locator.locate.return_value = self.face
        detector = FacialCandidateDetector(
            locator,
            params=FacialCandidateParams(
                face_refresh_frames=6,
                partial_face_guard=True,
            ),
        )

        detector.detect(frame, render=False)
        self.assertIsNotNone(detector._face_mask_baseline)
        detector.reset()

        self.assertIsNone(detector._cached_face)
        self.assertEqual(detector._face_frames_left, 0)
        self.assertEqual(detector._candidate_miss_streak, 0)
        self.assertIsNone(detector._face_mask_baseline)
        self.assertIsNone(detector._face_mask_pending)
        self.assertEqual(detector._face_mask_pending_count, 0)

    def test_deferred_render_builds_privacy_view_only_once(self):
        frame = np.full((480, 640, 3), (205, 205, 205), dtype=np.uint8)
        cv2.ellipse(frame, (320, 245), (145, 185), 0, 0, 360, (95, 145, 205), -1)
        detector = FacialCandidateDetector(FixedFaceLocator(self.face))

        with mock.patch(
            "jetson_nano_vision.jetson_vision.vision.facial_candidate.build_privacy_view",
            wraps=build_privacy_view,
        ) as render_spy:
            result = detector.detect(frame, render=False)
            self.assertEqual(render_spy.call_count, 0)
            detector.render_candidates(frame, result, result.candidates)

        self.assertEqual(render_spy.call_count, 1)

    def test_masks_exclude_eyes_mouth_and_background(self):
        face_mask, valid_mask, excluded = build_facial_masks(
            (480, 640, 3), self.face
        )
        self.assertEqual(face_mask.shape, (480, 640))
        self.assertEqual(valid_mask.shape, (480, 640))
        self.assertEqual(int(valid_mask[20, 20]), 0)
        for name in (
            "left_eye",
            "right_eye",
            "left_eyebrow",
            "right_eyebrow",
            "nostrils",
            "mouth",
        ):
            self.assertIn(name, excluded)
            self.assertGreater(int(np.count_nonzero(excluded[name])), 0)
            self.assertEqual(
                int(np.count_nonzero(valid_mask[excluded[name] > 0])),
                0,
            )

        for side in ("left", "right"):
            eye = excluded["{}_eye".format(side)]
            eye_guard = excluded["{}_eye_guard".format(side)]
            eyebrow_guard = excluded["{}_eyebrow_guard".format(side)]
            self.assertGreater(cv2.countNonZero(eye_guard), cv2.countNonZero(eye))
            self.assertGreater(cv2.countNonZero(eyebrow_guard), 0)
            self.assertEqual(
                int(np.count_nonzero(valid_mask[eye_guard > 0])),
                0,
            )

    def test_eyes_only_mode_keeps_only_eyes_as_hard_anatomical_exclusion(self):
        _face_mask, valid_mask, excluded = build_facial_masks(
            (480, 640, 3),
            self.face,
            detection_exclusion_mode="eyes_only",
        )

        # The face/background ROI is still mandatory.
        self.assertEqual(int(valid_mask[20, 20]), 0)
        # Eye and its small safety halo remain fail-closed.
        self.assertEqual(int(valid_mask[190, 257]), 0)
        self.assertGreater(int(excluded["left_eye_guard"][190, 257]), 0)
        # Other anatomical features are diagnostic metadata only; they no
        # longer delete otherwise valid target pixels in production mode.
        self.assertEqual(int(valid_mask[145, 257]), 255)  # eyebrow
        self.assertEqual(int(valid_mask[267, 320]), 255)  # nostrils
        self.assertEqual(int(valid_mask[330, 320]), 255)  # mouth
        self.assertNotIn("upper_edge_guard", excluded)

    def test_eyes_only_mode_detects_round_mark_in_old_upper_edge_guard(self):
        frame = np.full((480, 640, 3), (205, 205, 205), dtype=np.uint8)
        cv2.ellipse(
            frame,
            (320, 245),
            (145, 185),
            0,
            0,
            360,
            (95, 145, 205),
            -1,
        )
        for eye_x in (255, 385):
            cv2.ellipse(frame, (eye_x, 190), (35, 12), 0, 0, 360, (20, 20, 25), -1)
            cv2.ellipse(frame, (eye_x, 145), (38, 6), 0, 0, 360, (45, 45, 50), -1)
        cheek_mark = (190, 245)
        cv2.circle(frame, cheek_mark, 7, (15, 15, 20), -1)
        detector = FacialCandidateDetector(
            FixedFaceLocator(FaceBox(170, 60, 300, 360, 1.0, "haar")),
            DetectParams(
                min_area=8,
                max_area=220,
                min_circularity=0.62,
                min_solidity=0.85,
                max_aspect_ratio=2.4,
                blackhat_ksize=19,
                min_contrast=28,
                skin_dilate=11,
                use_skin_mask=True,
            ),
            FacialCandidateParams(detection_exclusion_mode="eyes_only"),
        )

        result = detector.detect(frame, render=False)

        self.assertEqual(int(result.valid_mask[cheek_mark[1], cheek_mark[0]]), 255)
        self.assertTrue(
            any(
                np.hypot(
                    item.center[0] - cheek_mark[0],
                    item.center[1] - cheek_mark[1],
                )
                <= 4
                for item in result.candidates
            ),
            [item.center for item in result.candidates],
        )

    def test_eyes_only_mode_rejects_brow_texture_candidate(self):
        frame = np.full((480, 640, 3), (205, 205, 205), dtype=np.uint8)
        cv2.ellipse(
            frame,
            (320, 245),
            (145, 185),
            0,
            0,
            360,
            (95, 145, 205),
            -1,
        )
        for eye_x in (255, 385):
            cv2.ellipse(frame, (eye_x, 190), (35, 12), 0, 0, 360, (20, 20, 25), -1)
            cv2.ellipse(frame, (eye_x, 145), (38, 6), 0, 0, 360, (45, 45, 50), -1)
        cheek_mark = (405, 268)
        brow_false_mark = (255, 145)
        cv2.circle(frame, cheek_mark, 7, (15, 15, 20), -1)
        cv2.circle(frame, brow_false_mark, 7, (10, 10, 15), -1)
        detector = FacialCandidateDetector(
            FixedFaceLocator(FaceBox(170, 60, 300, 360, 1.0, "haar")),
            DetectParams(
                min_area=8,
                max_area=220,
                min_circularity=0.55,
                min_solidity=0.80,
                max_aspect_ratio=2.4,
                blackhat_ksize=19,
                min_contrast=20,
                skin_dilate=11,
                use_skin_mask=True,
            ),
            FacialCandidateParams(
                detection_exclusion_mode="eyes_only",
                max_candidates=10,
            ),
        )

        result = detector.detect(frame, render=False)
        centers = [item.center for item in result.candidates]

        self.assertTrue(
            any(np.hypot(x - cheek_mark[0], y - cheek_mark[1]) <= 4 for x, y in centers),
            centers,
        )
        self.assertFalse(
            any(
                np.hypot(x - brow_false_mark[0], y - brow_false_mark[1]) <= 10
                for x, y in centers
            ),
            centers,
        )

    def test_enhanced_filter_can_be_disabled_for_legacy_brow_candidates(self):
        frame = np.full((480, 640, 3), (205, 205, 205), dtype=np.uint8)
        cv2.ellipse(
            frame,
            (320, 245),
            (145, 185),
            0,
            0,
            360,
            (95, 145, 205),
            -1,
        )
        for eye_x in (255, 385):
            cv2.ellipse(frame, (eye_x, 190), (35, 12), 0, 0, 360, (20, 20, 25), -1)
            cv2.ellipse(frame, (eye_x, 145), (38, 6), 0, 0, 360, (45, 45, 50), -1)
        brow_false_mark = (255, 145)
        cv2.circle(frame, brow_false_mark, 7, (10, 10, 15), -1)
        detector = FacialCandidateDetector(
            FixedFaceLocator(FaceBox(170, 60, 300, 360, 1.0, "haar")),
            DetectParams(),
            FacialCandidateParams(
                detection_exclusion_mode="eyes_only",
                enhanced_candidate_filters=False,
                max_candidates=10,
            ),
        )
        brow_candidate = DetectedMole(
            brow_false_mark,
            7,
            100.0,
            0.85,
            60.0,
        )

        with mock.patch(
            "jetson_nano_vision.jetson_vision.vision.facial_candidate.detect_moles",
            return_value=[brow_candidate],
        ):
            result = detector.detect(frame, render=False)
        centers = [item.center for item in result.candidates]

        self.assertTrue(
            any(
                np.hypot(x - brow_false_mark[0], y - brow_false_mark[1]) <= 10
                for x, y in centers
            ),
            centers,
        )

    def test_tiny_weak_lower_boundary_noise_is_rejected(self):
        frame = np.full((480, 640, 3), (205, 205, 205), dtype=np.uint8)
        cv2.ellipse(
            frame,
            (320, 245),
            (145, 185),
            0,
            0,
            360,
            (95, 145, 205),
            -1,
        )
        cheek_mark = (405, 268)
        weak_bottom_noise = (410, 456)
        cv2.circle(frame, cheek_mark, 7, (15, 15, 20), -1)
        cv2.circle(frame, weak_bottom_noise, 4, (55, 55, 60), -1)
        detector = FacialCandidateDetector(
            FixedFaceLocator(FaceBox(170, 60, 300, 360, 1.0, "haar")),
            DetectParams(
                min_area=8,
                max_area=220,
                min_circularity=0.45,
                min_solidity=0.65,
                max_aspect_ratio=2.4,
                blackhat_ksize=19,
                min_contrast=10,
                skin_dilate=11,
                use_skin_mask=True,
            ),
            FacialCandidateParams(
                detection_exclusion_mode="eyes_only",
                max_candidates=10,
            ),
        )

        result = detector.detect(frame, render=False)
        centers = [item.center for item in result.candidates]

        self.assertTrue(
            any(np.hypot(x - cheek_mark[0], y - cheek_mark[1]) <= 4 for x, y in centers),
            centers,
        )
        self.assertFalse(
            any(
                np.hypot(x - weak_bottom_noise[0], y - weak_bottom_noise[1]) <= 8
                for x, y in centers
            ),
            centers,
        )

    def test_strong_lower_face_mark_is_not_rejected_as_boundary_noise(self):
        frame = np.full((480, 640, 3), (205, 205, 205), dtype=np.uint8)
        cv2.ellipse(
            frame,
            (320, 245),
            (145, 185),
            0,
            0,
            360,
            (95, 145, 205),
            -1,
        )
        lower_mark = (320, 410)
        cv2.circle(frame, lower_mark, 8, (10, 10, 15), -1)
        detector = FacialCandidateDetector(
            FixedFaceLocator(FaceBox(170, 60, 300, 360, 1.0, "haar")),
            DetectParams(
                min_area=8,
                max_area=260,
                min_circularity=0.45,
                min_solidity=0.65,
                max_aspect_ratio=2.4,
                blackhat_ksize=19,
                min_contrast=10,
                skin_dilate=11,
                use_skin_mask=True,
            ),
            FacialCandidateParams(
                detection_exclusion_mode="eyes_only",
                max_candidates=10,
            ),
        )

        result = detector.detect(frame, render=False)
        centers = [item.center for item in result.candidates]

        self.assertTrue(
            any(np.hypot(x - lower_mark[0], y - lower_mark[1]) <= 4 for x, y in centers),
            centers,
        )

    def test_dark_spot_in_cheek_is_candidate_but_eye_spot_is_excluded(self):
        frame = np.full((480, 640, 3), (205, 205, 205), dtype=np.uint8)
        cv2.ellipse(frame, (320, 250), (142, 180), 0, 0, 360, (95, 145, 205), -1)
        cheek_spot = (255, 285)
        eye_spot = (255, 175)
        eyebrow_spot = (255, 145)
        cv2.circle(frame, cheek_spot, 7, (20, 20, 25), -1)
        cv2.circle(frame, eye_spot, 7, (20, 20, 25), -1)
        cv2.circle(frame, eyebrow_spot, 7, (20, 20, 25), -1)

        detector = FacialCandidateDetector(
            FixedFaceLocator(self.face),
            DetectParams(
                min_area=20,
                max_area=500,
                min_circularity=0.4,
                min_solidity=0.7,
                max_aspect_ratio=2.0,
                blackhat_ksize=31,
                min_contrast=12,
                skin_dilate=3,
                use_skin_mask=True,
            ),
            FacialCandidateParams(max_candidates=10),
        )
        result = detector.detect(frame)
        centers = [item.center for item in result.candidates]
        self.assertTrue(
            any(np.hypot(x - cheek_spot[0], y - cheek_spot[1]) <= 4 for x, y in centers),
            centers,
        )
        self.assertFalse(
            any(np.hypot(x - eye_spot[0], y - eye_spot[1]) <= 10 for x, y in centers),
            centers,
        )
        self.assertFalse(
            any(
                np.hypot(x - eyebrow_spot[0], y - eyebrow_spot[1]) <= 10
                for x, y in centers
            ),
            centers,
        )

    def test_lower_cheek_mark_cannot_become_eye_mask(self):
        frame = np.full((480, 640, 3), (205, 205, 205), dtype=np.uint8)
        cv2.ellipse(
            frame,
            (320, 245),
            (145, 185),
            0,
            0,
            360,
            (95, 145, 205),
            -1,
        )
        cheek_spot = (400, 265)
        cv2.ellipse(
            frame,
            cheek_spot,
            (14, 7),
            0,
            0,
            360,
            (15, 15, 20),
            -1,
        )
        detector = FacialCandidateDetector(
            FixedFaceLocator(FaceBox(170, 60, 300, 360, 1.0, "haar")),
            DetectParams(
                min_area=20,
                max_area=600,
                min_circularity=0.4,
                min_solidity=0.7,
                max_aspect_ratio=2.4,
                blackhat_ksize=31,
                min_contrast=12,
                use_skin_mask=True,
            ),
            FacialCandidateParams(max_candidates=10),
        )

        result = detector.detect(frame, render=False)

        self.assertEqual(
            int(
                result.excluded_masks["right_eye"][
                    cheek_spot[1], cheek_spot[0]
                ]
            ),
            0,
        )
        self.assertEqual(
            int(result.valid_mask[cheek_spot[1], cheek_spot[0]]),
            255,
        )
        self.assertTrue(
            any(
                np.hypot(
                    item.center[0] - cheek_spot[0],
                    item.center[1] - cheek_spot[1],
                )
                <= 4
                for item in result.candidates
            ),
            [item.center for item in result.candidates],
        )

        # When dynamic eye extraction is uncertain, the fixed anatomical eye
        # area must still remain excluded for laser safety.
        anatomical_eye = (400, 210)
        self.assertGreater(
            int(
                result.excluded_masks["right_eye"][
                    anatomical_eye[1], anatomical_eye[0]
                ]
            ),
            0,
        )
        self.assertEqual(
            int(result.valid_mask[anatomical_eye[1], anatomical_eye[0]]),
            0,
        )

    def test_skin_face_closeup_keeps_high_eye_guard_off_cheek_mark(self):
        frame = np.full((480, 640, 3), (205, 205, 205), dtype=np.uint8)
        cv2.ellipse(
            frame,
            (167, 198),
            (167, 198),
            0,
            0,
            360,
            (95, 145, 205),
            -1,
        )
        anatomical_eyes = np.zeros(frame.shape[:2], dtype=np.uint8)
        for eye_x in (80, 260):
            cv2.ellipse(
                frame,
                (eye_x, 98),
                (32, 13),
                0,
                0,
                360,
                (20, 20, 25),
                -1,
            )
            cv2.ellipse(
                anatomical_eyes,
                (eye_x, 98),
                (32, 13),
                0,
                0,
                360,
                255,
                -1,
            )
            cv2.ellipse(
                frame,
                (eye_x, 55),
                (36, 6),
                0,
                0,
                360,
                (45, 45, 50),
                -1,
            )
        cheek_spot = (298, 180)
        cv2.circle(frame, cheek_spot, 7, (15, 15, 20), -1)
        detector = FacialCandidateDetector(
            FixedFaceLocator(FaceBox(0, 0, 335, 396, 1.0, "skin")),
            DetectParams(
                min_area=8,
                max_area=220,
                min_circularity=0.62,
                min_solidity=0.85,
                max_aspect_ratio=1.8,
                blackhat_ksize=19,
                min_contrast=28,
                use_skin_mask=True,
            ),
            FacialCandidateParams(max_candidates=12),
        )

        result = detector.detect(frame, render=False)

        self.assertEqual(
            int(
                result.excluded_masks["right_eye_guard"][
                    cheek_spot[1], cheek_spot[0]
                ]
            ),
            0,
        )
        self.assertEqual(
            int(result.valid_mask[cheek_spot[1], cheek_spot[0]]),
            255,
        )
        self.assertTrue(
            any(
                np.hypot(
                    item.center[0] - cheek_spot[0],
                    item.center[1] - cheek_spot[1],
                )
                <= 4
                for item in result.candidates
            ),
            [item.center for item in result.candidates],
        )

        anatomical_eye = (260, 98)
        self.assertGreater(
            int(
                result.excluded_masks["right_eye_guard"][
                    anatomical_eye[1], anatomical_eye[0]
                ]
            ),
            0,
        )
        self.assertEqual(
            int(result.valid_mask[anatomical_eye[1], anatomical_eye[0]]),
            0,
        )
        self.assertEqual(
            int(np.count_nonzero(result.valid_mask[anatomical_eyes > 0])),
            0,
        )

    def test_skin_full_head_guards_lower_features_but_keeps_cheek_mole(self):
        frame = np.full((480, 640, 3), (205, 205, 205), dtype=np.uint8)
        cv2.ellipse(
            frame,
            (336, 240),
            (176, 240),
            0,
            0,
            360,
            (95, 145, 205),
            -1,
        )
        eyes = ((244, 240), (439, 226))
        for eye in eyes:
            cv2.ellipse(frame, eye, (34, 12), 0, 0, 360, (20, 20, 25), -1)
            cv2.ellipse(
                frame,
                (eye[0], eye[1] - 38),
                (38, 6),
                0,
                0,
                360,
                (45, 45, 50),
                -1,
            )
        nostril = (387, 348)
        mouth = (365, 420)
        cheek_mole = (489, 306)
        cv2.circle(frame, nostril, 5, (25, 25, 30), -1)
        cv2.ellipse(frame, mouth, (45, 7), 0, 0, 360, (35, 35, 40), -1)
        cv2.circle(frame, cheek_mole, 7, (15, 15, 20), -1)
        detector = FacialCandidateDetector(
            FixedFaceLocator(FaceBox(160, 0, 352, 480, 1.0, "skin")),
            DetectParams(
                min_area=8,
                max_area=220,
                min_circularity=0.62,
                min_solidity=0.85,
                max_aspect_ratio=1.8,
                blackhat_ksize=19,
                min_contrast=28,
                use_skin_mask=True,
            ),
            FacialCandidateParams(max_candidates=12),
        )

        result = detector.detect(frame, render=False)

        for point in eyes:
            self.assertEqual(int(result.face_mask[point[1], point[0]]), 255)
            self.assertEqual(int(result.valid_mask[point[1], point[0]]), 0)
        self.assertEqual(int(result.valid_mask[nostril[1], nostril[0]]), 0)
        self.assertEqual(int(result.valid_mask[mouth[1], mouth[0]]), 0)
        self.assertEqual(int(result.valid_mask[cheek_mole[1], cheek_mole[0]]), 255)
        self.assertTrue(
            any(
                np.hypot(
                    item.center[0] - cheek_mole[0],
                    item.center[1] - cheek_mole[1],
                )
                <= 5
                for item in result.candidates
            ),
            [item.center for item in result.candidates],
        )

    def test_eyes_only_full_head_fallback_does_not_cover_close_cheek_mole(self):
        # Reproduces the alignment pose where the skin locator spans the full
        # image height.  If dynamic eye extraction fails for one frame, the
        # fixed eye fallback must still cover the eye but stop above the mole.
        face = FaceBox(0, 0, 329, 480, 1.0, "skin")
        _face_mask, valid_mask, excluded = build_facial_masks(
            (480, 640, 3),
            face,
            detection_exclusion_mode="eyes_only",
        )
        right_eye = (245, 175)
        cheek_mole = (295, 254)

        self.assertGreater(
            int(excluded["right_eye_guard"][right_eye[1], right_eye[0]]),
            0,
        )
        self.assertEqual(int(valid_mask[right_eye[1], right_eye[0]]), 0)
        self.assertEqual(
            int(excluded["right_eye_guard"][cheek_mole[1], cheek_mole[0]]),
            0,
        )
        self.assertEqual(int(valid_mask[cheek_mole[1], cheek_mole[0]]), 255)

    def test_near_skin_head_eye_guard_does_not_cover_upper_cheek_mole(self):
        # Current Qt close-up: the visible skin face can be narrow and fill the
        # frame height.  The right cheek mole sits just below the eye band, so
        # the eyes-only safety mask must not extend down to it.
        face = FaceBox(0, 0, 281, 462, 1.0, "skin")
        _face_mask, valid_mask, excluded = build_facial_masks(
            (462, 616, 3),
            face,
            detection_exclusion_mode="eyes_only",
        )
        right_eye = (200, 175)
        upper_cheek_mole = (243, 222)

        self.assertGreater(
            int(excluded["right_eye_guard"][right_eye[1], right_eye[0]]),
            0,
        )
        self.assertEqual(int(valid_mask[right_eye[1], right_eye[0]]), 0)
        self.assertEqual(
            int(
                excluded["right_eye_guard"][
                    upper_cheek_mole[1], upper_cheek_mole[0]
                ]
            ),
            0,
        )
        self.assertEqual(
            int(valid_mask[upper_cheek_mole[1], upper_cheek_mole[0]]),
            255,
        )

    def test_side_closeup_eye_guard_does_not_swallow_lower_cheek_mole(self):
        frame = np.full((480, 640, 3), (205, 205, 205), dtype=np.uint8)
        cv2.ellipse(
            frame,
            (420, 255),
            (205, 225),
            0,
            0,
            360,
            (95, 145, 205),
            -1,
        )
        cv2.ellipse(frame, (505, 235), (45, 15), 0, 0, 360, (20, 20, 25), -1)
        cv2.ellipse(frame, (505, 187), (48, 7), 0, 0, 360, (45, 45, 50), -1)
        cheek_mole = (556, 291)
        # A circular printed mole becomes roughly 8x15 px in this side view.
        cv2.ellipse(frame, cheek_mole, (4, 8), 0, 0, 360, (15, 15, 20), -1)
        face = FaceBox(242, 95, 375, 375, 1.0, "haar")
        detector = FacialCandidateDetector(
            FixedFaceLocator(face),
            DetectParams(
                min_area=8,
                max_area=220,
                min_circularity=0.62,
                min_solidity=0.85,
                max_aspect_ratio=1.8,
                blackhat_ksize=19,
                min_contrast=28,
                use_skin_mask=True,
            ),
        )

        result = detector.detect(frame, render=False)

        self.assertGreater(int(result.excluded_masks["right_eye_guard"][235, 505]), 0)
        self.assertEqual(int(result.valid_mask[235, 505]), 0)
        self.assertEqual(
            int(result.excluded_masks["right_eye_guard"][cheek_mole[1], cheek_mole[0]]),
            0,
        )
        self.assertEqual(int(result.valid_mask[cheek_mole[1], cheek_mole[0]]), 255)
        self.assertTrue(
            any(
                np.hypot(
                    item.center[0] - cheek_mole[0],
                    item.center[1] - cheek_mole[1],
                )
                <= 5
                for item in result.candidates
            ),
            [item.center for item in result.candidates],
        )

    def test_side_closeup_keeps_mole_near_visible_skin_edge(self):
        frame = np.full((480, 640, 3), (205, 205, 205), dtype=np.uint8)
        cv2.ellipse(
            frame,
            (420, 255),
            (205, 225),
            0,
            0,
            360,
            (95, 145, 205),
            -1,
        )
        edge_mole = (610, 291)
        cv2.circle(frame, edge_mole, 6, (15, 15, 20), -1)

        _face_mask, valid_mask, excluded = build_facial_masks(
            frame.shape,
            FaceBox(242, 95, 375, 375, 1.0, "haar"),
            frame_bgr=frame,
        )

        self.assertEqual(int(valid_mask[edge_mole[1], edge_mole[0]]), 255)
        self.assertFalse(
            any(int(mask[edge_mole[1], edge_mole[0]]) > 0 for mask in excluded.values())
        )

    def test_privacy_view_keeps_original_shape(self):
        frame = np.full((480, 640, 3), 160, dtype=np.uint8)
        detector = FacialCandidateDetector(FixedFaceLocator(self.face))
        result = detector.detect(frame)
        self.assertEqual(result.privacy_view.shape, frame.shape)
        self.assertEqual(result.annotated.shape, frame.shape)

    def test_skin_silhouette_extends_above_detector_box_and_removes_background(self):
        frame = np.full((480, 640, 3), (210, 210, 210), dtype=np.uint8)
        cv2.ellipse(frame, (320, 220), (130, 185), 0, 0, 360, (95, 145, 205), -1)
        detector_box = FaceBox(220, 150, 200, 250, 1.0, "test")
        detector = FacialCandidateDetector(FixedFaceLocator(detector_box))
        result = detector.detect(frame)
        x, y, width, height = cv2.boundingRect(result.face_mask)
        self.assertLess(y, detector_box.y)
        self.assertGreater(height, detector_box.height)
        self.assertTrue(np.array_equal(result.privacy_view[10, 10], np.zeros(3, dtype=np.uint8)))

    def test_detection_uses_full_valid_mask_not_only_detector_box(self):
        frame = np.full((480, 640, 3), (205, 205, 205), dtype=np.uint8)
        cv2.ellipse(frame, (320, 245), (145, 185), 0, 0, 360, (95, 145, 205), -1)
        detector_box = FaceBox(220, 145, 200, 250, 1.0, "test")
        spot_outside_detector_box = (205, 285)
        cv2.circle(frame, spot_outside_detector_box, 7, (20, 20, 25), -1)
        detector = FacialCandidateDetector(
            FixedFaceLocator(detector_box),
            DetectParams(
                min_area=20,
                max_area=500,
                min_circularity=0.4,
                min_solidity=0.7,
                max_aspect_ratio=2.4,
                blackhat_ksize=31,
                min_contrast=12,
                use_skin_mask=True,
            ),
        )

        result = detector.detect(frame)

        self.assertLess(spot_outside_detector_box[0], detector_box.x)
        self.assertEqual(int(result.valid_mask[285, 205]), 255)
        self.assertTrue(
            any(
                np.hypot(
                    item.center[0] - spot_outside_detector_box[0],
                    item.center[1] - spot_outside_detector_box[1],
                )
                <= 5
                for item in result.candidates
            ),
            [item.center for item in result.candidates],
        )

    def test_profile_cheek_is_not_removed_by_frontal_nostril_mask(self):
        frame = np.full((480, 640, 3), (205, 205, 205), dtype=np.uint8)
        cv2.ellipse(frame, (320, 245), (145, 185), 0, 0, 360, (95, 145, 205), -1)
        profile_face = FaceBox(170, 60, 300, 360, 1.0, "haar-profile")
        cheek_spot = (325, 285)
        cv2.circle(frame, cheek_spot, 7, (20, 20, 25), -1)
        detector = FacialCandidateDetector(
            FixedFaceLocator(profile_face),
            DetectParams(
                min_area=20,
                max_area=500,
                min_circularity=0.4,
                min_solidity=0.7,
                max_aspect_ratio=2.4,
                blackhat_ksize=31,
                min_contrast=12,
                use_skin_mask=True,
            ),
        )

        result = detector.detect(frame)

        self.assertEqual(int(result.valid_mask[cheek_spot[1], cheek_spot[0]]), 255)
        self.assertTrue(
            any(
                np.hypot(
                    item.center[0] - cheek_spot[0],
                    item.center[1] - cheek_spot[1],
                )
                <= 5
                for item in result.candidates
            ),
            [item.center for item in result.candidates],
        )

    def test_frontal_upper_edge_guard_removes_temple_artifact(self):
        frame = np.full((480, 640, 3), (205, 205, 205), dtype=np.uint8)
        cv2.ellipse(frame, (320, 245), (145, 185), 0, 0, 360, (95, 145, 205), -1)
        frontal_face = FaceBox(170, 60, 300, 360, 1.0, "haar")
        temple_artifact = (190, 245)
        cheek_spot = (395, 285)
        cv2.circle(frame, temple_artifact, 7, (20, 20, 25), -1)
        cv2.circle(frame, cheek_spot, 7, (20, 20, 25), -1)
        detector = FacialCandidateDetector(
            FixedFaceLocator(frontal_face),
            DetectParams(
                min_area=20,
                max_area=500,
                min_circularity=0.4,
                min_solidity=0.7,
                max_aspect_ratio=2.4,
                blackhat_ksize=31,
                min_contrast=12,
                use_skin_mask=True,
            ),
        )

        result = detector.detect(frame)
        centers = [item.center for item in result.candidates]

        self.assertEqual(int(result.valid_mask[temple_artifact[1], temple_artifact[0]]), 0)
        self.assertFalse(
            any(
                np.hypot(x - temple_artifact[0], y - temple_artifact[1]) <= 8
                for x, y in centers
            ),
            centers,
        )
        self.assertTrue(
            any(
                np.hypot(x - cheek_spot[0], y - cheek_spot[1]) <= 5
                for x, y in centers
            ),
            centers,
        )

    def test_frontal_upper_edge_guard_stops_before_lower_outer_cheek(self):
        frame = np.full((480, 640, 3), (205, 205, 205), dtype=np.uint8)
        cv2.ellipse(
            frame,
            (236, 240),
            (177, 240),
            0,
            0,
            360,
            (95, 145, 205),
            -1,
        )
        cheek_spot = (390, 272)
        face = FaceBox(37, 45, 417, 417, 1.0, "haar")

        _face_mask, valid_mask, excluded = build_facial_masks(
            frame.shape,
            face,
            frame_bgr=frame,
        )

        self.assertEqual(
            int(excluded["upper_edge_guard"][cheek_spot[1], cheek_spot[0]]),
            0,
        )
        self.assertEqual(int(valid_mask[cheek_spot[1], cheek_spot[0]]), 255)

    def test_single_frame_false_positive_is_not_confirmed(self):
        tracker = CandidateStabilityTracker(required_hits=4)
        spot = DetectedMole((200, 200), 5, 70.0, 0.8, 30.0)
        self.assertEqual(spot.track_id, 0)
        self.assertEqual(spot.stable_count, 0)
        self.assertEqual(spot.required_count, 0)
        self.assertEqual(spot.track_age_s, 0.0)
        self.assertEqual(tracker.update([spot]), [])
        self.assertEqual(tracker.update([]), [])
        self.assertEqual(tracker.update([]), [])

    def test_stable_candidate_is_confirmed_after_required_frames(self):
        tracker = CandidateStabilityTracker(
            required_hits=4,
            stable_radius=4.0,
        )
        outputs = []
        for center in ((200, 200), (201, 200), (200, 201), (201, 201)):
            outputs = tracker.update(
                [DetectedMole(center, 5, 70.0, 0.8, 30.0)]
            )
        self.assertEqual(len(outputs), 1)
        self.assertLessEqual(np.hypot(outputs[0].center[0] - 200, outputs[0].center[1] - 200), 2)
        self.assertEqual(outputs[0].track_id, 1)
        self.assertEqual(outputs[0].stable_count, 4)
        self.assertEqual(outputs[0].required_count, 4)
        self.assertEqual(outputs[0].track_age_s, 0.0)

        outputs = tracker.update(
            [DetectedMole((200, 200), 5, 70.0, 0.8, 30.0)]
        )
        self.assertEqual(outputs[0].track_id, 1)
        self.assertEqual(outputs[0].stable_count, 5)

    def test_small_fragment_radius_changes_keep_the_same_track(self):
        tracker = CandidateStabilityTracker(
            required_hits=4,
            stable_radius=6.0,
            min_stable_duration_s=0.30,
            max_radius_ratio=1.5,
        )
        samples = (
            ((390, 270), 3),
            ((393, 273), 8),
            ((391, 271), 4),
            ((393, 273), 8),
        )
        output = []
        for sample_time, (center, radius) in zip((0.0, 0.1, 0.2, 0.3), samples):
            output = tracker.update(
                [DetectedMole(center, radius, 70.0, 0.5, 60.0)],
                sample_time=sample_time,
            )

        self.assertEqual(len(output), 1)
        self.assertEqual(output[0].track_id, 1)

    def test_tracking_metadata_preserves_first_seen_time(self):
        tracker = CandidateStabilityTracker(
            required_hits=3,
            stable_radius=4.0,
        )
        spot = DetectedMole((200, 200), 5, 70.0, 0.8, 30.0)

        self.assertEqual(tracker.update([spot], sample_time=10.0), [])
        self.assertEqual(tracker.update([spot], sample_time=10.2), [])
        outputs = tracker.update([spot], sample_time=10.6)

        self.assertEqual(len(outputs), 1)
        self.assertEqual(outputs[0].track_id, 1)
        self.assertEqual(outputs[0].stable_count, 3)
        self.assertEqual(outputs[0].required_count, 3)
        self.assertAlmostEqual(outputs[0].track_age_s, 0.6)

    def test_missed_frame_restarts_consecutive_hit_count(self):
        tracker = CandidateStabilityTracker(
            required_hits=3,
            stable_radius=4.0,
            max_missed=2,
        )
        spot = DetectedMole((200, 200), 5, 70.0, 0.8, 30.0)

        tracker.update([spot], sample_time=1.0)
        tracker.update([spot], sample_time=1.1)
        tracker.update([], sample_time=1.2)
        self.assertEqual(tracker.update([spot], sample_time=1.3), [])
        self.assertEqual(tracker.update([spot], sample_time=1.4), [])
        outputs = tracker.update([spot], sample_time=1.5)

        self.assertEqual(len(outputs), 1)
        self.assertEqual(outputs[0].track_id, 1)
        self.assertEqual(outputs[0].stable_count, 3)
        self.assertAlmostEqual(outputs[0].track_age_s, 0.5)

    def test_short_dropout_preserves_track_id_without_early_stable_output(self):
        tracker = CandidateStabilityTracker(
            required_hits=4,
            stable_radius=6.0,
            max_missed=6,
            min_stable_duration_s=0.35,
        )
        original = DetectedMole((452, 204), 7, 150.0, 0.8, 60.0)
        returned = DetectedMole((430, 198), 7, 150.0, 0.8, 60.0)

        initial = []
        for sample_time in (1.00, 1.12, 1.24, 1.36):
            initial = tracker.update([original], sample_time=sample_time)
        self.assertEqual(len(initial), 1)
        original_track_id = initial[0].track_id

        for sample_time in (1.48, 1.60, 1.72, 1.84):
            self.assertEqual(tracker.update([], sample_time=sample_time), [])

        # The returning point moved about 22.8 px.  With radius 7 this remains
        # inside the tracker's 28 px dynamic association gate, but the gap must
        # restart both the consecutive-hit and elapsed-stability gates.
        for sample_time in (1.96, 2.08, 2.20):
            self.assertEqual(
                tracker.update([returned], sample_time=sample_time),
                [],
            )
        reacquired = tracker.update([returned], sample_time=2.32)

        self.assertEqual(len(reacquired), 1)
        self.assertEqual(reacquired[0].track_id, original_track_id)
        self.assertEqual(reacquired[0].stable_count, 4)

    def test_face_relative_association_preserves_id_after_large_camera_shift(self):
        tracker = CandidateStabilityTracker(
            required_hits=4,
            stable_radius=6.0,
            max_missed=45,
            max_gap_s=1.5,
            min_stable_duration_s=0.35,
        )
        first_box = (100, 50, 300, 360)
        shifted_box = (160, 50, 300, 360)
        original = DetectedMole((310, 230), 7, 150.0, 0.8, 60.0)
        shifted = DetectedMole((370, 230), 7, 150.0, 0.8, 60.0)

        initial = []
        for sample_time in (1.00, 1.12, 1.24, 1.36):
            initial = tracker.update(
                [original],
                sample_time=sample_time,
                reference_box=first_box,
            )
        self.assertEqual(len(initial), 1)
        original_track_id = initial[0].track_id

        for sample_time in (1.48, 1.60, 1.72, 1.84):
            self.assertEqual(
                tracker.update(
                    [],
                    sample_time=sample_time,
                    reference_box=shifted_box,
                ),
                [],
            )

        # The raw 60 px jump is outside the old 28 px gate.  It is zero in
        # face-relative coordinates, so identity is retained, but the gap
        # still requires a fresh 4-frame/0.35-second stability confirmation.
        for sample_time in (1.96, 2.08, 2.20):
            self.assertEqual(
                tracker.update(
                    [shifted],
                    sample_time=sample_time,
                    reference_box=shifted_box,
                ),
                [],
            )
        reacquired = tracker.update(
            [shifted],
            sample_time=2.32,
            reference_box=shifted_box,
        )

        self.assertEqual(len(reacquired), 1)
        self.assertEqual(reacquired[0].track_id, original_track_id)
        self.assertEqual(reacquired[0].stable_count, 4)

    def test_face_relative_association_rejects_another_anatomical_position(self):
        tracker = CandidateStabilityTracker(
            required_hits=4,
            stable_radius=6.0,
            max_missed=45,
            max_gap_s=1.5,
            min_stable_duration_s=0.35,
        )
        first_box = (100, 50, 300, 360)
        shifted_box = (160, 50, 300, 360)
        original = DetectedMole((310, 230), 7, 150.0, 0.8, 60.0)
        # This different facial position deliberately lands on the old raw
        # screen coordinate.  Raw-pixel fallback would incorrectly keep ID 1;
        # face-relative association must reject it after the face shifts.
        other_position = DetectedMole((310, 230), 7, 150.0, 0.8, 60.0)

        initial = []
        for sample_time in (1.00, 1.12, 1.24, 1.36):
            initial = tracker.update(
                [original],
                sample_time=sample_time,
                reference_box=first_box,
            )
        original_track_id = initial[0].track_id
        tracker.update([], sample_time=1.48, reference_box=shifted_box)

        output = []
        for sample_time in (1.60, 1.72, 1.84, 1.96):
            output = tracker.update(
                [other_position],
                sample_time=sample_time,
                reference_box=shifted_box,
            )

        self.assertEqual(len(output), 1)
        self.assertNotEqual(output[0].track_id, original_track_id)

    def test_face_relative_association_rejects_radius_change(self):
        tracker = CandidateStabilityTracker(
            required_hits=2,
            stable_radius=6.0,
            max_missed=45,
            max_gap_s=1.5,
            max_radius_ratio=1.5,
        )
        first_box = (100, 50, 300, 360)
        shifted_box = (160, 50, 300, 360)
        original = DetectedMole((310, 230), 7, 150.0, 0.8, 60.0)
        wrong_scale = DetectedMole((370, 230), 12, 400.0, 0.8, 60.0)

        tracker.update([original], sample_time=1.0, reference_box=first_box)
        initial = tracker.update(
            [original],
            sample_time=1.1,
            reference_box=first_box,
        )
        original_track_id = initial[0].track_id
        tracker.update([], sample_time=1.2, reference_box=shifted_box)
        tracker.update([wrong_scale], sample_time=1.3, reference_box=shifted_box)
        output = tracker.update(
            [wrong_scale],
            sample_time=1.4,
            reference_box=shifted_box,
        )

        self.assertEqual(len(output), 1)
        self.assertNotEqual(output[0].track_id, original_track_id)

    def test_time_gap_expires_track_even_with_large_frame_budget(self):
        tracker = CandidateStabilityTracker(
            required_hits=2,
            stable_radius=6.0,
            max_missed=45,
            max_gap_s=1.0,
        )
        box = (100, 50, 300, 360)
        spot = DetectedMole((310, 230), 7, 150.0, 0.8, 60.0)

        tracker.update([spot], sample_time=1.0, reference_box=box)
        initial = tracker.update([spot], sample_time=1.1, reference_box=box)
        original_track_id = initial[0].track_id
        tracker.update([], sample_time=2.2, reference_box=box)
        tracker.update([spot], sample_time=2.3, reference_box=box)
        output = tracker.update([spot], sample_time=2.4, reference_box=box)

        self.assertEqual(len(output), 1)
        self.assertNotEqual(output[0].track_id, original_track_id)

    def test_dropout_beyond_max_missed_creates_new_track_id(self):
        tracker = CandidateStabilityTracker(
            required_hits=4,
            stable_radius=6.0,
            max_missed=6,
            min_stable_duration_s=0.35,
        )
        spot = DetectedMole((452, 204), 7, 150.0, 0.8, 60.0)

        initial = []
        for sample_time in (10.00, 10.12, 10.24, 10.36):
            initial = tracker.update([spot], sample_time=sample_time)
        self.assertEqual(len(initial), 1)
        original_track_id = initial[0].track_id

        # A track is retained through six misses and removed on the seventh.
        for sample_time in (10.48, 10.60, 10.72, 10.84, 10.96, 11.08, 11.20):
            self.assertEqual(tracker.update([], sample_time=sample_time), [])

        for sample_time in (11.32, 11.44, 11.56):
            self.assertEqual(tracker.update([spot], sample_time=sample_time), [])
        reacquired = tracker.update([spot], sample_time=11.68)

        self.assertEqual(len(reacquired), 1)
        self.assertNotEqual(reacquired[0].track_id, original_track_id)
        self.assertEqual(reacquired[0].track_id, 2)
        self.assertEqual(reacquired[0].stable_count, 4)

    def test_time_gate_prevents_high_fps_candidate_from_locking_too_soon(self):
        tracker = CandidateStabilityTracker(
            required_hits=4,
            stable_radius=4.0,
            min_stable_duration_s=0.35,
        )
        spot = DetectedMole((200, 200), 5, 70.0, 0.8, 30.0)

        outputs = []
        for sample_time in (0.00, 0.05, 0.10, 0.15, 0.30):
            outputs = tracker.update([spot], sample_time=sample_time)
            self.assertEqual(outputs, [])

        outputs = tracker.update([spot], sample_time=0.35)
        self.assertEqual(len(outputs), 1)

    def test_time_gate_restarts_after_candidate_is_missed(self):
        tracker = CandidateStabilityTracker(
            required_hits=2,
            stable_radius=4.0,
            max_missed=2,
            min_stable_duration_s=0.30,
        )
        spot = DetectedMole((200, 200), 5, 70.0, 0.8, 30.0)

        tracker.update([spot], sample_time=0.00)
        tracker.update([spot], sample_time=0.20)
        tracker.update([], sample_time=0.25)
        self.assertEqual(tracker.update([spot], sample_time=0.35), [])
        self.assertEqual(tracker.update([spot], sample_time=0.60), [])
        self.assertEqual(len(tracker.update([spot], sample_time=0.65)), 1)

    def test_confirmed_track_follows_closed_loop_motion_without_relocking(self):
        tracker = CandidateStabilityTracker(
            required_hits=4,
            stable_radius=6.0,
            min_stable_duration_s=0.35,
        )
        output = []
        for sample_time in (0.00, 0.12, 0.24, 0.36):
            output = tracker.update(
                [DetectedMole((200, 200), 7, 140.0, 0.8, 60.0)],
                sample_time=sample_time,
            )
        self.assertEqual(len(output), 1)
        track_id = output[0].track_id

        for index, center_x in enumerate((205, 210, 215, 220), start=1):
            output = tracker.update(
                [DetectedMole((center_x, 200), 7, 140.0, 0.8, 60.0)],
                sample_time=0.36 + index * 0.12,
            )
            self.assertEqual(len(output), 1)
            self.assertEqual(output[0].track_id, track_id)
            self.assertEqual(output[0].center, (center_x, 200))

    def test_confirmed_track_recovers_after_nearby_unconfirmed_duplicate(self):
        tracker = CandidateStabilityTracker(
            required_hits=4,
            stable_radius=6.0,
            max_missed=45,
            fast_reacquire_hits=2,
            fast_reacquire_min_duration_s=0.10,
            fast_reacquire_max_gap_s=0.45,
        )
        target = DetectedMole((100, 100), 5, 70.0, 0.8, 40.0)
        output = []
        for sample_time in (0.00, 0.12, 0.24, 0.36):
            output = tracker.update([target], sample_time=sample_time)
        self.assertEqual(len(output), 1)
        target_track_id = output[0].track_id

        # The second contour is far enough away to create an unconfirmed
        # bookkeeping track without disturbing the selected target.
        duplicate_seed = DetectedMole((130, 100), 5, 70.0, 0.8, 40.0)
        output = tracker.update(
            [target, duplicate_seed],
            sample_time=0.48,
        )
        self.assertEqual(output[0].track_id, target_track_id)

        # This single contour lies equally close to both internal tracks.  It
        # is unsafe for this frame, so no coordinate is published.  The
        # never-confirmed duplicate is removed instead of being allowed to
        # starve the trusted identity forever.
        between = DetectedMole((115, 100), 5, 70.0, 0.8, 40.0)
        output = tracker.update([between], sample_time=0.60)
        self.assertEqual(output, [])
        self.assertEqual(len(tracker._tracks), 1)
        self.assertEqual(tracker._tracks[0].track_id, target_track_id)

        # Recovery still requires two new observations and 0.10 seconds; the
        # ambiguous sample above is never reused as a valid coordinate.
        self.assertEqual(
            tracker.update([between], sample_time=0.72),
            [],
        )
        output = tracker.update([between], sample_time=0.84)
        self.assertEqual(len(output), 1)
        self.assertEqual(output[0].track_id, target_track_id)
        self.assertEqual(output[0].center, between.center)

    def test_two_confirmed_tracks_remain_fail_closed_when_equally_plausible(self):
        tracker = CandidateStabilityTracker(
            required_hits=2,
            stable_radius=6.0,
        )
        left = DetectedMole((100, 100), 5, 70.0, 0.8, 40.0)
        right = DetectedMole((130, 100), 5, 70.0, 0.8, 40.0)
        tracker.update([left, right])
        output = tracker.update([left, right])
        self.assertEqual(len(output), 2)

        # One observation equally close to two real, confirmed identities is
        # still unsafe to assign.  Trust priority must not guess between them.
        between = DetectedMole((115, 100), 5, 70.0, 0.8, 40.0)
        self.assertEqual(tracker.update([between]), [])
        self.assertEqual(len(tracker._tracks), 2)

    def test_ambiguous_contours_do_not_spawn_tracks_forever(self):
        tracker = CandidateStabilityTracker(
            required_hits=2,
            stable_radius=6.0,
            max_missed=45,
            fast_reacquire_hits=2,
            fast_reacquire_min_duration_s=0.10,
            fast_reacquire_max_gap_s=0.45,
        )
        center = DetectedMole((200, 100), 5, 70.0, 0.8, 40.0)
        tracker.update([center], sample_time=0.00)
        initial = tracker.update([center], sample_time=0.12)
        track_id = initial[0].track_id

        # Two nearly identical contours make assignment unsafe.  Neither may
        # become a fresh track merely because the frame was ambiguous.
        left = DetectedMole((199, 100), 5, 70.0, 0.8, 40.0)
        right = DetectedMole((202, 100), 5, 70.0, 0.8, 40.0)
        self.assertEqual(
            tracker.update([left, right], sample_time=0.24),
            [],
        )
        self.assertEqual(len(tracker._tracks), 1)

        self.assertEqual(tracker.update([center], sample_time=0.36), [])
        recovered = tracker.update([center], sample_time=0.48)
        self.assertEqual(len(recovered), 1)
        self.assertEqual(recovered[0].track_id, track_id)
        self.assertEqual(len(tracker._tracks), 1)

    def test_confirmed_track_fast_reacquires_from_fresh_frames_after_short_gap(self):
        tracker = CandidateStabilityTracker(
            required_hits=4,
            stable_radius=6.0,
            min_stable_duration_s=0.35,
            max_missed=45,
            max_gap_s=1.5,
            fast_reacquire_hits=2,
            fast_reacquire_min_duration_s=0.10,
            fast_reacquire_max_gap_s=0.45,
        )
        box = (100, 50, 300, 360)
        original = DetectedMole((310, 230), 7, 150.0, 0.8, 60.0)
        shifted = DetectedMole((370, 230), 7, 150.0, 0.8, 60.0)

        output = []
        for sample_time in (1.00, 1.12, 1.24, 1.36):
            output = tracker.update(
                [original], sample_time=sample_time, reference_box=box
            )
        self.assertEqual(len(output), 1)
        track_id = output[0].track_id

        shifted_box = (160, 50, 300, 360)
        self.assertEqual(
            tracker.update([], sample_time=1.48, reference_box=shifted_box),
            [],
        )
        # The first returning observation is fresh but is not enough by itself.
        self.assertEqual(
            tracker.update(
                [shifted], sample_time=1.60, reference_box=shifted_box
            ),
            [],
        )
        reacquired = tracker.update(
            [shifted], sample_time=1.72, reference_box=shifted_box
        )
        self.assertEqual(len(reacquired), 1)
        self.assertEqual(reacquired[0].track_id, track_id)
        self.assertEqual(reacquired[0].required_count, 2)

    def test_fast_reacquire_is_not_used_after_long_gap(self):
        tracker = CandidateStabilityTracker(
            required_hits=4,
            stable_radius=6.0,
            min_stable_duration_s=0.35,
            max_missed=45,
            max_gap_s=1.5,
            fast_reacquire_hits=2,
            fast_reacquire_min_duration_s=0.10,
            fast_reacquire_max_gap_s=0.45,
        )
        spot = DetectedMole((200, 200), 7, 150.0, 0.8, 60.0)
        for sample_time in (1.00, 1.12, 1.24, 1.36):
            output = tracker.update([spot], sample_time=sample_time)
        self.assertEqual(len(output), 1)

        self.assertEqual(tracker.update([], sample_time=1.48), [])
        # 0.64 s since the last observed target: use the full initial gate.
        for sample_time in (2.00, 2.12, 2.24):
            self.assertEqual(tracker.update([spot], sample_time=sample_time), [])
        reacquired = tracker.update([spot], sample_time=2.36)
        self.assertEqual(len(reacquired), 1)
        self.assertEqual(reacquired[0].required_count, 4)


if __name__ == "__main__":
    unittest.main()
