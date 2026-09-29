import unittest

from jetson_nano_vision.jetson_vision.target_lock import TargetIdentityLock


class TargetIdentityLockTest(unittest.TestCase):
    def setUp(self):
        self.lock = TargetIdentityLock(
            max_step_ratio=0.10,
            max_radius_ratio=1.5,
            stable_frames=2,
        )

    def _acquire(self, candidates, first_time=1.0):
        pending = self.lock.update(candidates, (640, 480), first_time)
        self.assertIsNone(pending.index)
        acquired = self.lock.update(candidates, (640, 480), first_time + 0.1)
        self.assertIsNotNone(acquired.index)
        return acquired

    def test_result_reordering_does_not_switch_target(self):
        first = self._acquire([(100, 100, 8), (300, 100, 7)])
        reordered = self.lock.update([(302, 101, 7), (104, 102, 8)], (640, 480), 1.2)
        self.assertEqual(first.index, 0)
        self.assertEqual(reordered.index, 1)

    def test_missing_target_does_not_acquire_distant_candidate(self):
        self._acquire([(100, 100, 8)])
        missing = self.lock.update([(400, 350, 8)], (640, 480), 1.2)
        self.assertIsNone(missing.index)
        self.assertTrue(missing.locked)

    def test_original_target_can_be_reacquired_after_missing_frame(self):
        self._acquire([(100, 100, 8)])
        self.assertIsNone(self.lock.update([], (640, 480), 1.2).index)
        confirming = self.lock.update([(102, 99, 8), (400, 350, 8)], (640, 480), 1.3)
        self.assertIsNone(confirming.index)
        reacquired = self.lock.update([(103, 99, 8), (400, 350, 8)], (640, 480), 1.4)
        self.assertEqual(reacquired.index, 0)

    def test_radius_mismatch_is_rejected(self):
        self._acquire([(100, 100, 5)])
        mismatch = self.lock.update([(102, 101, 20)], (640, 480), 1.2)
        self.assertIsNone(mismatch.index)

    def test_duplicate_sample_is_not_consumed(self):
        self._acquire([(100, 100, 8)])
        duplicate = self.lock.update([(101, 100, 8)], (640, 480), 1.1)
        self.assertFalse(duplicate.consumed)

    def test_missing_original_does_not_switch_to_nearby_substitute(self):
        self._acquire([(100, 100, 8), (112, 100, 8)])

        # Seven pixels is inside the ordinary radius-bound continuation gate;
        # it must still be rejected because this candidate was already seen
        # as a different mole during initial lock acquisition.
        first_substitute = self.lock.update([(107, 100, 8)], (640, 480), 1.2)
        repeated_substitute = self.lock.update([(107, 100, 8)], (640, 480), 1.3)

        self.assertIsNone(first_substitute.index)
        self.assertIsNone(repeated_substitute.index)
        self.assertTrue(repeated_substitute.locked)

    def test_two_candidates_inside_identity_gate_are_ambiguous(self):
        self._acquire([(100, 100, 8)])

        ambiguous = self.lock.update(
            [(102, 100, 8), (105, 100, 8)],
            (640, 480),
            1.2,
        )

        self.assertIsNone(ambiguous.index)

    def test_external_track_id_survives_motion_larger_than_mole_radius(self):
        first = self.lock.update(
            [(100, 100, 7)],
            (640, 480),
            1.0,
            candidate_ids=[41],
        )
        self.assertIsNone(first.index)
        acquired = self.lock.update(
            [(101, 100, 7)],
            (640, 480),
            1.1,
            candidate_ids=[41],
        )
        self.assertEqual(acquired.index, 0)

        # Twenty pixels is well beyond the old one-radius (7 px) gate, but
        # remains the exact same upstream one-to-one track.
        continued = self.lock.update(
            [(121, 101, 7)],
            (640, 480),
            1.2,
            candidate_ids=[41],
        )
        self.assertEqual(continued.index, 0)
        self.assertEqual(self.lock.track_id, 41)
        self.assertFalse(self.lock.lost)

    def test_external_track_id_does_not_switch_to_another_candidate(self):
        self.lock.update(
            [(100, 100, 7)],
            (640, 480),
            1.0,
            candidate_ids=[41],
        )
        self.lock.update(
            [(101, 100, 7)],
            (640, 480),
            1.1,
            candidate_ids=[41],
        )

        missing = self.lock.update(
            [(103, 100, 7)],
            (640, 480),
            1.2,
            candidate_ids=[99],
        )

        self.assertIsNone(missing.index)
        self.assertTrue(self.lock.lost)
        self.assertEqual(self.lock.track_id, 41)
        self.assertIn("禁止切换", missing.reason)

    def test_external_track_id_conservatively_rebinds_nearby_candidate(self):
        self.lock.update(
            [(100, 100, 7)],
            (640, 480),
            1.0,
            candidate_ids=[41],
        )
        self.lock.update(
            [(101, 100, 7)],
            (640, 480),
            1.1,
            candidate_ids=[41],
        )
        self.lock.update([], (640, 480), 1.2, candidate_ids=[])

        confirming = self.lock.update(
            [(103, 101, 7)],
            (640, 480),
            1.5,
            candidate_ids=[99],
        )
        rebound = self.lock.update(
            [(104, 101, 7)],
            (640, 480),
            1.6,
            candidate_ids=[99],
        )

        self.assertIsNone(confirming.index)
        self.assertIn("确认 1/2", confirming.reason)
        self.assertEqual(rebound.index, 0)
        self.assertIn("41->99", rebound.reason)
        self.assertEqual(self.lock.track_id, 99)
        self.assertFalse(self.lock.lost)

    def test_external_track_id_rebind_rejects_distant_candidate(self):
        self.lock.update(
            [(100, 100, 7)],
            (640, 480),
            1.0,
            candidate_ids=[41],
        )
        self.lock.update(
            [(101, 100, 7)],
            (640, 480),
            1.1,
            candidate_ids=[41],
        )
        self.lock.update([], (640, 480), 1.2, candidate_ids=[])

        first = self.lock.update(
            [(130, 100, 7)],
            (640, 480),
            1.3,
            candidate_ids=[99],
        )
        repeated = self.lock.update(
            [(130, 100, 7)],
            (640, 480),
            1.4,
            candidate_ids=[99],
        )

        self.assertIsNone(first.index)
        self.assertIsNone(repeated.index)
        self.assertIn("远离旧锚点", repeated.reason)
        self.assertEqual(self.lock.track_id, 41)

    def test_external_track_id_rebind_requires_unique_candidate(self):
        self.lock.update(
            [(100, 100, 7)],
            (640, 480),
            1.0,
            candidate_ids=[41],
        )
        self.lock.update(
            [(101, 100, 7)],
            (640, 480),
            1.1,
            candidate_ids=[41],
        )
        self.lock.update([], (640, 480), 1.2, candidate_ids=[])

        ambiguous = self.lock.update(
            [(102, 100, 7), (109, 100, 7)],
            (640, 480),
            1.3,
            candidate_ids=[99, 100],
        )
        still_ambiguous = self.lock.update(
            [(103, 100, 7), (110, 100, 7)],
            (640, 480),
            1.4,
            candidate_ids=[99, 100],
        )

        self.assertIsNone(ambiguous.index)
        self.assertIsNone(still_ambiguous.index)
        self.assertIn("候选不唯一", still_ambiguous.reason)
        self.assertEqual(self.lock.pending_count, 0)
        self.assertEqual(self.lock.track_id, 41)

    def test_external_track_id_rebind_rejects_remembered_competitor(self):
        self.lock.update(
            [(100, 100, 7), (112, 100, 7)],
            (640, 480),
            1.0,
            candidate_ids=[41, 88],
        )
        self.lock.update(
            [(101, 100, 7), (112, 100, 7)],
            (640, 480),
            1.1,
            candidate_ids=[41, 88],
        )
        self.lock.update([], (640, 480), 1.2, candidate_ids=[])

        competitor = self.lock.update(
            [(112, 100, 7)],
            (640, 480),
            1.3,
            candidate_ids=[99],
        )

        self.assertIsNone(competitor.index)
        self.assertIn("已排除", competitor.reason)
        self.assertEqual(self.lock.track_id, 41)

    def test_external_track_id_rebind_expires_after_short_grace(self):
        self.lock.update(
            [(100, 100, 7)],
            (640, 480),
            1.0,
            candidate_ids=[41],
        )
        self.lock.update(
            [(101, 100, 7)],
            (640, 480),
            1.1,
            candidate_ids=[41],
        )
        self.lock.update([], (640, 480), 1.2, candidate_ids=[])

        expired = self.lock.update(
            [(102, 100, 7)],
            (640, 480),
            2.2,
            candidate_ids=[99],
        )

        self.assertIsNone(expired.index)
        self.assertIn("丢失超时", expired.reason)
        self.assertEqual(self.lock.track_id, 41)

    def test_external_track_id_reacquires_same_id_after_missing_frame(self):
        self.lock.update(
            [(100, 100, 7)],
            (640, 480),
            1.0,
            candidate_ids=[41],
        )
        self.lock.update(
            [(101, 100, 7)],
            (640, 480),
            1.1,
            candidate_ids=[41],
        )
        self.assertIsNone(
            self.lock.update([], (640, 480), 1.2, candidate_ids=[]).index
        )

        reacquired = self.lock.update(
            [(122, 101, 7)],
            (640, 480),
            1.3,
            candidate_ids=[41],
        )

        self.assertEqual(reacquired.index, 0)
        self.assertFalse(self.lock.lost)

    def test_external_track_id_cannot_inherit_a_remembered_competitor(self):
        self.lock.update(
            [(100, 100, 7), (120, 100, 7)],
            (640, 480),
            1.0,
            candidate_ids=[41, 99],
        )
        self.lock.update(
            [(101, 100, 7), (121, 100, 7)],
            (640, 480),
            1.1,
            candidate_ids=[41, 99],
        )

        inherited = self.lock.update(
            [(121, 100, 7)],
            (640, 480),
            1.2,
            candidate_ids=[41],
        )

        self.assertIsNone(inherited.index)
        self.assertIn("已排除", inherited.reason)


if __name__ == "__main__":
    unittest.main()
