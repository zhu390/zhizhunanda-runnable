import unittest

from jetson_nano_vision.jetson_vision.scene_guard import FaceSceneGuard


class FaceSceneGuardTest(unittest.TestCase):
    def setUp(self):
        self.guard = FaceSceneGuard(
            center_shift_ratio=0.08,
            scale_ratio=1.25,
            missing_timeout_s=0.8,
        )
        self.assertTrue(self.guard.latch((200, 100, 400, 300), (640, 480), 10.0))

    def test_stable_face_is_accepted(self):
        decision = self.guard.update((205, 104, 405, 304), (640, 480), 10.2)
        self.assertFalse(decision.abort)
        self.assertTrue(decision.consumed)

    def test_large_center_shift_requests_abort(self):
        decision = self.guard.update((300, 100, 500, 300), (640, 480), 10.2)
        self.assertTrue(decision.abort)
        self.assertIn("中心位移", decision.reason)

    def test_large_scale_change_requests_abort(self):
        decision = self.guard.update((170, 70, 430, 330), (640, 480), 10.2)
        self.assertTrue(decision.abort)
        self.assertIn("尺度变化", decision.reason)

    def test_missing_face_must_be_continuous(self):
        first = self.guard.update(None, (640, 480), 10.2)
        second = self.guard.update(None, (640, 480), 10.9)
        third = self.guard.update(None, (640, 480), 11.01)
        self.assertFalse(first.abort)
        self.assertFalse(second.abort)
        self.assertTrue(third.abort)

    def test_duplicate_sample_does_not_advance_missing_timer(self):
        first = self.guard.update(None, (640, 480), 10.2)
        duplicate = self.guard.update(None, (640, 480), 10.2)
        self.assertTrue(first.consumed)
        self.assertFalse(duplicate.consumed)
        self.assertFalse(duplicate.abort)

    def test_stream_stall_requests_abort(self):
        self.assertFalse(self.guard.poll(10.79).abort)
        self.assertTrue(self.guard.poll(10.81).abort)


if __name__ == "__main__":
    unittest.main()
