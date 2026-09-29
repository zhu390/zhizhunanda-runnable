import sys
import types
import unittest
from unittest import mock

import numpy as np

from jetson_nano_vision.jetson_vision import camera_utils


class _FakeCapture:
    def __init__(self, frame_shape=(480, 640, 3), *, driver_fps=60.0):
        self.frame = np.zeros(frame_shape, dtype=np.uint8)
        self.read_count = 0
        self.released = False
        self.set_calls = []
        self.properties = {
            1: float(frame_shape[1]),
            2: float(frame_shape[0]),
            3: float(driver_fps),
            4: 0.0,
            10: 0.0,
            11: 64.0,
            12: 0.0,
        }

    def isOpened(self):
        return True

    def set(self, prop, value):
        self.set_calls.append((prop, float(value), self.read_count))
        # This fake reproduces the YHTek quirk: the driver reports 60 even
        # when the requested interval is 30, while other properties read back.
        if prop != 3:
            self.properties[prop] = float(value)
        return True

    def get(self, prop):
        return self.properties.get(prop, 0.0)

    def read(self):
        self.read_count += 1
        return True, self.frame.copy()

    def release(self):
        self.released = True


def _fake_cv2(capture, *, include_backlight=True):
    module = types.ModuleType("cv2")
    module.CAP_PROP_FRAME_WIDTH = 1
    module.CAP_PROP_FRAME_HEIGHT = 2
    module.CAP_PROP_FPS = 3
    module.CAP_PROP_FOURCC = 4
    module.CAP_PROP_BUFFERSIZE = 5
    module.CAP_PROP_BRIGHTNESS = 10
    module.CAP_PROP_GAIN = 11
    if include_backlight:
        module.CAP_PROP_BACKLIGHT = 12
    module.VideoCapture = lambda *_args: capture
    module.VideoWriter_fourcc = lambda *chars: sum(
        ord(char) << (8 * index) for index, char in enumerate(chars)
    )
    return module


class CameraUtilsTest(unittest.TestCase):
    def test_fourcc_is_set_before_mode_and_controls_after_first_frame(self):
        capture = _FakeCapture()
        fake_cv2 = _fake_cv2(capture)

        with mock.patch.dict(sys.modules, {"cv2": fake_cv2}):
            opened = camera_utils._try_open_camera(
                "0",
                640,
                480,
                30,
                "MJPG",
                99,
                brightness=8,
                gain=72,
                backlight_compensation=1,
            )

        self.assertIsNotNone(opened)
        _cap, label = opened
        first_props = [call[0] for call in capture.set_calls[:5]]
        self.assertEqual(first_props, [4, 1, 2, 3, 5])
        control_calls = [call for call in capture.set_calls if call[0] in (10, 11, 12)]
        self.assertTrue(all(call[2] >= 1 for call in control_calls))
        self.assertIn("req=640x480@30 MJPG", label)
        self.assertIn("driver_fps=60.0", label)
        self.assertIn("brightness=8", label)
        self.assertIn("gain=72", label)
        self.assertIn("backlight=1", label)

    def test_wrong_delivered_frame_size_is_rejected(self):
        capture = _FakeCapture(frame_shape=(720, 1280, 3))
        fake_cv2 = _fake_cv2(capture)

        with mock.patch.dict(sys.modules, {"cv2": fake_cv2}):
            opened = camera_utils._try_open_camera(
                "0",
                640,
                480,
                30,
                "MJPG",
                99,
            )

        self.assertIsNone(opened)
        self.assertTrue(capture.released)

    def test_unsupported_optional_control_does_not_block_camera(self):
        capture = _FakeCapture()
        fake_cv2 = _fake_cv2(capture, include_backlight=False)

        with mock.patch.dict(sys.modules, {"cv2": fake_cv2}):
            opened = camera_utils._try_open_camera(
                "0",
                640,
                480,
                30,
                "MJPG",
                99,
                backlight_compensation=1,
            )

        self.assertIsNotNone(opened)
        self.assertIn("backlight=unsupported", opened[1])

    def test_yuyv_and_yuy2_are_equivalent_driver_names(self):
        self.assertTrue(camera_utils._fourcc_matches("YUYV", "YUY2"))
        self.assertTrue(camera_utils._fourcc_matches("YUY2", "YUYV"))
        self.assertFalse(camera_utils._fourcc_matches("MJPG", "YUYV"))


if __name__ == "__main__":
    unittest.main()
