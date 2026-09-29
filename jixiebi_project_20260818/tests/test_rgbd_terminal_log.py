import io
import time
import unittest
from contextlib import redirect_stdout
from types import SimpleNamespace

from arm_mole_qt.app.main import (
    CameraCalibration,
    DepthPreview,
    MainWindow,
    _load_camera_calibration,
    parse_args,
)


class RgbdTerminalLogTest(unittest.TestCase):
    def _window(self, enabled=True):
        window = MainWindow.__new__(MainWindow)
        window.rgbd_distance_log_enabled = enabled
        window.rgbd_distance_log_interval = 10.0
        window._last_rgbd_distance_log_time = 0.0
        window._last_rgbd_distance_log_state = None
        window._latest_rgbd_detection_time = time.monotonic() - 0.01
        window._latest_rgbd_detections = {"arm": {}, "face": {}}
        window._astra_rgb_calibration = CameraCalibration(
            (500.0, 500.0, 320.0, 240.0),
            (0.0,) * 8,
            (640, 480),
            0.3,
        )
        return window

    def _preview(self):
        return DepthPreview(
            True,
            None,
            640,
            480,
            29.9,
            939.0,
            0.77,
            "Depth 正常",
            sample_time=time.monotonic() - 0.02,
            rgb_intrinsics=(500.0, 500.0, 320.0, 240.0),
            depth_to_rgb_rotation=object(),
            depth_to_rgb_translation=object(),
            depth_distortion=(0.0,) * 8,
            rgb_distortion=(0.0,) * 8,
            depth_distortion_mode="raw-explicit",
        )

    def test_cli_enables_distance_log(self):
        args = parse_args(["--rgbd-distance-log", "--rgbd-distance-log-interval", "0.25"])

        self.assertTrue(args.rgbd_distance_log)
        self.assertEqual(args.rgbd_distance_log_interval, 0.25)

    def test_default_v4l_rgb_calibration_loads_at_stream_resolution(self):
        args = parse_args([])

        calibration = _load_camera_calibration(args.astra_rgb_calibration)

        self.assertEqual(calibration.image_size, (640, 480))
        self.assertGreater(calibration.intrinsics[0], 0.0)
        self.assertEqual(len(calibration.distortion), 8)
        self.assertEqual(args.astra_depth_distortion_mode, "auto")

    def test_valid_log_contains_raw_filtered_and_quality_fields(self):
        window = self._window()
        result = SimpleNamespace(
            raw_distance_mm=329.4,
            filtered_distance_mm=327.2,
            sampled_minimum_mm=342.0,
            arm_cover_radius_mm=6.0,
            face_cover_radius_mm=8.6,
            arm_points=120,
            face_points=240,
        )
        output = io.StringIO()

        with redirect_stdout(output):
            window._log_rgbd_distance_terminal(
                327.2,
                True,
                "arm-face三维净距",
                self._preview(),
                point_ratio=0.62,
                result=result,
            )

        text = output.getvalue()
        self.assertIn("VALID reported_mm=327.2", text)
        self.assertIn("raw_mm=329.4", text)
        self.assertIn("filtered_mm=327.2", text)
        self.assertIn("sampled_min_mm=342.0", text)
        self.assertIn("cover_mm=6.0+8.6", text)
        self.assertIn("point_ratio=62%", text)
        self.assertIn("detections=arm,face", text)
        self.assertIn("projection=v4l-brown+d2c/raw-explicit", text)

    def test_repeated_state_is_throttled_but_invalid_transition_is_immediate(self):
        window = self._window()
        preview = self._preview()
        result = SimpleNamespace(
            raw_distance_mm=329.0,
            filtered_distance_mm=327.0,
            arm_points=100,
            face_points=200,
        )
        output = io.StringIO()

        with redirect_stdout(output):
            window._log_rgbd_distance_terminal(
                327.0, True, "arm-face三维净距", preview, point_ratio=0.60, result=result
            )
            window._log_rgbd_distance_terminal(
                329.0, True, "arm-face三维净距", preview, point_ratio=0.60, result=result
            )
            window._log_rgbd_distance_terminal(
                0.0, False, "RGB/Depth采样不同步", preview
            )

        text = output.getvalue()
        self.assertEqual(text.count(" VALID "), 1)
        self.assertEqual(text.count(" INVALID "), 1)
        self.assertIn("reason=RGB/Depth采样不同步", text)

    def test_disabled_log_prints_nothing(self):
        window = self._window(enabled=False)
        output = io.StringIO()

        with redirect_stdout(output):
            window._log_rgbd_distance_terminal(
                0.0, False, "YOLO结果过期", self._preview()
            )

        self.assertEqual(output.getvalue(), "")


if __name__ == "__main__":
    unittest.main()
