import unittest
from types import SimpleNamespace
from unittest import mock

from arm_mole_qt.app.main import (
    MainWindow,
    PAGE_BINOCULAR,
    PAGE_END_CAMERA,
    PAGE_WORKBENCH,
    VideoWorker,
    parse_args,
)


class CameraResourceLifecycleTest(unittest.TestCase):
    def test_astra_always_on_is_enabled_by_default(self):
        args = parse_args(["--camera-layout", "astra-rgbd"])

        self.assertEqual(args.astra_always_on, 1)

    def test_end_camera_brightness_profile_defaults_are_persisted(self):
        args = parse_args([])

        self.assertEqual(args.camera_brightness, 0.0)
        self.assertEqual(args.camera_gain, 64.0)
        self.assertEqual(args.camera_backlight_compensation, 0.0)

    def test_end_camera_defaults_use_formal_640x480_mode(self):
        args = parse_args([])

        self.assertEqual((args.width, args.height, args.fps), (640, 480, 30))

    def test_facial_enhancement_defaults_are_enabled_but_can_be_disabled(self):
        args = parse_args([])

        self.assertEqual(args.facial_enhanced_filters, 1)
        self.assertEqual(args.facial_triptych_top_candidate_only, 1)

        fallback_args = parse_args(
            [
                "--facial-enhanced-filters",
                "0",
                "--facial-triptych-top-candidate-only",
                "0",
            ]
        )

        self.assertEqual(fallback_args.facial_enhanced_filters, 0)
        self.assertEqual(fallback_args.facial_triptych_top_candidate_only, 0)

    def test_restart_launches_full_arm_script_with_display_zero(self):
        window = MainWindow.__new__(MainWindow)

        with mock.patch(
            "arm_mole_qt.app.main.QProcess.startDetached",
            return_value=True,
        ) as start_detached:
            ok, message = MainWindow._launch_full_arm_restart(window)

        self.assertTrue(ok)
        self.assertIn("run_full_arm.sh", message)
        program, args, cwd = start_detached.call_args.args
        self.assertEqual(program, "/usr/bin/env")
        self.assertEqual(args[0:2], ["DISPLAY=:0", "bash"])
        self.assertTrue(args[2].endswith("/arm_mole_qt/run_full_arm.sh"))
        self.assertTrue(cwd.endswith("/arm_mole_qt"))

    def _window(
        self,
        *,
        layout="astra-rgbd",
        always_on=1,
        demo=False,
        rgb_enabled=1,
        depth_enabled=1,
    ):
        window = MainWindow.__new__(MainWindow)
        window._closing_requested = False
        window.args = SimpleNamespace(
            camera_layout=layout,
            astra_always_on=always_on,
            demo=demo,
            astra_rgb_enabled=rgb_enabled,
            astra_depth_enabled=depth_enabled,
        )
        window._workflow_active = False
        window._scene_guard = SimpleNamespace(active=False)
        window._scene_abort_triggered = False
        window._fault_hold = False
        calls = []
        window._start_binocular_workers = lambda: calls.append("start_astra")
        window._stop_binocular_workers = lambda: calls.append("stop_astra")
        window._start_end_worker = lambda: calls.append("start_end")
        window._stop_end_worker = lambda: calls.append("stop_end")
        return window, calls

    def test_astra_rgbd_starts_on_workbench_and_stays_available(self):
        window, calls = self._window()

        window._activate_page_resources(PAGE_WORKBENCH)

        self.assertEqual(calls, ["start_astra", "stop_end"])

    def test_end_camera_page_keeps_astra_running_at_the_same_time(self):
        window, calls = self._window()

        window._activate_page_resources(PAGE_END_CAMERA)

        self.assertEqual(calls, ["start_astra", "start_end"])

    def test_binocular_page_stops_end_camera_but_keeps_astra(self):
        window, calls = self._window()

        window._activate_page_resources(PAGE_BINOCULAR)

        self.assertEqual(calls, ["stop_end", "start_astra"])

    def test_always_on_can_be_disabled(self):
        window, calls = self._window(always_on=0)

        window._activate_page_resources(PAGE_WORKBENCH)

        self.assertEqual(calls, ["stop_astra", "stop_end"])

    def test_demo_never_opens_real_astra(self):
        window, calls = self._window(demo=True)

        window._activate_page_resources(PAGE_WORKBENCH)

        self.assertEqual(calls, ["stop_astra", "stop_end"])


class EndCameraFrameSchedulingTest(unittest.TestCase):
    class _Capture:
        def __init__(self):
            self.frame = object()
            self.read_calls = 0
            self.grab_calls = 0
            self.retrieve_calls = 0

        def read(self):
            self.read_calls += 1
            return True, self.frame

        def grab(self):
            self.grab_calls += 1
            return True

        def retrieve(self):
            self.retrieve_calls += 1
            return True, self.frame

    @staticmethod
    def _worker():
        worker = VideoWorker.__new__(VideoWorker)
        worker.ui_interval = 0.05
        return worker

    def test_first_or_forced_frame_uses_read(self):
        capture = self._Capture()

        ok, frame, skipped = self._worker()._read_due_camera_frame(
            capture,
            forced_work=False,
            next_emit=0.0,
        )

        self.assertTrue(ok)
        self.assertIs(frame, capture.frame)
        self.assertFalse(skipped)
        self.assertEqual((capture.read_calls, capture.grab_calls), (1, 0))

        forced_capture = self._Capture()
        ok, frame, skipped = self._worker()._read_due_camera_frame(
            forced_capture,
            forced_work=True,
            next_emit=99.0,
        )

        self.assertTrue(ok)
        self.assertIs(frame, forced_capture.frame)
        self.assertFalse(skipped)
        self.assertEqual(
            (forced_capture.read_calls, forced_capture.grab_calls),
            (1, 0),
        )

    def test_early_frame_is_grabbed_once_and_skipped_without_read(self):
        capture = self._Capture()
        with mock.patch("arm_mole_qt.app.main.time.monotonic", return_value=10.03):
            ok, frame, skipped = self._worker()._read_due_camera_frame(
                capture,
                forced_work=False,
                next_emit=10.05,
            )

        self.assertTrue(ok)
        self.assertIsNone(frame)
        self.assertTrue(skipped)
        self.assertEqual(
            (capture.read_calls, capture.grab_calls, capture.retrieve_calls),
            (0, 1, 0),
        )

    def test_due_frame_retrieves_the_frame_already_grabbed(self):
        capture = self._Capture()
        with mock.patch("arm_mole_qt.app.main.time.monotonic", return_value=10.06):
            ok, frame, skipped = self._worker()._read_due_camera_frame(
                capture,
                forced_work=False,
                next_emit=10.05,
            )

        self.assertTrue(ok)
        self.assertIs(frame, capture.frame)
        self.assertFalse(skipped)
        self.assertEqual(
            (capture.read_calls, capture.grab_calls, capture.retrieve_calls),
            (0, 1, 1),
        )

    def test_deadline_keeps_requested_rate_independent_of_frame_phase(self):
        worker = self._worker()
        deadline = 0.0
        emitted = []

        for frame_index in range(301):
            sample_time = frame_index / 30.0
            if deadline <= 0.0 or sample_time >= deadline:
                emitted.append(sample_time)
                deadline = worker._advance_ui_deadline(deadline, sample_time)

        # Discrete 30 FPS frames cannot land on every 50 ms boundary, but the
        # long-run rate must remain close to the requested 20 FPS rather than
        # phase-locking to every second camera frame (15 FPS).
        measured_rate = (len(emitted) - 1) / (emitted[-1] - emitted[0])
        self.assertGreater(measured_rate, 19.8)
        self.assertLessEqual(measured_rate, 20.0)
        self.assertAlmostEqual(deadline, 10.0, places=6)

    def test_late_frame_skips_missed_deadlines_without_bursting(self):
        worker = self._worker()

        deadline = worker._advance_ui_deadline(10.05, 10.31)

        self.assertAlmostEqual(deadline, 10.35, places=6)

    def test_plain_preview_reuses_one_qimage_conversion(self):
        worker = self._worker()
        frame = object()
        converted = object()
        worker._to_qimage = mock.Mock(return_value=converted)

        sample = worker._build_image_sample(frame, frame, 12.5)

        self.assertIs(sample.image, converted)
        self.assertIs(sample.raw_image, converted)
        self.assertEqual(worker._to_qimage.call_count, 1)

    def test_overlay_keeps_a_separate_raw_qimage(self):
        worker = self._worker()
        shown = object()
        raw = object()
        shown_qimage = object()
        raw_qimage = object()
        worker._to_qimage = mock.Mock(side_effect=(shown_qimage, raw_qimage))

        sample = worker._build_image_sample(shown, raw, 12.5)

        self.assertIs(sample.image, shown_qimage)
        self.assertIs(sample.raw_image, raw_qimage)
        self.assertEqual(worker._to_qimage.call_count, 2)


if __name__ == "__main__":
    unittest.main()
