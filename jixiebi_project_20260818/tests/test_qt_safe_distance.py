import io
import os
import time
import unittest
from contextlib import redirect_stderr, redirect_stdout
from threading import Lock
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np


os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

try:
    from PyQt5.QtWidgets import QMessageBox

    from arm_mole_qt.app.main import (
        AstraRgbPreview,
        DepthPreview,
        DetectionResult,
        DetectionTarget,
        ERROR_SAFE_DISTANCE_TIMEOUT,
        ERROR_SEQ_CONFLICT,
        MainWindow,
        RA6_COMMAND_ACK_FUNC,
        RA6_SAFE_DISTANCE_FUNC,
        UNSUPPORTED_CONTINUOUS_SAFE_DISTANCE_MESSAGE,
        VISION_TRANSIENT_LOSS_GRACE_S,
        VideoWorker,
        _build_target_identity_lock,
        _classify_vision_invalid_reason,
        main,
        parse_args,
    )
except ImportError as exc:  # pragma: no cover - optional on non-Qt dev hosts
    AstraRgbPreview = None
    DepthPreview = None
    DetectionResult = None
    DetectionTarget = None
    MainWindow = None
    QMessageBox = None
    VideoWorker = None
    _build_target_identity_lock = None
    IMPORT_ERROR = exc
else:
    IMPORT_ERROR = None


class _Label:
    def __init__(self):
        self.text = ""

    def setText(self, text):
        self.text = str(text)


class _ResetSpy:
    def __init__(self):
        self.reset_calls = 0

    def reset(self):
        self.reset_calls += 1


class _DistanceEstimatorSpy(_ResetSpy):
    def __init__(self):
        super().__init__()
        self.estimate_calls = []

    def estimate(self, *args, **kwargs):
        self.estimate_calls.append((args, kwargs))
        return SimpleNamespace(
            distance_mm=250.0,
            arm_valid_ratio=0.80,
            face_valid_ratio=0.90,
            arm_points=80,
            face_points=160,
        )

    @staticmethod
    def commit_result(result):
        return result


class _Button:
    def __init__(self):
        self.enabled = None
        self.text = ""

    def setEnabled(self, enabled):
        self.enabled = bool(enabled)

    def setText(self, text):
        self.text = str(text)


class _ArmSpy:
    connected = True
    serial_protocol = "unified"
    last_safe_distance_seq = 7

    def __init__(self):
        self.safe_distance_calls = []

    def send_safe_distance(self, distance_mm, valid):
        self.safe_distance_calls.append((int(distance_mm), bool(valid)))
        return True, "sent"


class _WorkflowArmSpy:
    connected = True
    control_busy = False

    def __init__(self):
        # Deliberately start in the unsafe retired mode. MainWindow must
        # override it before asking ArmController to build the wire frame.
        self.continuous_safe_distance = True
        self.mode_seen_by_start = None

    def start_workflow(self):
        self.mode_seen_by_start = self.continuous_safe_distance
        return True, "已发送 WORKFLOW_CTRL START_MEASURE"


class _TargetArmSpy:
    connected = True
    control_busy = False
    abort_pending = False
    serial_protocol = "unified"
    aim_reference_configured = True

    def __init__(self):
        self.start_calls = 0
        self.finish_calls = 0
        self.target_samples = []

    def start_target_state_machine(self):
        self.start_calls += 1
        return True, "已发送 TARGET_CTRL enable=1 新协议帧"

    def send_target_sample(self, target):
        self.target_samples.append(target)
        return True, "已发送 VISION_ERROR valid=0" if target is None else "已发送 valid=1"

    def finish_workflow(self):
        self.finish_calls += 1
        return True, "已发送 WORKFLOW_CTRL FINISH_RETURN_HOME"

    @staticmethod
    def aim_corrected_error(target):
        width, height = target.frame_size
        # Test calibration: the real laser aim pixel is (390, 280).
        return target.pixel[0] - (width // 2 + 70), target.pixel[1] - (
            height // 2 + 40
        )


@unittest.skipIf(IMPORT_ERROR is not None, "Qt application dependencies unavailable")
class QtSafeDistanceFreshnessTest(unittest.TestCase):
    def _window_with_valid_sample(self):
        now = time.monotonic()
        rgb_time = now - 0.10
        depth_time = now - 0.08
        window = MainWindow.__new__(MainWindow)
        window.serial_safe_distance_source = "depth"
        window.serial_safe_distance_max_age = 0.5
        window._latest_safe_depth_mm = 321.4
        window._latest_safe_depth_valid_ratio = 0.72
        window._latest_safe_depth_time = depth_time
        window._latest_safe_rgb_time = rgb_time
        window._latest_safe_face_box = (100.0, 80.0, 260.0, 300.0)
        window._latest_safe_face_size = (640, 480)
        window._latest_safe_measurement_valid = True
        window._latest_safe_invalid_reason = ""
        window._latest_rgbd_detection_time = rgb_time
        window._latest_depth_preview = None
        window._latest_rgbd_detections = {
            "arm": {"box": [20.0, 30.0, 90.0, 120.0]},
            "face": {"box": [100.0, 80.0, 260.0, 300.0]},
        }
        window._latest_rgbd_rgb_size = (640, 480)
        window._measure_position_ready_time = now - 1.0
        window._rgbd_distance_estimator = _ResetSpy()
        window.rgbd_distance_log_enabled = False
        window.binocular_safe_distance_value = _Label()
        return window, now

    @staticmethod
    def _depth_preview(sample_time):
        return DepthPreview(
            True,
            None,
            640,
            480,
            30.0,
            900.0,
            0.8,
            "Depth 正常",
            sample_time=sample_time,
        )

    def _mapping_gate_fixture(
        self,
        *,
        hardware_pair_verified=False,
        allow_approximate_mapping=False,
        protocol_debug_controls=False,
        demo=False,
    ):
        window, now = self._window_with_valid_sample()
        window.args = SimpleNamespace(
            rgbd_hardware_pair_verified=hardware_pair_verified,
            rgbd_allow_approximate_mapping=allow_approximate_mapping,
            protocol_debug_controls=protocol_debug_controls,
            demo=demo,
            astra_rgb_flip="none",
        )
        window._astra_rgb_calibration = None
        window._rgbd_distance_estimator = _DistanceEstimatorSpy()
        window.rgbd_sync_max_delta = 0.15
        window.serial_safe_distance_min_valid_ratio = 0.20
        window._measure_position_ready = False
        window._waiting_for_safe_distance = False
        preview = DepthPreview(
            True,
            None,
            640,
            480,
            30.0,
            900.0,
            0.8,
            "Depth 正常",
            depth_mm=object(),
            intrinsics=(500.0, 500.0, 320.0, 240.0),
            sample_time=now - 0.08,
        )
        return window, preview

    def test_unverified_hardware_pair_rejects_rgbd_distance(self):
        window, preview = self._mapping_gate_fixture()

        window._update_safe_distance_from_depth(preview)

        self.assertFalse(window._latest_safe_measurement_valid)
        self.assertIn("尚未完成真机验证", window._latest_safe_invalid_reason)
        self.assertEqual(window._rgbd_distance_estimator.reset_calls, 1)
        self.assertEqual(window._rgbd_distance_estimator.estimate_calls, [])

    def test_debug_mode_with_allow_flag_permits_approximate_mapping(self):
        window, preview = self._mapping_gate_fixture(
            allow_approximate_mapping=True,
            protocol_debug_controls=True,
        )

        window._update_safe_distance_from_depth(preview)

        self.assertTrue(window._latest_safe_measurement_valid)
        self.assertEqual(window._latest_safe_invalid_reason, "")
        self.assertEqual(len(window._rgbd_distance_estimator.estimate_calls), 1)
        self.assertAlmostEqual(window._latest_safe_depth_mm, 250.0)

    def test_formal_mode_cannot_bypass_pair_gate_with_allow_flag_alone(self):
        window, preview = self._mapping_gate_fixture(
            allow_approximate_mapping=True,
            protocol_debug_controls=False,
            demo=False,
        )

        window._update_safe_distance_from_depth(preview)

        self.assertFalse(window._latest_safe_measurement_valid)
        self.assertIn("尚未完成真机验证", window._latest_safe_invalid_reason)
        self.assertEqual(window._rgbd_distance_estimator.reset_calls, 1)
        self.assertEqual(window._rgbd_distance_estimator.estimate_calls, [])

    def test_stale_keeps_last_value_but_is_not_sendable_or_filter_reset(self):
        window, now = self._window_with_valid_sample()
        old_depth_time = window._latest_safe_depth_time

        window._invalidate_safe_distance(
            "YOLO结果过期",
            self._depth_preview(now),
            preserve_last=True,
        )

        self.assertFalse(window._latest_safe_measurement_valid)
        self.assertEqual(window._latest_safe_depth_time, old_depth_time)
        self.assertAlmostEqual(window._latest_safe_depth_mm, 321.4)
        self.assertEqual(window._rgbd_distance_estimator.reset_calls, 0)
        self.assertIn("321 mm", window.binocular_safe_distance_value.text)
        self.assertIn("已过期", window.binocular_safe_distance_value.text)
        self.assertIn("未发送", window.binocular_safe_distance_value.text)
        self.assertEqual(
            window._serial_safe_distance_payload(now),
            (0, False, "RGB-D净距无效"),
        )

    def test_new_invalid_yolo_cannot_send_previous_valid_distance(self):
        window, now = self._window_with_valid_sample()
        window.rgb_preview_label = object()
        window.camera_layout_value = _Label()
        window.camera_primary_value = _Label()
        window.camera_raw_size_value = _Label()
        window.camera_view_size_value = _Label()
        window.camera_message_value = _Label()
        window.binocular_yolo_state = _Label()
        window._set_label_image = lambda *_args, **_kwargs: None
        window._workflow_active = True
        window._measure_position_ready = True
        window._waiting_for_safe_distance = True
        window._distance_safe_latched = False
        window._scene_abort_triggered = False
        window._scene_guard = SimpleNamespace(active=False)

        window._set_astra_rgb_preview(
            AstraRgbPreview(
                True,
                None,
                640,
                480,
                30.0,
                "Astra RGB 正常",
                "YOLO 等待：机械臂末端、人脸",
                False,
                False,
                {},
                now,
            )
        )

        self.assertFalse(window._latest_safe_measurement_valid)
        self.assertAlmostEqual(window._latest_safe_depth_mm, 321.4)
        self.assertIn("已过期", window.binocular_safe_distance_value.text)
        self.assertEqual(window._rgbd_distance_estimator.reset_calls, 0)

        window.arm = _ArmSpy()
        window.args = SimpleNamespace(continuous_safe_distance=False)
        window._measure_position_ready = True
        window._waiting_for_safe_distance = True
        window._distance_safe_latched = False
        window._last_sent_safe_sample_time = 0.0
        window._last_serial_safe_distance = 0.0

        self.assertFalse(window._send_serial_safe_distance_now(now + 0.01))
        self.assertEqual(window.arm.safe_distance_calls, [])

    def test_new_yolo_immediately_uses_already_buffered_synchronized_depth(self):
        window, now = self._window_with_valid_sample()
        window.rgb_preview_label = object()
        window.camera_layout_value = _Label()
        window.camera_primary_value = _Label()
        window.camera_raw_size_value = _Label()
        window.camera_view_size_value = _Label()
        window.camera_message_value = _Label()
        window.binocular_yolo_state = _Label()
        window._set_label_image = lambda *_args, **_kwargs: None
        window._workflow_active = True
        window._measure_position_ready = True
        window._waiting_for_safe_distance = True
        window._distance_safe_latched = False
        window._scene_abort_triggered = False
        window._scene_guard = SimpleNamespace(active=False)
        window.rgbd_sync_max_delta = 0.15
        depth_preview = SimpleNamespace(
            available=True,
            depth_mm=object(),
            sample_time=now - 0.02,
        )
        window._latest_depth_preview = depth_preview
        window._last_rgbd_estimated_detection_time = 0.0
        paired = []
        window._update_safe_distance_from_depth = paired.append

        window._set_astra_rgb_preview(
            AstraRgbPreview(
                True,
                None,
                640,
                480,
                30.0,
                "Astra RGB 正常",
                "YOLO arm_tip + face",
                True,
                True,
                {
                    "arm": {"box": [20.0, 30.0, 90.0, 120.0]},
                    "face": {"box": [100.0, 80.0, 260.0, 300.0]},
                },
                now,
            )
        )

        self.assertEqual(paired, [depth_preview])
        self.assertEqual(window._last_rgbd_estimated_detection_time, now)

    def test_clearance_estimation_waits_for_measure_position(self):
        window = MainWindow.__new__(MainWindow)
        window._workflow_active = False
        window._distance_safe_latched = False
        window._measure_position_ready = False
        window._waiting_for_safe_distance = False
        window._latest_rgbd_detection_time = 10.0
        window._last_rgbd_estimated_detection_time = 0.0

        self.assertFalse(window._rgbd_clearance_estimation_enabled())
        self.assertFalse(window._claim_rgbd_clearance_estimate(now=5.0))
        self.assertEqual(window._last_rgbd_estimated_detection_time, 0.0)

        window._workflow_active = True
        self.assertFalse(window._rgbd_clearance_estimation_enabled())
        self.assertFalse(window._claim_rgbd_clearance_estimate(now=5.1))

        window._measure_position_ready = True
        window._waiting_for_safe_distance = True
        self.assertTrue(window._rgbd_clearance_estimation_enabled())
        self.assertTrue(window._claim_rgbd_clearance_estimate(now=5.2))
        self.assertEqual(window._last_rgbd_estimated_detection_time, 10.0)

    def test_distance_latch_stops_safe_distance_transmission(self):
        window = MainWindow.__new__(MainWindow)
        window.arm = _ArmSpy()
        window._measure_position_ready = True
        window._waiting_for_safe_distance = False
        window._distance_safe_latched = True

        self.assertFalse(window._send_serial_safe_distance_now(time.monotonic()))
        self.assertEqual(window.arm.safe_distance_calls, [])

    def test_post_latch_rgbd_estimates_are_rate_limited_but_continue(self):
        window = MainWindow.__new__(MainWindow)
        window._workflow_active = True
        window._distance_safe_latched = True
        window.rgbd_continuous_stop_mm = 120.0
        window.rgbd_continuous_guard_interval = 0.20
        window._last_rgbd_estimated_detection_time = 0.0
        window._last_rgbd_continuous_estimate_time = 0.0
        window._latest_rgbd_detection_time = 10.0

        self.assertTrue(window._claim_rgbd_clearance_estimate(now=5.0))
        self.assertEqual(window._last_rgbd_estimated_detection_time, 10.0)

        window._latest_rgbd_detection_time = 11.0
        self.assertFalse(window._claim_rgbd_clearance_estimate(now=5.10))
        self.assertTrue(window._claim_rgbd_clearance_estimate(now=5.21))
        self.assertEqual(window._last_rgbd_estimated_detection_time, 11.0)

    def test_depth_callback_keeps_estimating_after_distance_latch(self):
        window = MainWindow.__new__(MainWindow)
        window._workflow_active = True
        window._distance_safe_latched = True
        window.rgbd_continuous_stop_mm = 120.0
        window.rgbd_continuous_guard_interval = 0.20
        window._last_rgbd_estimated_detection_time = 0.0
        window._last_rgbd_continuous_estimate_time = 0.0
        window._latest_rgbd_detection_time = time.monotonic() - 0.01
        window._latest_safe_measurement_valid = False
        window.serial_safe_distance_max_age = 0.5
        window._update_safe_distance_from_depth = Mock()
        window.pages = SimpleNamespace(currentIndex=lambda: -1)
        window.depth_state_value = _Label()
        window.depth_size_value = _Label()
        window.depth_center_value = _Label()
        window.depth_valid_value = _Label()
        window.depth_fps_value = _Label()
        window.summary_depth_value = _Label()
        window.binocular_center_depth_value = _Label()
        preview = DepthPreview(
            True,
            None,
            640,
            480,
            30.0,
            900.0,
            0.8,
            "Depth 正常",
            depth_mm=object(),
            sample_time=time.monotonic(),
        )

        window._set_depth_preview(preview)

        window._update_safe_distance_from_depth.assert_called_once_with(preview)

    def test_sub_120mm_post_latch_rgbd_requests_abort_hold_once(self):
        window = MainWindow.__new__(MainWindow)
        window._workflow_active = True
        window._distance_safe_latched = True
        window.rgbd_continuous_stop_mm = 120.0
        window._rgbd_continuous_stop_triggered = False
        window._latest_safe_measurement_valid = True
        window._latest_safe_depth_mm = 119.9
        window._cancel_workflow = Mock()
        window._log_event = Mock()

        with patch("builtins.print"):
            self.assertTrue(window._enforce_continuous_rgbd_guard())
            self.assertFalse(window._enforce_continuous_rgbd_guard())

        window._cancel_workflow.assert_called_once()
        reason = window._cancel_workflow.call_args.args[0]
        self.assertIn("119.9 mm < 120.0 mm", reason)
        self.assertIn("ABORT_HOLD", reason)

    def test_continuous_rgbd_guard_does_not_replace_initial_ra6m5_latch(self):
        window = MainWindow.__new__(MainWindow)
        window._workflow_active = True
        window._distance_safe_latched = False
        window.rgbd_continuous_stop_mm = 120.0
        window._rgbd_continuous_stop_triggered = False
        window._latest_safe_measurement_valid = True
        window._latest_safe_depth_mm = 80.0
        window._cancel_workflow = Mock()

        self.assertFalse(window._enforce_continuous_rgbd_guard())
        window._cancel_workflow.assert_not_called()

    def test_continuous_rgbd_guard_is_strictly_below_threshold(self):
        window = MainWindow.__new__(MainWindow)
        window._workflow_active = True
        window._distance_safe_latched = True
        window.rgbd_continuous_stop_mm = 120.0
        window._rgbd_continuous_stop_triggered = False
        window._latest_safe_measurement_valid = True
        window._latest_safe_depth_mm = 120.0
        window._cancel_workflow = Mock()

        self.assertFalse(window._enforce_continuous_rgbd_guard())
        window._cancel_workflow.assert_not_called()


@unittest.skipIf(IMPORT_ERROR is not None, "Qt application dependencies unavailable")
class QtOfficialWorkflowModeTest(unittest.TestCase):
    def test_retired_continuous_flag_is_rejected_before_qt_starts(self):
        stderr = io.StringIO()
        with redirect_stderr(stderr):
            exit_code = main(["--continuous-safe-distance"])

        self.assertEqual(exit_code, 2)
        self.assertIn("action=0x04", stderr.getvalue())
        self.assertIn("请删除该参数", stderr.getvalue())
        self.assertEqual(
            stderr.getvalue().strip(),
            UNSUPPORTED_CONTINUOUS_SAFE_DISTANCE_MESSAGE,
        )

    def test_formal_start_defensively_forces_action_0x01_mode(self):
        window = MainWindow.__new__(MainWindow)
        window.arm = _WorkflowArmSpy()
        window.workflow_state = _Label()
        window.ready_wait_value = _Label()
        window._log_event = lambda _message: None
        window._set_status = lambda _message: None
        window._refresh_controls = lambda: None
        window._workflow_active = False
        window._waiting_for_arm_ready = False
        window._waiting_for_safe_distance = False
        window._detection_running = False
        window._ready_after_action = ""
        window._arm_ready_wait_started = None

        window._send_workflow_start_command()

        self.assertFalse(window.arm.mode_seen_by_start)
        self.assertTrue(window._workflow_active)
        self.assertEqual(window._ready_after_action, "measure_position")

    def test_rejected_ack_stops_automatic_progress_and_displays_error(self):
        window = MainWindow.__new__(MainWindow)
        window.workflow_state = _Label()
        window.ready_wait_value = _Label()
        reasons = []
        window._stop_automatic_progress = reasons.append
        status = SimpleNamespace(
            error_code=0x03,
            func=RA6_COMMAND_ACK_FUNC,
            label="COMMAND_ACK_REJECTED_ERR_INVALID_PARAM",
        )

        self.assertTrue(window._handle_protocol_error(status))

        self.assertEqual(len(reasons), 1)
        self.assertIn("INVALID_PARAM", reasons[0])
        self.assertIn("INVALID_PARAM", window.workflow_state.text)
        self.assertEqual(window.ready_wait_value.text, "自动流程已停止")

    def test_seq_conflict_closes_heartbeat_and_blocks_restart(self):
        window = MainWindow.__new__(MainWindow)
        window.workflow_state = _Label()
        window.ready_wait_value = _Label()
        window.serial_state = _Label()
        reasons = []

        class ArmSpy:
            def __init__(self):
                self.close_calls = 0

            def close(self):
                self.close_calls += 1

        window.arm = ArmSpy()
        window._stop_automatic_progress = (
            lambda reason, restart_blocked=False: reasons.append(
                (str(reason), bool(restart_blocked))
            )
        )
        status = SimpleNamespace(
            error_code=ERROR_SEQ_CONFLICT,
            func=RA6_COMMAND_ACK_FUNC,
            label="COMMAND_ACK_REJECTED_ERR_SEQ_CONFLICT",
        )

        self.assertTrue(window._handle_protocol_error(status))

        self.assertEqual(len(reasons), 1)
        self.assertTrue(reasons[0][1])
        self.assertIn("SEQ", reasons[0][0])
        self.assertEqual(window.arm.close_calls, 1)
        self.assertIn("硬件复位", window.workflow_state.text)
        self.assertIn("串口已关闭", window.serial_state.text)
        self.assertTrue(
            window._is_terminal_serial_transport_event(
                "控制命令被拒绝：CAPTURE_SELECT COMMAND_ACK_REJECTED_ERR_SEQ_CONFLICT"
            )
        )


@unittest.skipIf(IMPORT_ERROR is not None, "Qt application dependencies unavailable")
class QtEndCameraPerformanceConfigTest(unittest.TestCase):
    def test_continuous_rgbd_guard_is_explicit_and_parses_120mm(self):
        defaults = parse_args([])
        enabled = parse_args(
            [
                "--rgbd-continuous-stop-mm",
                "120",
                "--rgbd-continuous-guard-interval",
                "0.20",
            ]
        )

        self.assertEqual(defaults.rgbd_continuous_stop_mm, 0.0)
        self.assertEqual(enabled.rgbd_continuous_stop_mm, 120.0)
        self.assertEqual(enabled.rgbd_continuous_guard_interval, 0.20)

    def test_faster_defaults_preserve_a_time_based_stability_window(self):
        args = parse_args([])

        self.assertEqual(args.ui_fps, 20.0)
        self.assertEqual(args.facial_face_imgsz, 416)
        self.assertEqual(args.facial_face_refresh_frames, 6)
        self.assertEqual(args.facial_stable_frames, 4)
        self.assertEqual(args.facial_stable_seconds, 0.35)

    def test_target_track_id_rebind_timeout_is_parsed_and_wired(self):
        defaults = parse_args([])
        custom = parse_args(["--target-lock-rebind-timeout", "3.25"])

        self.assertEqual(defaults.target_lock_rebind_timeout, 5.0)
        self.assertEqual(
            _build_target_identity_lock(defaults).track_id_rebind_timeout_s,
            5.0,
        )
        self.assertEqual(custom.target_lock_rebind_timeout, 3.25)
        self.assertEqual(
            _build_target_identity_lock(custom).track_id_rebind_timeout_s,
            3.25,
        )

    def test_facial_tracker_receives_current_face_mask_box(self):
        frame = np.zeros((120, 160, 3), dtype=np.uint8)
        face_mask = np.zeros((120, 160), dtype=np.uint8)
        face_mask[20:90, 30:110] = 255
        candidate = SimpleNamespace(
            center=(80, 60),
            radius=7,
            area=150.0,
            contrast=60.0,
            circularity=0.8,
            track_id=1,
            stable_count=4,
            required_count=4,
            track_age_s=0.5,
        )
        result = SimpleNamespace(
            candidates=[candidate],
            face=object(),
            face_mask=face_mask,
            privacy_view=frame,
        )
        detector = Mock()
        detector.detect.return_value = result

        def render_candidates(_frame, facial_result, stable_candidates):
            facial_result.candidates = list(stable_candidates)
            return facial_result

        detector.render_candidates.side_effect = render_candidates
        tracker = Mock()
        tracker.update.return_value = [candidate]
        worker = VideoWorker.__new__(VideoWorker)
        worker.detector_mode = "facial_candidate"
        worker._facial_detector = detector
        worker._facial_tracker = tracker
        worker.facial_max_candidates = 12

        targets, _message, _view = VideoWorker._detect(
            worker,
            frame,
            sample_time=12.5,
        )

        self.assertEqual(len(targets), 1)
        self.assertEqual(worker._last_raw_candidate_count, 1)
        self.assertEqual(worker._last_detector_reason, "")
        tracker.update.assert_called_once_with(
            [candidate],
            sample_time=12.5,
            reference_box=(30, 20, 80, 70),
        )

    def test_treatment_preview_renders_only_locked_track_but_keeps_all_for_lock(self):
        frame = np.zeros((120, 160, 3), dtype=np.uint8)
        face_mask = np.zeros((120, 160), dtype=np.uint8)
        face_mask[20:100, 20:140] = 255

        def candidate(track_id, center):
            return SimpleNamespace(
                center=center,
                radius=7,
                area=120.0,
                contrast=60.0,
                circularity=0.8,
                track_id=track_id,
                stable_count=4,
                required_count=4,
                track_age_s=0.5,
            )

        selected = candidate(7, (70, 60))
        approach_shadow = candidate(9, (105, 72))
        result = SimpleNamespace(
            candidates=[selected, approach_shadow],
            face=object(),
            face_mask=face_mask,
            privacy_view=frame,
        )
        detector = Mock()
        detector.detect.return_value = result
        rendered_track_ids = []

        def render_candidates(_frame, facial_result, stable_candidates):
            rendered_track_ids.append(
                [int(item.track_id) for item in stable_candidates]
            )
            facial_result.candidates = list(stable_candidates)
            return facial_result

        detector.render_candidates.side_effect = render_candidates
        tracker = Mock()
        tracker.update.return_value = [selected, approach_shadow]
        worker = VideoWorker.__new__(VideoWorker)
        worker.detector_mode = "facial_candidate"
        worker._facial_detector = detector
        worker._facial_tracker = tracker
        worker.facial_max_candidates = 12
        worker._lock = Lock()
        worker._display_track_id = 7

        targets, _message, _view = VideoWorker._detect(
            worker,
            frame,
            sample_time=12.5,
        )

        self.assertEqual([target.track_id for target in targets], [7, 9])
        self.assertEqual(rendered_track_ids[-1], [7])

    def test_facial_detector_reports_raw_candidate_zero(self):
        frame = np.zeros((120, 160, 3), dtype=np.uint8)
        result = SimpleNamespace(
            candidates=[],
            face=object(),
            face_mask=np.ones((120, 160), dtype=np.uint8) * 255,
            privacy_view=frame,
        )
        detector = Mock()
        detector.detect.return_value = result
        detector.render_candidates.return_value = result
        tracker = Mock()
        tracker.update.return_value = []
        worker = VideoWorker.__new__(VideoWorker)
        worker.detector_mode = "facial_candidate"
        worker._facial_detector = detector
        worker._facial_tracker = tracker
        worker.facial_max_candidates = 12

        targets, _message, _view = VideoWorker._detect(worker, frame, sample_time=1.0)

        self.assertEqual(targets, [])
        self.assertEqual(worker._last_raw_candidate_count, 0)
        self.assertEqual(worker._last_detector_reason, "RAW_CANDIDATE_0")

    def test_facial_detector_reports_stability_pending(self):
        frame = np.zeros((120, 160, 3), dtype=np.uint8)
        candidate = SimpleNamespace(
            center=(80, 60),
            radius=7,
            area=150.0,
            contrast=60.0,
            circularity=0.8,
        )
        result = SimpleNamespace(
            candidates=[candidate],
            face=object(),
            face_mask=np.ones((120, 160), dtype=np.uint8) * 255,
            privacy_view=frame,
        )
        detector = Mock()
        detector.detect.return_value = result

        def render_candidates(_frame, facial_result, stable_candidates):
            facial_result.candidates = list(stable_candidates)
            return facial_result

        detector.render_candidates.side_effect = render_candidates
        tracker = Mock()
        tracker.update.return_value = []
        worker = VideoWorker.__new__(VideoWorker)
        worker.detector_mode = "facial_candidate"
        worker._facial_detector = detector
        worker._facial_tracker = tracker
        worker.facial_max_candidates = 12

        targets, _message, _view = VideoWorker._detect(worker, frame, sample_time=1.0)

        self.assertEqual(targets, [])
        self.assertEqual(worker._last_raw_candidate_count, 1)
        self.assertEqual(worker._last_detector_reason, "STABILITY_PENDING")

    def test_detect_once_frame_budget_covers_the_time_gate(self):
        detector = _ResetSpy()
        tracker = _ResetSpy()
        worker = SimpleNamespace(
            _lock=Lock(),
            _detect_once=False,
            _detect_once_frames_left=0,
            facial_stable_frames=4,
            facial_stable_seconds=0.35,
            ui_interval=0.05,
            fps=60,
            _facial_detector=detector,
            _facial_tracker=tracker,
        )

        VideoWorker.request_detect_once(worker)

        self.assertEqual(worker._detect_once_frames_left, 8)
        self.assertEqual(detector.reset_calls, 1)
        self.assertEqual(tracker.reset_calls, 1)

    def test_repeated_start_does_not_reset_active_facial_detection_state(self):
        detector = _ResetSpy()
        tracker = _ResetSpy()
        worker = SimpleNamespace(
            _lock=Lock(),
            _continuous=False,
            _detect_once=False,
            _detect_once_frames_left=0,
            _facial_detector=detector,
            _facial_tracker=tracker,
        )

        VideoWorker.start_detection(worker)
        VideoWorker.start_detection(worker)
        self.assertEqual(detector.reset_calls, 1)
        self.assertEqual(tracker.reset_calls, 1)

        VideoWorker.stop_detection(worker)
        VideoWorker.start_detection(worker)
        self.assertEqual(detector.reset_calls, 2)
        self.assertEqual(tracker.reset_calls, 2)


@unittest.skipIf(IMPORT_ERROR is not None, "Qt application dependencies unavailable")
class QtVisionInvalidReasonTest(unittest.TestCase):
    @staticmethod
    def _result(detector_reason=""):
        return DetectionResult(
            detected=False,
            targets=[],
            frame_width=640,
            frame_height=480,
            fps=6.0,
            mode="连续识别",
            raw_candidate_count=0,
            detector_reason=detector_reason,
        )

    def test_invalid_reason_categories_are_unambiguous(self):
        cases = (
            ("PARTIAL_FACE", "候选尚未通过身份稳定锁", "PARTIAL_FACE"),
            ("NO_FACE", "候选尚未通过身份稳定锁", "NO_FACE"),
            ("RAW_CANDIDATE_0", "锁定目标暂时丢失", "RAW_CANDIDATE_0"),
            ("STABILITY_PENDING", "候选尚未通过身份稳定锁", "STABILITY_PENDING"),
            ("", "锁定目标暂时丢失", "TARGET_LOCK_LOST"),
        )
        for detector_reason, gate_reason, expected in cases:
            with self.subTest(expected=expected):
                self.assertEqual(
                    _classify_vision_invalid_reason(
                        target_valid=False,
                        gate_reason=gate_reason,
                        result=self._result(detector_reason),
                        aim_reference_configured=True,
                    ),
                    expected,
                )

    def test_stale_result_and_uncalibrated_reference_have_priority(self):
        self.assertEqual(
            _classify_vision_invalid_reason(
                target_valid=False,
                gate_reason="末端识别结果过期 812ms",
                result=self._result("RAW_CANDIDATE_0"),
                aim_reference_configured=True,
            ),
            "RESULT_STALE",
        )
        self.assertEqual(
            _classify_vision_invalid_reason(
                target_valid=False,
                gate_reason="激光瞄准参考像素未标定",
                result=self._result("STABILITY_PENDING"),
                aim_reference_configured=False,
            ),
            "AIM_REFERENCE_UNCALIBRATED",
        )

    def test_throttled_gate_log_prints_structured_invalid_reason(self):
        class PublishingArmSpy:
            connected = True
            aim_reference_configured = True
            background_vision_enabled = True

            def __init__(self):
                self.samples = []

            def publish_target_sample(self, target, sample_time):
                self.samples.append((target, sample_time))
                return True, "已发布 valid=0"

        window = MainWindow.__new__(MainWindow)
        window.arm = PublishingArmSpy()
        window._target_ready = True
        window._latest_result_time = time.monotonic()
        window.current_result = self._result("RAW_CANDIDATE_0")
        window.current_result.sample_time = window._latest_result_time
        window._locked_current_target = None
        window._target_identity_lock = SimpleNamespace(
            track_id=0,
            anchor=None,
            active=True,
            lost=True,
            pending_count=0,
            stable_frames=2,
        )
        window._last_vision_gate_reason = ""
        window._last_vision_invalid_reason = ""
        window._last_vision_gate_log_time = 0.0
        window._current_arm_target_with_reason = lambda: (
            None,
            "锁定目标暂时丢失",
        )
        window._set_status = lambda _message: None

        output = io.StringIO()
        with redirect_stdout(output):
            MainWindow._publish_latest_vision_result(window)

        self.assertEqual(window.arm.samples, [(None, window._latest_result_time)])
        self.assertIn("invalid_reason=RAW_CANDIDATE_0", output.getvalue())
        self.assertIn("raw_candidates=0", output.getvalue())
        self.assertIn("detector_reason=RAW_CANDIDATE_0", output.getvalue())

    def test_short_stale_result_keeps_background_serial_mailbox_silent(self):
        class PublishingArmSpy:
            connected = True
            aim_reference_configured = True
            background_vision_enabled = True

            def __init__(self):
                self.samples = []

            def publish_target_sample(self, target, sample_time):
                self.samples.append((target, sample_time))
                return True, "unexpected publish"

        window = MainWindow.__new__(MainWindow)
        window.arm = PublishingArmSpy()
        window._target_ready = True
        window._latest_result_time = time.monotonic() - 0.55
        window.current_result = self._result()
        window._locked_current_target = None
        window._target_identity_lock = SimpleNamespace(
            track_id=1,
            anchor=None,
            active=True,
            lost=False,
            pending_count=0,
            stable_frames=2,
        )
        window._last_vision_gate_reason = ""
        window._last_vision_invalid_reason = ""
        window._last_vision_gate_log_time = 0.0
        window._current_arm_target_with_reason = lambda: (
            None,
            "末端识别结果过期 550ms",
        )
        window._set_status = lambda _message: None

        output = io.StringIO()
        with redirect_stdout(output):
            MainWindow._publish_latest_vision_result(window)

        self.assertEqual(window.arm.samples, [])
        self.assertIn("invalid_reason=RESULT_STALE", output.getvalue())
        self.assertIn("published=0", output.getvalue())

    def test_one_frame_lock_loss_after_valid_stays_silent(self):
        class PublishingArmSpy:
            connected = True
            aim_reference_configured = True
            background_vision_enabled = True

            def __init__(self):
                self.samples = []

            def publish_target_sample(self, target, sample_time):
                self.samples.append((target, sample_time))
                return True, "published"

        window = MainWindow.__new__(MainWindow)
        window.arm = PublishingArmSpy()
        window._target_ready = True
        window._latest_result_time = time.monotonic()
        window.current_result = self._result("STABILITY_PENDING")
        window.current_result.sample_time = window._latest_result_time
        window._locked_current_target = None
        window._target_identity_lock = SimpleNamespace(
            track_id=7,
            anchor=None,
            active=True,
            lost=True,
            pending_count=0,
            stable_frames=2,
        )
        window._last_vision_gate_reason = ""
        window._last_vision_invalid_reason = ""
        window._last_vision_gate_log_time = 0.0
        window._vision_last_published_valid = True
        window._vision_invalid_transition_sent = False
        window._vision_transient_loss_started_at = 0.0
        window._current_arm_target_with_reason = lambda: (
            None,
            "锁定目标暂时丢失",
        )
        window._set_status = lambda _message: None

        with redirect_stdout(io.StringIO()):
            MainWindow._publish_latest_vision_result(window)

        self.assertEqual(window.arm.samples, [])
        self.assertTrue(window._vision_last_published_valid)
        self.assertGreater(window._vision_transient_loss_started_at, 0.0)

    def test_transient_lock_loss_emits_one_invalid_after_grace(self):
        class PublishingArmSpy:
            connected = True
            aim_reference_configured = True
            background_vision_enabled = True

            def __init__(self):
                self.samples = []

            def publish_target_sample(self, target, sample_time):
                self.samples.append((target, sample_time))
                return True, "published"

        window = MainWindow.__new__(MainWindow)
        window.arm = PublishingArmSpy()
        window._target_ready = True
        window._latest_result_time = time.monotonic()
        window.current_result = self._result("RAW_CANDIDATE_0")
        window.current_result.sample_time = window._latest_result_time
        window._locked_current_target = None
        window._target_identity_lock = SimpleNamespace(
            track_id=7,
            anchor=None,
            active=True,
            lost=True,
            pending_count=0,
            stable_frames=2,
        )
        window._last_vision_gate_reason = ""
        window._last_vision_invalid_reason = ""
        window._last_vision_gate_log_time = 0.0
        window._vision_last_published_valid = True
        window._vision_invalid_transition_sent = False
        window._vision_transient_loss_started_at = (
            time.monotonic() - VISION_TRANSIENT_LOSS_GRACE_S - 0.05
        )
        window._current_arm_target_with_reason = lambda: (
            None,
            "锁定目标暂时丢失",
        )
        window._set_status = lambda _message: None

        with redirect_stdout(io.StringIO()):
            MainWindow._publish_latest_vision_result(window)
        window._latest_result_time += 0.05
        with redirect_stdout(io.StringIO()):
            MainWindow._publish_latest_vision_result(window)

        self.assertEqual(len(window.arm.samples), 1)
        self.assertIsNone(window.arm.samples[0][0])
        self.assertFalse(window._vision_last_published_valid)
        self.assertTrue(window._vision_invalid_transition_sent)

    def test_partial_face_invalid_is_immediate_even_after_valid(self):
        class PublishingArmSpy:
            connected = True
            aim_reference_configured = True
            background_vision_enabled = True

            def __init__(self):
                self.samples = []

            def publish_target_sample(self, target, sample_time):
                self.samples.append((target, sample_time))
                return True, "published"

        window = MainWindow.__new__(MainWindow)
        window.arm = PublishingArmSpy()
        window._target_ready = True
        window._latest_result_time = time.monotonic()
        window.current_result = self._result("PARTIAL_FACE")
        window.current_result.sample_time = window._latest_result_time
        window._locked_current_target = None
        window._target_identity_lock = SimpleNamespace(
            track_id=7,
            anchor=None,
            active=True,
            lost=True,
            pending_count=0,
            stable_frames=2,
        )
        window._last_vision_gate_reason = ""
        window._last_vision_invalid_reason = ""
        window._last_vision_gate_log_time = 0.0
        window._vision_last_published_valid = True
        window._vision_invalid_transition_sent = False
        window._vision_transient_loss_started_at = 0.0
        window._current_arm_target_with_reason = lambda: (
            None,
            "人脸区域不完整",
        )
        window._set_status = lambda _message: None

        with redirect_stdout(io.StringIO()):
            MainWindow._publish_latest_vision_result(window)

        self.assertEqual(window.arm.samples, [(None, window._latest_result_time)])
        self.assertFalse(window._vision_last_published_valid)
        self.assertTrue(window._vision_invalid_transition_sent)


@unittest.skipIf(IMPORT_ERROR is not None, "Qt application dependencies unavailable")
class QtTargetDisplayStateTest(unittest.TestCase):
    @staticmethod
    def _window(lock_target):
        window = MainWindow.__new__(MainWindow)
        window.arm = _TargetArmSpy()
        window._locked_current_target = None
        window._locked_target_sample_time = 0.0
        window._target_ready = True
        for name in (
            "mode_value",
            "fps_value",
            "target_count",
            "summary_fps_value",
            "summary_target_value",
            "detect_state",
            "pixel_coord",
            "robot_coord",
            "radius_value",
            "score_value",
        ):
            setattr(window, name, _Label())

        def update_lock(result):
            if lock_target:
                window._locked_current_target = result.targets[0]
                window._locked_target_sample_time = window._latest_result_time

        window._update_target_identity_lock = update_lock
        window._set_safety_state = lambda _result: None
        window._add_history = lambda _target: None
        window._refresh_controls = lambda: None
        return window

    @staticmethod
    def _result():
        return DetectionResult(
            detected=True,
            targets=[DetectionTarget(438, 237, 7, 120.0, 91.1, 0.9)],
            frame_width=640,
            frame_height=480,
            fps=14.0,
            mode="连续识别",
            sample_time=time.monotonic(),
        )

    def test_candidate_text_does_not_claim_coordinates_are_valid(self):
        window = self._window(lock_target=False)

        window._set_result(self._result())

        self.assertIn("等待稳定锁定", window.detect_state.text)
        self.assertIn("valid=0", window.robot_coord.text)
        self.assertIn("不上报坐标", window.robot_coord.text)

    def test_locked_target_text_marks_valid_coordinates(self):
        window = self._window(lock_target=True)

        window._set_result(self._result())

        self.assertIn("已稳定锁定", window.detect_state.text)
        self.assertIn("valid=1", window.robot_coord.text)
        self.assertIn("dcx 48 / dcy -43", window.robot_coord.text)

    def test_unconfigured_aim_reference_is_shown_as_fail_closed(self):
        window = self._window(lock_target=True)
        window.arm.aim_reference_configured = False

        def unconfigured(_target):
            raise ValueError("激光瞄准参考像素未标定")

        window.arm.aim_corrected_error = unconfigured

        window._set_result(self._result())

        self.assertIn("未标定", window.robot_coord.text)
        self.assertIn("valid=0", window.robot_coord.text)

    def test_locked_session_does_not_display_a_different_candidate_after_loss(self):
        window = self._window(lock_target=False)
        window._target_identity_lock = SimpleNamespace(active=True)
        window._last_target_lock_reason = "锁定目标暂时丢失，禁止切换其他候选"

        window._set_result(self._result())

        self.assertEqual(window.target_count.text, "0")
        self.assertEqual(window.summary_target_value.text, "0")
        self.assertEqual(window.pixel_coord.text, "--")
        self.assertIn("禁止切换", window.detect_state.text)


@unittest.skipIf(IMPORT_ERROR is not None, "Qt application dependencies unavailable")
class QtTargetStartGateTest(unittest.TestCase):
    @staticmethod
    def _window():
        window = MainWindow.__new__(MainWindow)
        window.arm = _TargetArmSpy()
        window.args = SimpleNamespace(demo=False, protocol_debug_controls=False)
        window._workflow_active = True
        window._fault_hold = False
        window._restart_blocked = False
        window._distance_safe_latched = True
        window._selected_view_ready = True
        window._target_ctrl_on = False
        window._target_ready = False
        window._waiting_for_arm_ready = False
        window._protocol_session_generation = 7
        window._ready_after_action = ""
        window._arm_ready_wait_started = None
        window.workflow_state = _Label()
        window.ready_wait_value = _Label()
        window.status_text = ""
        window.events = []
        window._set_status = lambda text: setattr(window, "status_text", str(text))
        window._log_event = window.events.append
        window._refresh_controls = lambda: None
        return window

    @classmethod
    def _refresh_window(cls):
        window = cls._window()
        window._waiting_for_safe_distance = False
        window._reference_capturing = False
        window._safety_capturing = False
        window._capture_sequence_active = False
        window._safety_blocking = False
        window._capture_complete = True
        window._capture_pending_point = 0
        window._detection_running = False
        window._align_done = False
        window._output_on = False
        window._current_arm_target = lambda: None
        window.pages = SimpleNamespace(currentIndex=lambda: 2)
        window.nav_buttons = []
        for name in (
            "start_btn",
            "finish_btn",
            "end_finish_btn",
            "capture_reference_btn",
            "capture_safety_btn",
            "capture_faces_btn",
            "binocular_capture_faces_btn",
            "end_capture_faces_btn",
            "once_btn",
            "stop_btn",
            "end_stop_btn",
            "preview_btn",
            "fire_btn",
            "capture_left_btn",
            "capture_center_btn",
            "capture_right_btn",
            "select_left_btn",
            "select_center_btn",
            "select_right_btn",
            "capture_home_btn",
        ):
            setattr(window, name, _Button())
        window.face_select_buttons = {}
        window.triptych_worker = None
        return window

    def test_start_without_stable_target_still_sends_target_ctrl(self):
        window = self._window()
        window._current_arm_target = lambda: None

        with patch(
            "arm_mole_qt.app.main.QMessageBox.question",
            return_value=QMessageBox.Yes,
        ), patch("arm_mole_qt.app.main.QMessageBox.warning") as warning:
            window._fire_laser()

        warning.assert_not_called()
        self.assertEqual(window.arm.start_calls, 1)
        self.assertTrue(window._waiting_for_arm_ready)
        self.assertEqual(window._ready_after_action, "target_start")
        self.assertEqual(window.ready_wait_value.text, "等待 TARGET_READY")
        self.assertIn("valid=0", window.status_text)

    def test_confirmation_still_rechecks_the_safety_latch(self):
        window = self._window()

        def lose_safety_latch(*_args, **_kwargs):
            window._distance_safe_latched = False
            return QMessageBox.Yes

        with patch(
            "arm_mole_qt.app.main.QMessageBox.question",
            side_effect=lose_safety_latch,
        ), patch("arm_mole_qt.app.main.QMessageBox.warning") as warning:
            window._fire_laser()

        self.assertEqual(window.arm.start_calls, 0)
        warning.assert_called_once()
        self.assertIn("流程状态已变化", str(warning.call_args.args[1]))

    def test_unconfigured_aim_reference_blocks_target_ctrl_before_confirmation(self):
        window = self._window()
        window.arm.aim_reference_configured = False

        with patch(
            "arm_mole_qt.app.main.QMessageBox.question"
        ) as question, patch(
            "arm_mole_qt.app.main.QMessageBox.warning"
        ) as warning:
            window._fire_laser()

        question.assert_not_called()
        self.assertEqual(window.arm.start_calls, 0)
        warning.assert_called_once()
        self.assertIn("未标定", str(warning.call_args.args[1]))
        self.assertIn("TARGET_CTRL START", str(warning.call_args.args[2]))

    def test_start_button_enabled_without_target_but_preview_stays_disabled(self):
        window = self._refresh_window()
        window._detection_running = True

        MainWindow._refresh_controls(window)

        self.assertTrue(window.fire_btn.enabled)
        self.assertFalse(window.preview_btn.enabled)

    def test_end_cancel_matches_global_cancel_in_active_contexts(self):
        contexts = {
            "target": lambda window: setattr(window, "_target_ready", True),
            "detection": lambda window: setattr(window, "_detection_running", True),
            "ordinary busy": lambda window: setattr(window.arm, "control_busy", True),
        }

        for context, enable_context in contexts.items():
            with self.subTest(context=context):
                window = self._refresh_window()
                window._workflow_active = False
                window._selected_view_ready = False
                enable_context(window)

                MainWindow._refresh_controls(window)

                self.assertTrue(window.stop_btn.enabled)
                self.assertEqual(window.end_stop_btn.enabled, window.stop_btn.enabled)

    def test_end_finish_matches_global_finish_in_target_hold(self):
        window = self._refresh_window()
        window._workflow_active = True
        window._selected_view_ready = True
        window._target_ctrl_on = True
        window._target_ready = True
        window._output_on = False

        MainWindow._refresh_controls(window)

        self.assertTrue(window.finish_btn.enabled)
        self.assertTrue(window.end_finish_btn.enabled)

    def test_output_off_after_laser_output_auto_requests_return_home(self):
        window = self._window()
        window.worker = SimpleNamespace(stop_detection=Mock())
        window._workflow_active = True
        window._target_ctrl_on = True
        window._target_ready = True
        window._align_done = True
        window._output_on = True
        window._detection_running = True
        window._recoverable_vision_loss_active = True
        window._refresh_controls = Mock()

        window._handle_output_status(0)

        self.assertEqual(window.arm.finish_calls, 1)
        self.assertFalse(window._target_ready)
        self.assertFalse(window._target_ctrl_on)
        self.assertFalse(window._align_done)
        self.assertFalse(window._output_on)
        self.assertFalse(window._detection_running)
        self.assertFalse(window._recoverable_vision_loss_active)
        self.assertTrue(window._waiting_for_arm_ready)
        self.assertEqual(window._ready_after_action, "finish_return_home")
        self.assertEqual(window.ready_wait_value.text, "等待 RETURN_HOME_DONE")
        self.assertIn("OUTPUT_OFF", window.status_text)
        self.assertTrue(
            any("自动返回 HOME" in str(event) for event in window.events),
            window.events,
        )

    def test_output_off_does_not_send_duplicate_finish_while_return_pending(self):
        window = self._window()
        window.worker = None
        window._workflow_active = True
        window._output_on = True
        window._ready_after_action = "finish_return_home"

        window._handle_output_status(0)

        self.assertEqual(window.arm.finish_calls, 0)
        self.assertEqual(window.ready_wait_value.text, "可完成/返回")

    def test_both_cancel_buttons_disable_while_abort_is_pending(self):
        abort_states = {
            "controller pending": lambda window: setattr(
                window.arm, "abort_pending", True
            ),
            "waiting abort hold": lambda window: setattr(
                window, "_ready_after_action", "abort_hold"
            ),
        }

        for state, set_abort_state in abort_states.items():
            with self.subTest(state=state):
                window = self._refresh_window()
                set_abort_state(window)

                MainWindow._refresh_controls(window)

                self.assertFalse(window.stop_btn.enabled)
                self.assertEqual(window.end_stop_btn.enabled, window.stop_btn.enabled)

    def test_target_ready_without_lock_sends_explicit_invalid_sample(self):
        window = self._window()
        window._target_ready = True
        window._measure_position_ready = False
        window._waiting_for_safe_distance = False
        window._last_serial_vision_error = 0.0
        window.serial_vision_interval = 0.0
        window._latest_result_time = 123.0
        window._last_sent_vision_sample_time = 100.0
        window._current_arm_target = lambda: None

        window._service_unified_serial_keepalive()

        self.assertEqual(window.arm.target_samples, [None])
        self.assertEqual(window._last_sent_vision_sample_time, 100.0)


@unittest.skipIf(IMPORT_ERROR is not None, "Qt application dependencies unavailable")
class QtSafeDistanceTimeoutErrorTest(unittest.TestCase):
    def test_timeout_error_shows_retreat_instead_of_generic_fault(self):
        window = MainWindow.__new__(MainWindow)
        window._waiting_for_safe_distance = True
        window._distance_safe_latched = True
        window._preposition_ready = True
        window._latest_safe_measurement_valid = True
        window._latest_safe_invalid_reason = ""
        window._latest_safe_depth_mm = 280.0
        window._latest_safe_depth_time = time.monotonic()
        window.binocular_safe_distance_value = _Label()
        window.workflow_state = _Label()
        window.ready_wait_value = _Label()
        window.clear_calls = 0
        window.fault_calls = []
        window.status_text = ""
        window._clear_safe_scene_binding = lambda: setattr(
            window, "clear_calls", window.clear_calls + 1
        )
        window._stop_automatic_progress = lambda reason: window.fault_calls.append(reason)
        window._set_status = lambda text: setattr(window, "status_text", str(text))
        status = SimpleNamespace(
            error_code=ERROR_SAFE_DISTANCE_TIMEOUT,
            func=RA6_SAFE_DISTANCE_FUNC,
            label="SAFE_DISTANCE_UNSAFE_ERR_SAFE_DISTANCE_TIMEOUT",
        )

        self.assertTrue(window._handle_protocol_error(status))

        self.assertEqual(window.clear_calls, 1)
        self.assertFalse(window._waiting_for_safe_distance)
        self.assertFalse(window._distance_safe_latched)
        self.assertFalse(window._preposition_ready)
        self.assertFalse(window._latest_safe_measurement_valid)
        self.assertIn("超时", window.workflow_state.text)
        self.assertIn("安全撤离中", window.workflow_state.text)
        self.assertEqual(window.ready_wait_value.text, "等待 RETREAT_DONE")
        self.assertIn("固化撤离", window.status_text)
        self.assertIn("已过期", window.binocular_safe_distance_value.text)
        self.assertEqual(window.fault_calls, [])


@unittest.skipIf(IMPORT_ERROR is not None, "Qt application dependencies unavailable")
class QtTerminalSerialFailureTest(unittest.TestCase):
    def test_terminal_tx_failure_closes_without_resending_abort(self):
        window = MainWindow.__new__(MainWindow)
        calls = []

        class ArmSpy:
            def close(self):
                calls.append("close")

        window.arm = ArmSpy()
        window.serial_state = _Label()
        window._stop_automatic_progress = (
            lambda reason, restart_blocked=False: calls.append(
                (str(reason), bool(restart_blocked))
            )
        )
        event = (
            "串口TX写入失败且无法确认是否已写入部分帧：Write timeout；"
            "已立即停止后续发送，防止残帧与下一帧交叠"
        )

        self.assertTrue(window._is_terminal_serial_transport_event(event))
        window._handle_terminal_serial_transport_event(event)

        self.assertEqual(calls[-1], "close")
        self.assertEqual(calls[0][1], True)
        self.assertIn("通信状态未知", calls[0][0])
        self.assertIn("重启 Qt", window.serial_state.text)


if __name__ == "__main__":
    unittest.main()
