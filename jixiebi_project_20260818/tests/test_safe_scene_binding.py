import os
import time
import unittest
from types import SimpleNamespace
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

try:
    from arm_mole_qt.app.main import MainWindow, SafeFaceSnapshot
    from jetson_nano_vision.jetson_vision.scene_guard import FaceSceneGuard
except ImportError as exc:  # pragma: no cover - optional on non-Qt dev hosts
    MainWindow = None
    SafeFaceSnapshot = None
    FaceSceneGuard = None
    IMPORT_ERROR = exc
else:
    IMPORT_ERROR = None


@unittest.skipIf(IMPORT_ERROR is not None, "Qt application dependencies unavailable")
class SafeSceneBindingTest(unittest.TestCase):
    def _window(self, snapshot, latest_box, latest_time):
        window = MainWindow.__new__(MainWindow)
        window._scene_guard = FaceSceneGuard(
            center_shift_ratio=0.08,
            scale_ratio=1.25,
            missing_timeout_s=0.8,
        )
        window._scene_abort_triggered = False
        window._safe_face_snapshots = {7: snapshot}
        window._approved_safe_face_snapshot = None
        window._latest_rgbd_detections = {"face": {"box": latest_box}}
        window._latest_rgbd_rgb_size = snapshot.frame_size
        window._latest_rgbd_detection_time = latest_time
        window.abort_reasons = []

        def record_abort(reason):
            window.abort_reasons.append(reason)
            window._scene_abort_triggered = True

        window._abort_for_scene_change = record_abort
        self.assertTrue(window._provision_safe_scene_snapshot(snapshot))
        return window

    def test_safe_ok_uses_the_snapshot_with_the_same_seq(self):
        snapshot = SafeFaceSnapshot(
            box=(100.0, 100.0, 220.0, 260.0),
            frame_size=(640, 480),
            rgb_sample_time=1.0,
            depth_sample_time=1.02,
        )
        window = self._window(snapshot, snapshot.box, 1.1)

        self.assertTrue(window._approve_safe_scene_snapshot(7))
        self.assertIs(window._approved_safe_face_snapshot, snapshot)
        self.assertFalse(window.abort_reasons)

    def test_safe_ok_with_unknown_seq_is_rejected(self):
        snapshot = SafeFaceSnapshot(
            box=(100.0, 100.0, 220.0, 260.0),
            frame_size=(640, 480),
            rgb_sample_time=1.0,
            depth_sample_time=1.02,
        )
        window = self._window(snapshot, snapshot.box, 1.1)

        self.assertFalse(window._approve_safe_scene_snapshot(8))
        self.assertIsNone(window._approved_safe_face_snapshot)

    def test_motion_between_safe_send_and_safe_ok_is_rejected(self):
        snapshot = SafeFaceSnapshot(
            box=(100.0, 100.0, 220.0, 260.0),
            frame_size=(640, 480),
            rgb_sample_time=1.0,
            depth_sample_time=1.02,
        )
        moved_box = (260.0, 100.0, 380.0, 260.0)
        window = self._window(snapshot, moved_box, 1.1)

        self.assertFalse(window._approve_safe_scene_snapshot(7))
        self.assertTrue(window.abort_reasons)


@unittest.skipIf(IMPORT_ERROR is not None, "Qt application dependencies unavailable")
class QtIdleViewSelectionSceneGuardTest(unittest.TestCase):
    FACE_BOX = (100.0, 100.0, 220.0, 260.0)
    FRAME_SIZE = (640, 480)

    def _window(self, sample_time=10.0):
        window = MainWindow.__new__(MainWindow)
        capture_select_calls = []

        def capture_select(point_id):
            capture_select_calls.append(int(point_id))
            return True, "CAPTURE_SELECT sent"

        window.arm = SimpleNamespace(
            connected=True,
            control_busy=False,
            capture_select=capture_select,
        )
        window.capture_select_calls = capture_select_calls
        window.args = SimpleNamespace(demo=False)
        window._workflow_active = True
        window._distance_safe_latched = True
        window._capture_complete = True
        window._capture_sequence_active = False
        window._capture_home_phase = ""
        window._selected_capture_point = 0
        window._selected_view_ready = False
        window._target_ctrl_on = False
        window._target_ready = False
        window._output_on = False
        window._waiting_for_arm_ready = False
        window._waiting_for_safe_distance = False
        window._fault_hold = False
        window._scene_abort_triggered = False
        window._scene_monitor_degraded = False
        window.serial_safe_distance_max_age = 0.5
        window._scene_guard = FaceSceneGuard(
            center_shift_ratio=0.08,
            scale_ratio=1.25,
            missing_timeout_s=0.8,
        )
        snapshot = SafeFaceSnapshot(
            box=self.FACE_BOX,
            frame_size=self.FRAME_SIZE,
            rgb_sample_time=sample_time,
            depth_sample_time=sample_time + 0.02,
        )
        self.assertTrue(
            window._scene_guard.latch(
                snapshot.box,
                snapshot.frame_size,
                snapshot.rgb_sample_time,
            )
        )
        window._approved_safe_face_snapshot = snapshot
        window._latest_rgbd_detections = {"face": {"box": self.FACE_BOX}}
        window._latest_rgbd_rgb_size = self.FRAME_SIZE
        window._latest_rgbd_detection_time = sample_time
        window.abort_reasons = []
        window.status_messages = []
        window.event_messages = []

        def record_abort(reason):
            window.abort_reasons.append(str(reason))
            window._scene_abort_triggered = True

        window._abort_for_scene_change = record_abort
        window._set_status = window.status_messages.append
        window._log_event = window.event_messages.append
        window._refresh_controls = lambda: None
        window._reset_target_identity_lock = lambda: None
        window._update_capture_selection_ui = lambda: None
        window._clear_current_target = lambda _reason: None
        window._switch_page = lambda _page: None
        window.worker = None
        window.selected_view_value = SimpleNamespace(setText=lambda _text: None)
        window.workflow_state = SimpleNamespace(setText=lambda _text: None)
        return window

    def test_missing_face_degrades_but_does_not_abort_while_waiting_for_selection(self):
        window = self._window()

        first_missing = window._scene_guard.update(None, self.FRAME_SIZE, 10.1)
        window._handle_scene_guard_decision(first_missing, face_present=False)
        timed_out = window._scene_guard.update(None, self.FRAME_SIZE, 10.91)
        self.assertTrue(timed_out.abort)
        window._handle_scene_guard_decision(timed_out, face_present=False)

        self.assertTrue(window._scene_monitor_degraded)
        self.assertEqual(window.abort_reasons, [])

    def test_stalled_monitor_degrades_but_does_not_abort_while_waiting_for_selection(self):
        window = self._window()

        stalled = window._scene_guard.poll(10.81)
        self.assertTrue(stalled.abort)
        window._handle_scene_guard_decision(stalled)

        self.assertTrue(window._scene_monitor_degraded)
        self.assertEqual(window.abort_reasons, [])

    def test_stable_face_recovers_selection_after_temporary_degradation(self):
        window = self._window()

        missing = window._scene_guard.update(None, self.FRAME_SIZE, 10.1)
        window._handle_scene_guard_decision(missing, face_present=False)
        self.assertTrue(window._scene_monitor_degraded)

        stable = window._scene_guard.update(self.FACE_BOX, self.FRAME_SIZE, 10.2)
        window._handle_scene_guard_decision(stable, face_present=True)

        self.assertFalse(window._scene_monitor_degraded)
        self.assertEqual(window.abort_reasons, [])

    def test_real_face_movement_still_aborts_while_waiting_for_selection(self):
        window = self._window()
        moved_box = (260.0, 100.0, 380.0, 260.0)

        moved = window._scene_guard.update(moved_box, self.FRAME_SIZE, 10.1)
        self.assertTrue(moved.abort)
        window._handle_scene_guard_decision(moved, face_present=True)

        self.assertEqual(len(window.abort_reasons), 1)
        self.assertIn("人脸中心位移", window.abort_reasons[0])

    def test_missing_face_aborts_after_a_view_has_been_selected(self):
        window = self._window()
        window._selected_capture_point = 1

        first_missing = window._scene_guard.update(None, self.FRAME_SIZE, 10.1)
        window._handle_scene_guard_decision(first_missing, face_present=False)
        timed_out = window._scene_guard.update(None, self.FRAME_SIZE, 10.91)
        self.assertTrue(timed_out.abort)
        window._handle_scene_guard_decision(timed_out, face_present=False)

        self.assertEqual(len(window.abort_reasons), 1)
        self.assertIn("人脸持续丢失", window.abort_reasons[0])

    def test_stale_yolo_result_blocks_capture_select(self):
        now = time.monotonic()
        window = self._window(sample_time=now - 0.6)

        ready, reason = window._scene_ready_for_view_selection(now)
        self.assertFalse(ready)
        self.assertIn("过期", reason)

        with patch("arm_mole_qt.app.main.time.monotonic", return_value=now), patch(
            "arm_mole_qt.app.main.QMessageBox.warning"
        ) as warning:
            window._send_capture_select(1)

        warning.assert_called_once()
        self.assertEqual(window.capture_select_calls, [])
        self.assertEqual(window._selected_capture_point, 0)

    def test_fresh_matching_face_allows_one_capture_select(self):
        now = time.monotonic()
        window = self._window(sample_time=now - 0.1)

        ready, reason = window._scene_ready_for_view_selection(now)
        self.assertTrue(ready, reason)

        with patch("arm_mole_qt.app.main.time.monotonic", return_value=now), patch(
            "arm_mole_qt.app.main.QMessageBox.warning"
        ) as warning:
            window._send_capture_select(2)

        warning.assert_not_called()
        self.assertEqual(window.capture_select_calls, [2])
        self.assertEqual(window._selected_capture_point, 2)


@unittest.skipIf(IMPORT_ERROR is not None, "Qt application dependencies unavailable")
class QtProtocolRaceGuardTest(unittest.TestCase):
    class Label:
        def __init__(self):
            self.text = ""

        def setText(self, text):
            self.text = str(text)

    def test_repeated_abort_does_not_replace_pending_request(self):
        window = MainWindow.__new__(MainWindow)
        arm = SimpleNamespace(abort_pending=True, abort_calls=0)

        def abort_workflow():
            arm.abort_calls += 1
            return True, "unexpected"

        arm.abort_workflow = abort_workflow
        window.arm = arm
        window._ready_after_action = "abort_hold"
        window.workflow_state = self.Label()
        window.ready_wait_value = self.Label()
        window._set_status = lambda _text: None
        window._refresh_controls = lambda: None

        window._cancel_workflow("重复点击")

        self.assertEqual(arm.abort_calls, 0)
        self.assertEqual(window.ready_wait_value.text, "等待 ABORTED_HOLD")

    def test_local_scene_abort_discards_home_ready_side_effect(self):
        window = MainWindow.__new__(MainWindow)
        home_ready = SimpleNamespace(
            protocol="unified",
            func=0x11,
            value=0x01,
            error_code=0,
            label="CAPTURE_HOME_READY",
            seq=4,
        )
        arm = SimpleNamespace(
            abort_pending=False,
            poll_statuses=lambda: [home_ready],
            service_transport=lambda: [],
        )
        window.arm = arm
        window._ready_after_action = ""
        window._scene_abort_triggered = False
        window._capture_home_phase = "before"
        window.forward_calls = 0

        def poll_scene_guard():
            window._scene_abort_triggered = True
            window._ready_after_action = "abort_hold"

        window._poll_scene_guard = poll_scene_guard
        window._request_next_face_capture = lambda: setattr(
            window,
            "forward_calls",
            window.forward_calls + 1,
        )
        window._log_event = lambda _text: None
        window._update_arm_ready_wait = lambda: None
        window._service_unified_serial_keepalive = lambda: None
        window._refresh_controls = lambda: None

        window._poll_arm_status()

        self.assertEqual(window.forward_calls, 0)

    def test_vision_lost_does_not_discard_target_handshake_in_same_batch(self):
        window = MainWindow.__new__(MainWindow)
        statuses = [
            SimpleNamespace(
                protocol="unified",
                func=0x04,
                value=0x01,
                error_code=0,
                label="TARGET_CTRL_ON",
                seq=9,
            ),
            SimpleNamespace(
                protocol="unified",
                func=0x01,
                value=0x01,
                error_code=0,
                label="TARGET_READY",
                seq=9,
            ),
            SimpleNamespace(
                protocol="unified",
                func=0x03,
                value=0x00,
                error_code=0x0A,
                label="OUTPUT_OFF_ERR_VISION_LOST",
                seq=9,
            ),
        ]
        window.arm = SimpleNamespace(
            abort_pending=False,
            poll_statuses=lambda: statuses,
            service_transport=lambda: [],
        )
        window._ready_after_action = "target_start"
        window._scene_abort_triggered = False
        window._target_ctrl_on = False
        window._target_ready = False
        window._waiting_for_arm_ready = True
        window._arm_ready_wait_started = 1.0
        window._detection_running = False
        window._last_serial_vision_error = 1.0
        window._last_sent_vision_sample_time = 1.0
        window._output_on = True
        window._vision_valid = True
        window._recoverable_vision_loss_active = False
        window._align_done = True
        window.worker = None
        window.ra6_state = self.Label()
        window.workflow_state = self.Label()
        window.ready_wait_value = self.Label()
        window._poll_scene_guard = lambda: None
        window._set_status = lambda _text: None
        window._log_event = lambda _text: None
        window._update_arm_ready_wait = lambda: None
        window._service_unified_serial_keepalive = lambda: None
        window._refresh_controls = lambda: None

        window._poll_arm_status()

        self.assertTrue(window._target_ctrl_on)
        self.assertTrue(window._target_ready)
        self.assertFalse(window._output_on)
        self.assertFalse(window._align_done)

    def test_recovery_in_same_batch_preserves_later_align_and_output(self):
        window = MainWindow.__new__(MainWindow)
        statuses = [
            SimpleNamespace(
                protocol="unified",
                func=0x07,
                value=0x00,
                error_code=0x0A,
                label="VISION_LOST_ERR_VISION_LOST",
                seq=9,
            ),
            SimpleNamespace(
                protocol="unified",
                func=0x07,
                value=0x01,
                error_code=0,
                label="VISION_VALID",
                seq=9,
            ),
            SimpleNamespace(
                protocol="unified",
                func=0x02,
                value=0x01,
                error_code=0,
                label="ALIGN_DONE",
                seq=9,
            ),
            SimpleNamespace(
                protocol="unified",
                func=0x03,
                value=0x01,
                error_code=0,
                label="OUTPUT_ON",
                seq=9,
            ),
        ]
        window.arm = SimpleNamespace(
            abort_pending=False,
            poll_statuses=lambda: statuses,
            service_transport=lambda: [],
        )
        window._ready_after_action = ""
        window._scene_abort_triggered = False
        window._recoverable_vision_loss_active = False
        window._target_ctrl_on = True
        window._target_ready = True
        window._waiting_for_arm_ready = False
        window._arm_ready_wait_started = None
        window._detection_running = True
        window._output_on = False
        window._vision_valid = True
        window._align_done = False
        window.worker = None
        window.ra6_state = self.Label()
        window.workflow_state = self.Label()
        window.ready_wait_value = self.Label()
        window._poll_scene_guard = lambda: None
        window._set_status = lambda _text: None
        window._log_event = lambda _text: None
        window._update_arm_ready_wait = lambda: None
        window._service_unified_serial_keepalive = lambda: None
        window._refresh_controls = lambda: None

        window._poll_arm_status()

        self.assertTrue(window._vision_valid)
        self.assertFalse(window._recoverable_vision_loss_active)
        self.assertTrue(window._align_done)
        self.assertTrue(window._output_on)


if __name__ == "__main__":
    unittest.main()
