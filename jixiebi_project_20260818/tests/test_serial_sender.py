import sys
import threading
import time
import types
import unittest
from unittest.mock import patch

from jetson_nano_vision.jetson_vision.serial_sender import (
    CONTROL_ACK_TIMEOUT_S,
    SERIAL_WRITE_TIMEOUT_S,
    SerialSender,
)
from jetson_nano_vision.jetson_vision.vision_protocol import (
    ERROR_INVALID_PARAM,
    ERROR_NONE,
    MSG_CAPTURE_CTRL,
    MSG_ERROR,
    MSG_STATUS,
    MSG_VISION_ERROR,
    MSG_WORKFLOW_CTRL,
    STATUS_CAPTURE_HOME_READY,
    STATUS_COMMAND_ACK,
    STATUS_SAFE_DISTANCE,
    STATUS_TARGET_CTRL,
    STATUS_TARGET_READY,
    STATUS_WORKFLOW,
    WORKFLOW_ABORTED_HOLD,
    WORKFLOW_DISTANCE_SAFE_LATCHED,
    WORKFLOW_MEASURE_POSITION_READY,
    WORKFLOW_RETURN_HOME_DONE,
    WORKFLOW_START_ACCEPTED,
    WORKFLOW_ACTION_START_MEASURE_CONTINUOUS,
    WORKFLOW_ACTION_FINISH_RETURN_HOME,
    Ra6Status,
    build_unified_frame,
    parse_unified_frame,
)


class FakeSerial:
    def __init__(self, *args, **kwargs):
        self.is_open = True
        self.writes = []
        self._rx = bytearray()
        self.write_error = None

    @property
    def in_waiting(self):
        return len(self._rx)

    def write(self, data):
        if self.write_error is not None:
            raise self.write_error
        data = bytes(data)
        self.writes.append(data)
        return len(data)

    def read(self, size):
        data = bytes(self._rx[:size])
        del self._rx[:size]
        return data

    def close(self):
        self.is_open = False


def status(seq, event, value, error=ERROR_NONE):
    return Ra6Status(
        func=event,
        value=value,
        error_code=error,
        protocol="unified",
        seq=seq,
        msg_type=MSG_STATUS,
    )


class SerialSenderTest(unittest.TestCase):
    def setUp(self):
        self.fake_serial = FakeSerial()
        self.serial_open_kwargs = {}

        def open_serial(*args, **kwargs):
            self.serial_open_kwargs = dict(kwargs)
            return self.fake_serial

        serial_module = types.SimpleNamespace(Serial=open_serial)
        self.serial_patch = patch.dict(sys.modules, {"serial": serial_module})
        self.serial_patch.start()
        self.sender = SerialSender("fake", auto_heartbeat=False)

    def tearDown(self):
        self.sender.close()
        self.serial_patch.stop()

    def _accept_and_measure_ready(self, continuous=False):
        self.assertTrue(self.sender.send_workflow_start(continuous=continuous))
        seq = self.sender.pending_control.seq
        self.assertTrue(self.sender._observe_control_status(status(seq, STATUS_COMMAND_ACK, 1)))
        self.assertTrue(
            self.sender._observe_control_status(
                status(seq, STATUS_WORKFLOW, WORKFLOW_MEASURE_POSITION_READY)
            )
        )
        self.assertTrue(self.sender.measure_window_open)
        return seq

    def test_default_write_timeout_tolerates_jetson_ch34x_startup_load(self):
        self.assertEqual(
            self.serial_open_kwargs.get("write_timeout"),
            SERIAL_WRITE_TIMEOUT_S,
        )
        self.assertEqual(SERIAL_WRITE_TIMEOUT_S, 0.200)

    def test_jetson_tx_seq_uses_one_through_ff_and_skips_zero_on_wrap(self):
        self.sender._seq = 0xFE

        for tick in range(4):
            self.assertTrue(self.sender.send_heartbeat(tick))

        seqs = [frame[4] for frame in self.fake_serial.writes]
        self.assertEqual(seqs, [0xFE, 0xFF, 0x01, 0x02])
        self.assertNotIn(0x00, seqs)

    def test_new_control_skips_recent_same_type_seq_after_stream_wrap(self):
        self.sender._seq = 0x6C
        self.assertTrue(self.sender.send_capture_home())
        first_seq = self.sender.pending_control.seq
        self.assertEqual(first_seq, 0x6C)
        self.assertTrue(
            self.sender._observe_control_status(
                status(first_seq, STATUS_COMMAND_ACK, 1)
            )
        )
        self.assertTrue(
            self.sender._observe_control_status(
                status(first_seq, STATUS_CAPTURE_HOME_READY, 1)
            )
        )

        # HEARTBEAT alone can wrap the shared 8-bit sequence while RA6M5's
        # control cache still contains the old CAPTURE_CTRL key.
        for tick in range(254):
            self.assertTrue(self.sender.send_heartbeat(tick))
        self.assertEqual(self.sender._seq, first_seq)

        self.assertTrue(self.sender.send_capture_select(3))
        second_seq = self.sender.pending_control.seq
        self.assertEqual(second_seq, 0x6D)
        self.assertNotEqual(second_seq, first_seq)

        first_control = parse_unified_frame(self.fake_serial.writes[0])
        second_control = parse_unified_frame(self.fake_serial.writes[-1])
        self.assertEqual(first_control.msg_type, MSG_CAPTURE_CTRL)
        self.assertEqual(second_control.msg_type, MSG_CAPTURE_CTRL)
        self.assertEqual(first_control.seq, first_seq)
        self.assertEqual(second_control.seq, second_seq)

    def test_ra6m5_async_error_with_seq_zero_is_still_parsed(self):
        self.fake_serial._rx.extend(
            build_unified_frame(
                MSG_ERROR,
                0x00,
                bytes([ERROR_INVALID_PARAM]),
            )
        )

        statuses = self.sender.poll_statuses()

        self.assertEqual(len(statuses), 1)
        self.assertEqual(statuses[0].seq, 0x00)
        self.assertEqual(statuses[0].protocol, "unified_error")
        self.assertEqual(statuses[0].error_code, ERROR_INVALID_PARAM)

    def _open_target_vision_session(self, seq=0x44):
        with self.sender._control_lock:
            self.sender._active_target_seq = seq
            self.sender._reset_vision_mailbox_locked()

    def _vision_frames(self):
        return [
            parse_unified_frame(frame)
            for frame in self.fake_serial.writes
            if parse_unified_frame(frame).msg_type == MSG_VISION_ERROR
        ]

    def test_background_vision_session_starts_invalid_once_then_stays_silent(self):
        self._open_target_vision_session()

        self.sender.service_vision(10.00)
        self.sender.service_vision(10.10)
        self.sender.service_vision(10.15)
        self.sender.service_vision(11.00)

        frames = self._vision_frames()
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0].payload, b"\x00\x00\x00\x00\x00")

    def test_background_vision_consumes_valid_sample_once_without_gap_invalid(self):
        self._open_target_vision_session()
        self.assertTrue(self.sender.publish_vision_sample(12, -7, True, 20.00))

        self.sender.service_vision(20.05)
        self.sender.service_vision(20.20)
        self.sender.service_vision(20.50)

        frames = self._vision_frames()
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0].payload[-1], 1)
        self.assertEqual(frames[0].payload[:4], b"\x0c\x00\xf9\xff")

    def test_background_vision_valid_expiry_sends_one_invalid_then_stays_silent(self):
        self._open_target_vision_session()
        self.assertTrue(self.sender.publish_vision_sample(12, -7, True, 20.00))

        self.sender.service_vision(20.05)
        self.sender.service_vision(20.51)
        self.sender.service_vision(20.70)
        self.assertEqual(len(self._vision_frames()), 1)
        self.sender.service_vision(20.76)
        self.sender.service_vision(21.50)

        frames = self._vision_frames()
        self.assertEqual(len(frames), 2)
        self.assertEqual(frames[0].payload[-1], 1)
        self.assertEqual(frames[1].payload, b"\x00\x00\x00\x00\x00")

    def test_background_vision_explicit_invalid_waits_for_rate_gate_once(self):
        self._open_target_vision_session()
        self.assertTrue(self.sender.publish_vision_sample(12, -7, True, 20.00))
        self.sender.service_vision(20.05)

        self.assertTrue(self.sender.publish_vision_sample(0, 0, False, 20.10))
        self.sender.service_vision(20.11)
        self.assertEqual(len(self._vision_frames()), 1)
        self.sender.service_vision(20.20)
        self.sender.service_vision(20.35)

        frames = self._vision_frames()
        self.assertEqual(len(frames), 2)
        self.assertEqual(frames[0].payload[-1], 1)
        self.assertEqual(frames[1].payload, b"\x00\x00\x00\x00\x00")

    def test_background_vision_explicit_invalid_survives_newer_recovery_sample(self):
        self._open_target_vision_session()
        self.assertTrue(self.sender.publish_vision_sample(0, 0, False, 25.00))
        self.assertTrue(self.sender.publish_vision_sample(8, 9, True, 25.01))

        self.sender.service_vision(25.02)
        self.sender.service_vision(25.17)

        frames = self._vision_frames()
        self.assertEqual(len(frames), 2)
        self.assertEqual(frames[0].payload, b"\x00\x00\x00\x00\x00")
        self.assertEqual(frames[1].payload[-1], 1)
        self.assertEqual(frames[1].payload[:4], b"\x08\x00\x09\x00")

    def test_background_vision_rejects_stale_and_out_of_order_samples(self):
        self._open_target_vision_session()
        self.assertTrue(self.sender.publish_vision_sample(3, 4, True, 30.00))
        self.assertFalse(self.sender.publish_vision_sample(8, 9, True, 29.00))

        self.sender.service_vision(30.51)
        self.assertEqual(self._vision_frames(), [])
        self.sender.service_vision(30.76)

        frames = self._vision_frames()
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0].payload[-1], 0)

    def test_background_vision_fresh_recovery_during_stale_grace_avoids_invalid(self):
        self._open_target_vision_session()
        self.assertTrue(self.sender.publish_vision_sample(3, 4, True, 50.00))

        self.sender.service_vision(50.05)
        self.sender.service_vision(50.55)
        self.assertTrue(self.sender.publish_vision_sample(5, 6, True, 50.60))
        self.sender.service_vision(50.65)

        frames = self._vision_frames()
        self.assertEqual(len(frames), 2)
        self.assertEqual([frame.payload[-1] for frame in frames], [1, 1])

    def test_background_vision_mailbox_is_bound_to_target_session(self):
        self._open_target_vision_session(seq=0x44)
        self.assertTrue(self.sender.publish_vision_sample(3, 4, True, 40.00))
        with self.sender._control_lock:
            self.sender._active_target_seq = 0x45
            self.sender._reset_vision_mailbox_locked()

        self.sender.service_vision(40.05)

        frames = self._vision_frames()
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0].payload[-1], 0)

    def test_concurrent_messages_share_one_complete_frame_write(self):
        guard = threading.Lock()
        active_writes = 0
        max_active_writes = 0
        results = []

        def slow_write(data):
            nonlocal active_writes, max_active_writes
            frame = bytes(data)
            with guard:
                active_writes += 1
                max_active_writes = max(max_active_writes, active_writes)
            time.sleep(0.005)
            self.fake_serial.writes.append(frame)
            with guard:
                active_writes -= 1
            return len(frame)

        self.fake_serial.write = slow_write
        workers = [
            threading.Thread(
                target=lambda tick=tick: results.append(
                    self.sender.send_heartbeat(tick)
                )
            )
            for tick in range(12)
        ]

        for worker in workers:
            worker.start()
        for worker in workers:
            worker.join(timeout=1.0)

        self.assertTrue(all(not worker.is_alive() for worker in workers))
        self.assertEqual(results, [True] * len(workers))
        self.assertEqual(max_active_writes, 1)
        self.assertEqual(len(self.fake_serial.writes), len(workers))
        for frame in self.fake_serial.writes:
            self.assertEqual(len(frame), 12)
            self.assertEqual(len(parse_unified_frame(frame).payload), 4)

    def test_close_waits_for_active_write_and_blocks_future_writes(self):
        write_entered = threading.Event()
        release_write = threading.Event()
        close_called = threading.Event()
        guard = threading.Lock()
        write_active = False
        close_overlapped_write = False

        def blocking_write(data):
            nonlocal write_active
            frame = bytes(data)
            with guard:
                write_active = True
            write_entered.set()
            self.assertTrue(release_write.wait(timeout=1.0))
            self.fake_serial.writes.append(frame)
            with guard:
                write_active = False
            return len(frame)

        def observed_close():
            nonlocal close_overlapped_write
            with guard:
                close_overlapped_write = write_active
            self.fake_serial.is_open = False
            close_called.set()

        self.fake_serial.write = blocking_write
        self.fake_serial.close = observed_close
        send_worker = threading.Thread(
            target=lambda: self.sender.send_heartbeat(1)
        )
        close_worker = threading.Thread(target=self.sender.close)

        send_worker.start()
        self.assertTrue(write_entered.wait(timeout=1.0))
        close_worker.start()
        time.sleep(0.02)
        self.assertFalse(close_called.is_set())

        release_write.set()
        send_worker.join(timeout=1.0)
        close_worker.join(timeout=1.0)

        self.assertFalse(send_worker.is_alive())
        self.assertFalse(close_worker.is_alive())
        self.assertTrue(close_called.is_set())
        self.assertFalse(close_overlapped_write)
        writes_before = list(self.fake_serial.writes)
        self.assertFalse(self.sender.send_heartbeat(2))
        self.assertEqual(self.fake_serial.writes, writes_before)

    def test_proposed_continuous_workflow_is_rejected_before_write(self):
        seq_before = self.sender._seq

        self.assertFalse(self.sender.send_workflow_start(continuous=True))

        self.assertIn("action=0x04", self.sender.last_error)
        self.assertEqual(self.sender._seq, seq_before)
        self.assertEqual(self.fake_serial.writes, [])
        self.assertIsNone(self.sender.pending_control)
        self.assertFalse(self.sender._continuous_safe_distance)
        self.assertFalse(self.sender.measure_window_open)
        self.assertIsNone(self.sender.active_workflow_seq)

    def test_direct_continuous_workflow_action_is_rejected_before_write(self):
        seq_before = self.sender._seq

        self.assertFalse(
            self.sender.send_workflow_ctrl(
                WORKFLOW_ACTION_START_MEASURE_CONTINUOUS
            )
        )

        self.assertIn("action=0x04", self.sender.last_error)
        self.assertEqual(self.sender._seq, seq_before)
        self.assertEqual(self.fake_serial.writes, [])
        self.assertIsNone(self.sender.pending_control)

    def test_control_retries_use_identical_bytes_and_stop_after_three(self):
        self.assertTrue(self.sender.send_capture_home())
        pending = self.sender.pending_control
        first = self.fake_serial.writes[-1]
        for retry_index in range(3):
            self.sender.service_control(
                pending.last_sent_at + CONTROL_ACK_TIMEOUT_S + 0.001
            )
            self.assertEqual(self.fake_serial.writes[-1], first)
            self.assertEqual(self.sender.pending_control.retries, retry_index + 1)

        pending = self.sender.pending_control
        self.sender.service_control(pending.last_sent_at + CONTROL_ACK_TIMEOUT_S + 0.001)
        self.assertIsNone(self.sender.pending_control)
        self.assertEqual(self.fake_serial.writes, [first, first, first, first])

    def test_business_status_before_ack_is_not_accepted(self):
        self.assertTrue(self.sender.send_capture_home())
        seq = self.sender.pending_control.seq
        self.assertFalse(
            self.sender._observe_control_status(
                status(seq, STATUS_CAPTURE_HOME_READY, 1)
            )
        )
        self.assertIsNotNone(self.sender.pending_control)

    def test_control_cannot_accept_another_command_types_status(self):
        self._accept_and_measure_ready()
        self.assertTrue(self.sender.send_capture_home())
        seq = self.sender.pending_control.seq
        self.assertTrue(
            self.sender._observe_control_status(status(seq, STATUS_COMMAND_ACK, 1))
        )

        self.assertFalse(
            self.sender._observe_control_status(status(seq, STATUS_TARGET_CTRL, 1))
        )
        self.assertFalse(
            self.sender._observe_control_status(status(seq, STATUS_TARGET_READY, 1))
        )
        self.assertIsNotNone(self.sender.pending_control)
        self.assertTrue(
            self.sender._observe_control_status(status(seq, STATUS_CAPTURE_HOME_READY, 1))
        )
        self.assertIsNone(self.sender.pending_control)

    def test_workflow_start_rejects_wrong_same_seq_completion(self):
        self.assertTrue(self.sender.send_workflow_start())
        seq = self.sender.pending_control.seq
        self.assertTrue(
            self.sender._observe_control_status(status(seq, STATUS_COMMAND_ACK, 1))
        )

        self.assertFalse(
            self.sender._observe_control_status(
                status(seq, STATUS_WORKFLOW, WORKFLOW_RETURN_HOME_DONE)
            )
        )
        self.assertIsNotNone(self.sender.pending_control)
        self.assertEqual(self.sender.active_workflow_seq, seq)

        self.assertTrue(
            self.sender._observe_control_status(
                status(seq, STATUS_WORKFLOW, WORKFLOW_START_ACCEPTED)
            )
        )
        self.assertTrue(
            self.sender._observe_control_status(
                status(seq, STATUS_WORKFLOW, WORKFLOW_MEASURE_POSITION_READY)
            )
        )
        self.assertIsNone(self.sender.pending_control)

    def test_workflow_completion_requires_corresponding_pending_action(self):
        workflow_seq = self._accept_and_measure_ready()

        # No FINISH request exists, so a CRC-valid same-session completion
        # must not reset/unlock the current workflow.
        self.assertFalse(
            self.sender._observe_control_status(
                status(workflow_seq, STATUS_WORKFLOW, WORKFLOW_RETURN_HOME_DONE)
            )
        )
        self.assertEqual(self.sender.active_workflow_seq, workflow_seq)

        self.assertTrue(self.sender.send_workflow_finish())
        finish_seq = self.sender.pending_control.seq
        self.assertTrue(
            self.sender._observe_control_status(status(finish_seq, STATUS_COMMAND_ACK, 1))
        )
        self.assertFalse(
            self.sender._observe_control_status(
                status(finish_seq, STATUS_WORKFLOW, WORKFLOW_ABORTED_HOLD)
            )
        )
        self.assertIsNotNone(self.sender.pending_control)
        self.assertTrue(
            self.sender._observe_control_status(
                status(finish_seq, STATUS_WORKFLOW, WORKFLOW_RETURN_HOME_DONE)
            )
        )
        self.assertIsNone(self.sender.pending_control)

    def test_workflow_finish_stops_vision_and_writes_action_02(self):
        self._open_target_vision_session(seq=0x44)
        self.assertTrue(self.sender.publish_vision_sample(4, -2, True, 20.0))

        self.assertTrue(self.sender.send_workflow_finish())

        self.assertIsNotNone(self.sender.pending_control)
        self.assertEqual(self.sender.pending_control.label, "WORKFLOW_FINISH")
        self.assertIsNone(self.sender.active_target_seq)
        finish = parse_unified_frame(self.fake_serial.writes[-1])
        self.assertEqual(finish.msg_type, MSG_WORKFLOW_CTRL)
        self.assertEqual(
            finish.payload,
            bytes([WORKFLOW_ACTION_FINISH_RETURN_HOME]),
        )

        vision_frames_before = len(self._vision_frames())
        self.sender.service_vision(20.5)
        self.assertEqual(len(self._vision_frames()), vision_frames_before)

    def test_uncertain_write_exception_immediately_stops_transport(self):
        self.fake_serial.write_error = OSError("USB gone")

        self.assertFalse(self.sender.send_heartbeat(0))

        events = self.sender.pop_transport_events()
        self.assertTrue(any("无法确认是否已写入部分帧" in event for event in events))
        self.assertTrue(self.sender._heartbeat_stop.is_set())
        writes_before = list(self.fake_serial.writes)
        self.assertFalse(self.sender.send_heartbeat(1))
        self.assertEqual(self.fake_serial.writes, writes_before)

    def test_short_write_immediately_stops_following_frames(self):
        def short_write(data):
            frame = bytes(data)
            self.fake_serial.writes.append(frame[:-2])
            return len(frame) - 2

        self.fake_serial.write = short_write

        self.assertFalse(self.sender.send_heartbeat(1))
        self.assertTrue(self.sender._heartbeat_stop.is_set())
        self.assertFalse(self.sender.is_open)
        events = self.sender.pop_transport_events()
        self.assertTrue(any("串口帧短写" in event for event in events))

        writes_before = list(self.fake_serial.writes)
        self.assertFalse(self.sender.send_heartbeat(2))
        self.assertEqual(self.fake_serial.writes, writes_before)

    def test_waiting_writer_cannot_follow_a_partial_frame(self):
        first_write_entered = threading.Event()
        release_first_write = threading.Event()
        write_calls = []
        write_guard = threading.Lock()

        def first_write_is_short(data):
            frame = bytes(data)
            with write_guard:
                write_calls.append(frame)
                call_number = len(write_calls)
            if call_number == 1:
                first_write_entered.set()
                self.assertTrue(release_first_write.wait(timeout=1.0))
                return len(frame) - 2
            return len(frame)

        self.fake_serial.write = first_write_is_short
        results = []
        first = threading.Thread(
            target=lambda: results.append(self.sender.send_heartbeat(1))
        )
        second = threading.Thread(
            target=lambda: results.append(self.sender.send_heartbeat(2))
        )

        first.start()
        self.assertTrue(first_write_entered.wait(timeout=1.0))
        second.start()
        time.sleep(0.02)
        release_first_write.set()
        first.join(timeout=1.0)
        second.join(timeout=1.0)

        self.assertFalse(first.is_alive())
        self.assertFalse(second.is_alive())
        self.assertEqual(results, [False, False])
        self.assertEqual(len(write_calls), 1)

    def test_safe_tx_and_window_close_are_serialized(self):
        self._accept_and_measure_ready()
        self.assertTrue(self.sender.send_safe_distance(120, True))
        first_sample_seq = self.sender.last_safe_distance_seq

        write_entered = threading.Event()
        release_write = threading.Event()
        status_done = threading.Event()
        original_write = self.fake_serial.write

        def blocking_write(data):
            write_entered.set()
            self.assertTrue(release_write.wait(timeout=1.0))
            return original_write(data)

        self.fake_serial.write = blocking_write
        send_result = []
        status_result = []
        send_worker = threading.Thread(
            target=lambda: send_result.append(
                self.sender.send_safe_distance(121, True)
            )
        )

        def observe_previous_safe_decision():
            status_result.append(
                self.sender._observe_control_status(
                    status(first_sample_seq, STATUS_SAFE_DISTANCE, 1)
                )
            )
            status_done.set()

        status_worker = threading.Thread(target=observe_previous_safe_decision)
        send_worker.start()
        self.assertTrue(write_entered.wait(timeout=1.0))
        status_worker.start()
        time.sleep(0.02)

        # RX cannot close the window in the middle of a permitted complete TX.
        self.assertFalse(status_done.is_set())
        release_write.set()
        send_worker.join(timeout=1.0)
        status_worker.join(timeout=1.0)

        self.assertEqual(send_result, [True])
        self.assertEqual(status_result, [True])
        self.assertFalse(self.sender.measure_window_open)
        writes_before = list(self.fake_serial.writes)
        self.assertFalse(self.sender.send_safe_distance(122, True))
        self.assertEqual(self.fake_serial.writes, writes_before)

    def test_delayed_safe_response_cannot_unlock_a_new_workflow(self):
        first_workflow_seq = self._accept_and_measure_ready()
        self.assertTrue(self.sender.send_safe_distance(120, True))
        old_sample_seq = self.fake_serial.writes[-1][4]

        # A new START clears the previous measurement sample window.
        second_workflow_seq = self._accept_and_measure_ready()
        self.assertNotEqual(first_workflow_seq, second_workflow_seq)
        self.assertFalse(
            self.sender._observe_control_status(
                status(second_workflow_seq, STATUS_WORKFLOW, WORKFLOW_DISTANCE_SAFE_LATCHED)
            )
        )
        self.assertFalse(
            self.sender._observe_control_status(
                status(old_sample_seq, STATUS_SAFE_DISTANCE, 1)
            )
        )

        self.assertTrue(self.sender.send_safe_distance(125, True))
        current_sample_seq = self.fake_serial.writes[-1][4]
        self.assertTrue(
            self.sender._observe_control_status(
                status(current_sample_seq, STATUS_SAFE_DISTANCE, 1)
            )
        )
        self.assertFalse(self.sender.measure_window_open)

        # Workflow latch itself must still carry this START session's SEQ.
        self.assertFalse(
            self.sender._observe_control_status(
                status(first_workflow_seq, STATUS_WORKFLOW, WORKFLOW_DISTANCE_SAFE_LATCHED)
            )
        )
        self.assertTrue(
            self.sender._observe_control_status(
                status(second_workflow_seq, STATUS_WORKFLOW, WORKFLOW_DISTANCE_SAFE_LATCHED)
            )
        )
        self.assertFalse(
            self.sender._observe_control_status(
                status(second_workflow_seq, STATUS_WORKFLOW, WORKFLOW_MEASURE_POSITION_READY)
            )
        )
        self.assertFalse(self.sender.measure_window_open)

    def test_safe_response_to_invalid_sample_cannot_unlock_workflow(self):
        workflow_seq = self._accept_and_measure_ready()
        self.assertTrue(self.sender.send_safe_distance(0, False))
        invalid_sample_seq = self.fake_serial.writes[-1][4]
        self.assertEqual(self.sender.last_safe_distance_seq, invalid_sample_seq)

        self.assertFalse(
            self.sender._observe_control_status(
                status(invalid_sample_seq, STATUS_SAFE_DISTANCE, 1)
            )
        )
        self.assertTrue(self.sender.measure_window_open)
        self.assertFalse(
            self.sender._observe_control_status(
                status(workflow_seq, STATUS_WORKFLOW, WORKFLOW_DISTANCE_SAFE_LATCHED)
            )
        )

        self.assertTrue(self.sender.send_safe_distance(125, True))
        valid_sample_seq = self.fake_serial.writes[-1][4]
        self.assertEqual(self.sender.last_safe_distance_seq, valid_sample_seq)
        self.assertTrue(
            self.sender._observe_control_status(
                status(valid_sample_seq, STATUS_SAFE_DISTANCE, 1)
            )
        )
        self.assertTrue(
            self.sender._observe_control_status(
                status(workflow_seq, STATUS_WORKFLOW, WORKFLOW_DISTANCE_SAFE_LATCHED)
            )
        )

if __name__ == "__main__":
    unittest.main()
