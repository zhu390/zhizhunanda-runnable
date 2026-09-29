import os
import pty
import select
import struct
import time
import tty
import unittest

from jetson_nano_vision.jetson_vision.vision_protocol import (
    CAPTURE_ACTION_GOTO_VIEW,
    CAPTURE_ACTION_HOME,
    CAPTURE_ACTION_SELECT_VIEW,
    ERROR_INVALID_PARAM,
    ERROR_INVALID_STATE,
    ERROR_NONE,
    ERROR_SAFE_DISTANCE_TOO_CLOSE,
    ERROR_SEQ_CONFLICT,
    MSG_CAPTURE_CTRL,
    MSG_HEARTBEAT,
    MSG_SAFE_DISTANCE,
    MSG_TARGET_CTRL,
    MSG_VISION_ERROR,
    MSG_WORKFLOW_CTRL,
    STATUS_ALIGN_DONE,
    STATUS_CAPTURE_HOME_READY,
    STATUS_CAPTURE_POINT_READY,
    STATUS_COMMAND_ACK,
    STATUS_HEARTBEAT_ALIVE,
    STATUS_OUTPUT,
    STATUS_SAFE_DISTANCE,
    STATUS_SELECTED_VIEW_READY,
    STATUS_TARGET_CTRL,
    STATUS_TARGET_READY,
    STATUS_VISION_STATE,
    STATUS_WORKFLOW,
    TARGET_CTRL_START,
    VISION_INVALID,
    VISION_VALID,
    WORKFLOW_ABORTED_HOLD,
    WORKFLOW_ACTION_ABORT_HOLD,
    WORKFLOW_ACTION_START_MEASURE,
    WORKFLOW_DISTANCE_SAFE_LATCHED,
    WORKFLOW_MEASURE_POSITION_READY,
    WORKFLOW_RETREAT_DONE_WAIT_RESTART,
    WORKFLOW_START_ACCEPTED,
    build_unified_frame,
    parse_unified_frame,
)

from ra6m5_sim_check import (
    ERROR_SAFE_DISTANCE_TIMEOUT,
    WORKFLOW_ACTION_START_MEASURE_CONTINUOUS,
    Ra6m5ProtocolCore,
    Ra6m5SerialSimulator,
    frame,
    split_frames,
    status_payloads,
)


class ManualClock:
    def __init__(self, value=100.0):
        self.value = float(value)

    def __call__(self):
        return self.value


class ContinuousSafeDistanceCoreTest(unittest.TestCase):
    def test_production_default_rejects_proposed_continuous_action(self):
        core = Ra6m5ProtocolCore()
        self.assertEqual(core.cache_size, 8)
        replies = core.process(
            frame(
                MSG_WORKFLOW_CTRL,
                0x0F,
                bytes([WORKFLOW_ACTION_START_MEASURE_CONTINUOUS]),
            )
        )
        self.assertEqual(
            status_payloads(replies),
            [(STATUS_COMMAND_ACK, 0x00, ERROR_INVALID_PARAM)],
        )
        self.assertFalse(core.workflow_active)

    def _start(self, core, *, continuous, seq=0x10):
        action = (
            WORKFLOW_ACTION_START_MEASURE_CONTINUOUS
            if continuous
            else WORKFLOW_ACTION_START_MEASURE
        )
        replies = core.process(frame(MSG_WORKFLOW_CTRL, seq, bytes([action])))
        self.assertEqual(
            status_payloads(replies),
            [
                (STATUS_COMMAND_ACK, 0x01, ERROR_NONE),
                (STATUS_WORKFLOW, WORKFLOW_START_ACCEPTED, ERROR_NONE),
                (STATUS_WORKFLOW, WORKFLOW_MEASURE_POSITION_READY, ERROR_NONE),
            ],
        )
        return seq

    def _latch(self, core, *, first_seq=0x11, distance_mm=120):
        for seq in range(first_seq, first_seq + core.safe_samples_required - 1):
            self.assertEqual(
                core.process(
                    frame(
                        MSG_SAFE_DISTANCE,
                        seq,
                        struct.pack("<HB", distance_mm, VISION_VALID),
                    )
                ),
                [],
            )
        final_seq = first_seq + core.safe_samples_required - 1
        replies = core.process(
            frame(
                MSG_SAFE_DISTANCE,
                final_seq,
                struct.pack("<HB", distance_mm, VISION_VALID),
            )
        )
        self.assertEqual(
            status_payloads(replies),
            [
                (STATUS_SAFE_DISTANCE, 0x01, ERROR_NONE),
                (STATUS_WORKFLOW, WORKFLOW_DISTANCE_SAFE_LATCHED, ERROR_NONE),
            ],
        )
        self.assertTrue(core.safe_distance_latched)
        return final_seq

    def test_legacy_start_keeps_one_shot_post_latch_rejection(self):
        clock = ManualClock()
        core = Ra6m5ProtocolCore(clock=clock)
        self._start(core, continuous=False)
        self._latch(core)

        self.assertEqual(core.service_watchdogs(now=clock.value + 10.0), [])
        replies = core.process(
            frame(MSG_SAFE_DISTANCE, 0x20, struct.pack("<HB", 125, VISION_VALID))
        )
        parsed = parse_unified_frame(replies[0])
        self.assertEqual(parsed.payload, bytes([ERROR_INVALID_STATE]))
        self.assertTrue(core.safe_distance_latched)

    def test_production_initial_samples_have_no_future_600ms_gate(self):
        clock = ManualClock()
        core = Ra6m5ProtocolCore(clock=clock)
        workflow_seq = self._start(core, continuous=False)

        for seq in (0x11, 0x12):
            self.assertEqual(
                core.process(
                    frame(MSG_SAFE_DISTANCE, seq, struct.pack("<HB", 120, VISION_VALID))
                ),
                [],
            )
            clock.value += 1.0

        replies = core.process(
            frame(MSG_SAFE_DISTANCE, 0x13, struct.pack("<HB", 120, VISION_VALID))
        )
        self.assertEqual(
            status_payloads(replies),
            [
                (STATUS_SAFE_DISTANCE, 0x01, ERROR_NONE),
                (STATUS_WORKFLOW, WORKFLOW_DISTANCE_SAFE_LATCHED, ERROR_NONE),
            ],
        )
        self.assertEqual(parse_unified_frame(replies[1]).seq, workflow_seq)
        self.assertEqual(len(core._safe_sample_cache), 1)

    def test_continuous_refresh_extends_600ms_deadline(self):
        clock = ManualClock()
        core = Ra6m5ProtocolCore(
            enable_continuous_extension=True,
            clock=clock,
        )
        workflow_seq = self._start(core, continuous=True)
        self._latch(core)

        clock.value += 0.500
        refresh_seq = 0x20
        self.assertEqual(
            core.process(
                frame(MSG_SAFE_DISTANCE, refresh_seq, struct.pack("<HB", 125, VISION_VALID))
            ),
            [],
        )
        self.assertEqual(core.last_valid_safe_time, clock.value)
        self.assertEqual(core.last_valid_safe_seq, refresh_seq)
        self.assertEqual(core.service_watchdogs(now=clock.value + 0.599), [])

        replies = core.service_watchdogs(now=clock.value + 0.601)
        self.assertEqual(
            status_payloads(replies),
            [
                (STATUS_SAFE_DISTANCE, 0x00, ERROR_SAFE_DISTANCE_TIMEOUT),
                (STATUS_WORKFLOW, WORKFLOW_RETREAT_DONE_WAIT_RESTART, ERROR_NONE),
            ],
        )
        self.assertEqual(parse_unified_frame(replies[0]).seq, refresh_seq)
        self.assertEqual(parse_unified_frame(replies[1]).seq, workflow_seq)
        self.assertFalse(core.workflow_active)
        self.assertFalse(core.safe_distance_latched)

    def test_initial_safe_streak_resets_when_samples_are_not_fresh(self):
        clock = ManualClock()
        core = Ra6m5ProtocolCore(
            enable_continuous_extension=True,
            clock=clock,
        )
        self._start(core, continuous=True)

        for seq in (0x11, 0x12):
            self.assertEqual(
                core.process(
                    frame(MSG_SAFE_DISTANCE, seq, struct.pack("<HB", 120, VISION_VALID))
                ),
                [],
            )
        self.assertEqual(core.safe_streak, 2)

        clock.value += core.safe_freshness_timeout_s + 0.001
        self.assertEqual(
            core.process(
                frame(MSG_SAFE_DISTANCE, 0x13, struct.pack("<HB", 120, VISION_VALID))
            ),
            [],
        )
        self.assertEqual(core.safe_streak, 1)
        self.assertFalse(core.safe_distance_latched)

    def test_heartbeat_and_invalid_sample_do_not_refresh_distance(self):
        clock = ManualClock()
        core = Ra6m5ProtocolCore(
            enable_continuous_extension=True,
            clock=clock,
        )
        self._start(core, continuous=True)
        latched_seq = self._latch(core)
        latched_at = core.last_valid_safe_time

        clock.value += 0.300
        self.assertEqual(
            status_payloads(
                core.process(frame(MSG_HEARTBEAT, 0x30, struct.pack("<I", 1234)))
            ),
            [(STATUS_HEARTBEAT_ALIVE, 0x01, ERROR_NONE)],
        )
        clock.value += 0.200
        self.assertEqual(
            core.process(
                frame(MSG_SAFE_DISTANCE, 0x31, struct.pack("<HB", 0, VISION_INVALID))
            ),
            [],
        )
        self.assertEqual(core.last_valid_safe_time, latched_at)
        self.assertEqual(core.last_valid_safe_seq, latched_seq)

        replies = core.service_watchdogs(now=latched_at + 0.601)
        self.assertEqual(
            status_payloads(replies)[0],
            (STATUS_SAFE_DISTANCE, 0x00, ERROR_SAFE_DISTANCE_TIMEOUT),
        )

    def test_replayed_sample_does_not_refresh_and_conflicting_payload_is_rejected(self):
        clock = ManualClock()
        core = Ra6m5ProtocolCore(
            enable_continuous_extension=True,
            clock=clock,
        )
        self._start(core, continuous=True)
        latched_seq = self._latch(core)
        latched_at = core.last_valid_safe_time

        clock.value += 0.400
        self.assertEqual(
            core.process(
                frame(MSG_SAFE_DISTANCE, latched_seq, struct.pack("<HB", 120, VISION_VALID))
            ),
            [],
        )
        self.assertEqual(core.last_valid_safe_time, latched_at)

        conflict = core.process(
            frame(MSG_SAFE_DISTANCE, latched_seq, struct.pack("<HB", 130, VISION_VALID))
        )
        parsed = parse_unified_frame(conflict[0])
        self.assertEqual(parsed.seq, latched_seq)
        self.assertEqual(parsed.payload, bytes([ERROR_SEQ_CONFLICT]))
        self.assertEqual(core.last_valid_safe_time, latched_at)

        replies = core.service_watchdogs(now=latched_at + 0.601)
        self.assertEqual(
            status_payloads(replies)[0],
            (STATUS_SAFE_DISTANCE, 0x00, ERROR_SAFE_DISTANCE_TIMEOUT),
        )

    def test_low_distance_after_latch_immediately_retreats_and_clears_output(self):
        clock = ManualClock()
        core = Ra6m5ProtocolCore(
            enable_continuous_extension=True,
            clock=clock,
        )
        core.process(frame(MSG_HEARTBEAT, 0x01, struct.pack("<I", 1)))
        workflow_seq = self._start(core, continuous=True)
        self._latch(core)

        capture_steps = [
            (0x20, CAPTURE_ACTION_HOME, 0, STATUS_CAPTURE_HOME_READY, 1),
            (0x21, CAPTURE_ACTION_GOTO_VIEW, 1, STATUS_CAPTURE_POINT_READY, 1),
            (0x22, CAPTURE_ACTION_GOTO_VIEW, 2, STATUS_CAPTURE_POINT_READY, 2),
            (0x23, CAPTURE_ACTION_GOTO_VIEW, 3, STATUS_CAPTURE_POINT_READY, 3),
            (0x24, CAPTURE_ACTION_HOME, 0, STATUS_CAPTURE_HOME_READY, 1),
            (0x25, CAPTURE_ACTION_SELECT_VIEW, 2, STATUS_SELECTED_VIEW_READY, 2),
        ]
        for seq, action, point_id, event, value in capture_steps:
            self.assertEqual(
                status_payloads(
                    core.process(
                        frame(MSG_CAPTURE_CTRL, seq, bytes([action, point_id]))
                    )
                ),
                [
                    (STATUS_COMMAND_ACK, 0x01, ERROR_NONE),
                    (event, value, ERROR_NONE),
                ],
            )

        target_seq = 0x30
        self.assertEqual(
            status_payloads(
                core.process(frame(MSG_TARGET_CTRL, target_seq, bytes([TARGET_CTRL_START])))
            ),
            [
                (STATUS_COMMAND_ACK, 0x01, ERROR_NONE),
                (STATUS_TARGET_CTRL, 0x01, ERROR_NONE),
                (STATUS_TARGET_READY, 0x01, ERROR_NONE),
            ],
        )
        vision_replies = []
        for seq in (0x31, 0x32, 0x33):
            vision_replies.extend(
                core.process(
                    frame(MSG_VISION_ERROR, seq, struct.pack("<hhB", 2, -3, VISION_VALID))
                )
            )
        self.assertIn(
            (STATUS_ALIGN_DONE, 0x01, ERROR_NONE),
            status_payloads(vision_replies),
        )
        self.assertEqual(
            status_payloads(core.set_p000(True)),
            [(STATUS_OUTPUT, 0x01, ERROR_NONE)],
        )
        self.assertTrue(core.output_on)

        unsafe_seq = 0x40
        replies = core.process(
            frame(MSG_SAFE_DISTANCE, unsafe_seq, struct.pack("<HB", 90, VISION_VALID))
        )
        self.assertEqual(
            status_payloads(replies),
            [
                (STATUS_SAFE_DISTANCE, 0x00, ERROR_SAFE_DISTANCE_TOO_CLOSE),
                (STATUS_WORKFLOW, WORKFLOW_RETREAT_DONE_WAIT_RESTART, ERROR_NONE),
            ],
        )
        self.assertEqual(parse_unified_frame(replies[0]).seq, unsafe_seq)
        self.assertEqual(parse_unified_frame(replies[1]).seq, workflow_seq)
        self.assertFalse(core.output_on)
        self.assertFalse(core.p000_pressed)
        self.assertFalse(core.target_enabled)
        self.assertIsNone(core.selected_view)
        self.assertFalse(core.safe_distance_latched)
        self.assertEqual(core.workflow_state, WORKFLOW_RETREAT_DONE_WAIT_RESTART)

    def test_abort_cancels_continuous_watchdog(self):
        clock = ManualClock()
        core = Ra6m5ProtocolCore(
            enable_continuous_extension=True,
            clock=clock,
        )
        self._start(core, continuous=True)
        self._latch(core)

        replies = core.process(
            frame(MSG_WORKFLOW_CTRL, 0x50, bytes([WORKFLOW_ACTION_ABORT_HOLD]))
        )
        self.assertEqual(
            status_payloads(replies),
            [
                (STATUS_COMMAND_ACK, 0x01, ERROR_NONE),
                (STATUS_WORKFLOW, WORKFLOW_ABORTED_HOLD, ERROR_NONE),
            ],
        )
        self.assertEqual(core.service_watchdogs(now=clock.value + 10.0), [])

    def test_pty_simulator_emits_timeout_without_new_serial_input(self):
        master_fd, slave_fd = pty.openpty()
        tty.setraw(slave_fd)
        os.set_blocking(slave_fd, False)
        core = Ra6m5ProtocolCore(
            safe_samples_required=1,
            safe_freshness_timeout_s=0.050,
            enable_continuous_extension=True,
        )
        simulator = Ra6m5SerialSimulator(master_fd, core)
        simulator.start()
        received = bytearray()
        frames = []
        try:
            os.write(
                slave_fd,
                build_unified_frame(
                    MSG_WORKFLOW_CTRL,
                    0x60,
                    bytes([WORKFLOW_ACTION_START_MEASURE_CONTINUOUS]),
                ),
            )
            os.write(
                slave_fd,
                build_unified_frame(
                    MSG_SAFE_DISTANCE,
                    0x61,
                    struct.pack("<HB", 120, VISION_VALID),
                ),
            )

            deadline = time.monotonic() + 1.0
            while time.monotonic() < deadline and len(frames) < 7:
                ready, _, _ = select.select([slave_fd], [], [], 0.05)
                if not ready:
                    continue
                received.extend(os.read(slave_fd, 4096))
                frames.extend(split_frames(received))

            self.assertGreaterEqual(len(frames), 7)
            self.assertEqual(
                status_payloads(frames[-2:]),
                [
                    (STATUS_SAFE_DISTANCE, 0x00, ERROR_SAFE_DISTANCE_TIMEOUT),
                    (STATUS_WORKFLOW, WORKFLOW_RETREAT_DONE_WAIT_RESTART, ERROR_NONE),
                ],
            )
            self.assertFalse(core.safe_distance_latched)
        finally:
            simulator.stop()
            simulator.join(timeout=0.5)
            os.close(slave_fd)
            os.close(master_fd)


if __name__ == "__main__":
    unittest.main()
