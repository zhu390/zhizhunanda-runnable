#!/usr/bin/env python3
"""RA6M5 protocol-v5 simulator and hardware-free smoke test.

This file models the formal ``A5 5A`` competition flow described by
``视觉协议(5).md``.  Running it performs two checks:

1. a deterministic state-machine check, including duplicate/conflicting
   control frames; and
2. a pseudo-terminal end-to-end check against ``SerialSender``.

No RA6M5, camera, robot arm, or laser hardware is accessed.
"""

from __future__ import annotations

import argparse
from collections import OrderedDict
import os
import pty
import select
import struct
import subprocess
import sys
import threading
import time
import tty
from typing import Callable, Dict, Iterable, List, Optional, Sequence, Tuple


ROOT = os.path.abspath(os.path.dirname(__file__))
VISION_ROOT = os.path.join(ROOT, "jetson_nano_vision")
if VISION_ROOT not in sys.path:
    sys.path.insert(0, VISION_ROOT)

from jetson_vision.serial_sender import SerialSender
from jetson_vision.vision_protocol import (
    CAPTURE_ACTION_GOTO_VIEW,
    CAPTURE_ACTION_HOME,
    CAPTURE_ACTION_SELECT_VIEW,
    ERROR_BUSY,
    ERROR_INVALID_PARAM,
    ERROR_INVALID_STATE,
    ERROR_NONE,
    ERROR_SAFE_DISTANCE_TOO_CLOSE,
    ERROR_SAFE_DISTANCE_TIMEOUT,
    ERROR_SEQ_CONFLICT,
    ERROR_TARGET_GATE_DENIED,
    ERROR_UNKNOWN_TYPE,
    ERROR_VISION_LOST,
    MSG_CAPTURE_CTRL,
    MSG_ERROR,
    MSG_HEARTBEAT,
    MSG_SAFE_DISTANCE,
    MSG_STATUS,
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
    TARGET_CTRL_STOP_HOLD,
    UNIFIED_MAX_PAYLOAD,
    UnifiedFrame,
    VISION_INVALID,
    VISION_VALID,
    WORKFLOW_ABORTED_HOLD,
    WORKFLOW_ACTION_ABORT_HOLD,
    WORKFLOW_ACTION_FINISH_RETURN_HOME,
    WORKFLOW_ACTION_START_MEASURE,
    WORKFLOW_ACTION_START_MEASURE_CONTINUOUS,
    WORKFLOW_DISTANCE_SAFE_LATCHED,
    WORKFLOW_IDLE,
    WORKFLOW_MEASURE_POSITION_READY,
    WORKFLOW_RETREAT_DONE_WAIT_RESTART,
    WORKFLOW_RETURN_HOME_DONE,
    WORKFLOW_START_ACCEPTED,
    build_unified_frame,
    parse_unified_frame,
)


CONTROL_TYPES = (MSG_WORKFLOW_CTRL, MSG_CAPTURE_CTRL, MSG_TARGET_CTRL)
ControlKey = Tuple[int, int]
StatusTuple = Tuple[int, int, int]

# The current RA6M5 production contract only defines workflow actions 0x01,
# 0x02 and 0x03.  The continuous 0x04/timeout 0x10 model remains available
# behind an explicit constructor switch for protocol-design tests, but the
# default simulator must match the real firmware and reject 0x04.
DEFAULT_SAFE_FRESHNESS_TIMEOUT_S = 0.600


class SmokeCheckError(AssertionError):
    """Raised when the simulator or transport violates the formal protocol."""


def require(condition: bool, message: str) -> None:
    if not condition:
        raise SmokeCheckError(message)


def hex_bytes(data: bytes) -> str:
    return " ".join("{:02X}".format(byte) for byte in data)


def make_status(seq: int, event: int, value: int, error: int = ERROR_NONE) -> bytes:
    return build_unified_frame(
        MSG_STATUS,
        seq,
        bytes([event & 0xFF, value & 0xFF, error & 0xFF]),
    )


def make_error(seq: int, error: int) -> bytes:
    return build_unified_frame(MSG_ERROR, seq, bytes([error & 0xFF]))


def status_payloads(replies: Iterable[bytes]) -> List[StatusTuple]:
    result: List[StatusTuple] = []
    for raw in replies:
        frame = parse_unified_frame(raw)
        require(frame.msg_type == MSG_STATUS, "expected STATUS, got TYPE=0x{:02X}".format(frame.msg_type))
        require(len(frame.payload) == 3, "STATUS payload must contain event/value/error")
        result.append((frame.payload[0], frame.payload[1], frame.payload[2]))
    return result


def expect_statuses(
    label: str,
    replies: Sequence[bytes],
    expected: Sequence[StatusTuple],
) -> None:
    actual = status_payloads(replies)
    require(
        actual == list(expected),
        "{}: status order mismatch; got={!r}, expected={!r}".format(label, actual, list(expected)),
    )


def expect_error(label: str, replies: Sequence[bytes], seq: int, error: int) -> None:
    require(len(replies) == 1, "{}: expected exactly one ERROR frame".format(label))
    frame = parse_unified_frame(replies[0])
    require(frame.msg_type == MSG_ERROR, "{}: expected TYPE=ERROR".format(label))
    require(frame.seq == seq, "{}: ERROR SEQ must match request".format(label))
    require(frame.payload == bytes([error]), "{}: wrong error code".format(label))


class Ra6m5ProtocolCore:
    """Deterministic formal-flow state machine used by the serial simulator.

    Motion and ``soft_reset`` complete immediately in this hardware-free model,
    but reply ordering remains realistic: every accepted control first returns
    ``COMMAND_ACK`` with the original SEQ, then its business status(es).
    """

    def __init__(
        self,
        safe_threshold_mm: int = 110,
        safe_samples_required: int = 3,
        safe_freshness_timeout_s: float = DEFAULT_SAFE_FRESHNESS_TIMEOUT_S,
        align_tolerance_px: int = 8,
        align_samples_required: int = 3,
        # Match the current RA6M5 firmware.  A 64-entry control cache belongs
        # to the older/future recommendation, not the Chapter 19 contract.
        cache_size: int = 8,
        enable_continuous_extension: bool = False,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        require(safe_samples_required > 0, "safe_samples_required must be positive")
        require(safe_freshness_timeout_s > 0.0, "safe_freshness_timeout_s must be positive")
        require(align_samples_required > 0, "align_samples_required must be positive")
        self.safe_threshold_mm = int(safe_threshold_mm)
        self.safe_samples_required = int(safe_samples_required)
        self.safe_freshness_timeout_s = float(safe_freshness_timeout_s)
        self.align_tolerance_px = int(align_tolerance_px)
        self.align_samples_required = int(align_samples_required)
        self.cache_size = max(1, int(cache_size))
        self.enable_continuous_extension = bool(enable_continuous_extension)
        self._clock = clock
        self._lock = threading.RLock()
        self._control_cache: "OrderedDict[ControlKey, Tuple[bytes, Tuple[bytes, ...]]]" = OrderedDict()
        self.control_dispatch_count: Dict[ControlKey, int] = {}
        self.duplicate_controls = 0
        self.sequence_conflicts = 0
        self.heartbeat_alive = False
        self._clear_session(WORKFLOW_IDLE)

    def _clear_session(self, workflow_state: int) -> None:
        self.workflow_state = workflow_state
        self.workflow_session_seq: Optional[int] = None
        self.workflow_active = False
        self.measurement_window = False
        self.continuous_safe_distance = False
        self.safe_distance_latched = False
        self.safe_streak = 0
        self.last_valid_safe_time: Optional[float] = None
        self.last_valid_safe_seq: Optional[int] = None
        self._safe_sample_cache: "OrderedDict[int, bytes]" = OrderedDict()
        self.capture_stage = 0
        self.selected_view: Optional[int] = None
        self.target_enabled = False
        self.target_ready = False
        self.target_session_seq: Optional[int] = None
        self.vision_valid = False
        self.align_streak = 0
        self.alignment_confirmed = False
        self.p000_pressed = False
        self.output_on = False

    @staticmethod
    def _ack(seq: int, accepted: bool = True, error: int = ERROR_NONE) -> bytes:
        return make_status(seq, STATUS_COMMAND_ACK, 0x01 if accepted else 0x00, error)

    def _reject_control(
        self,
        seq: int,
        error: int,
        business_event: Optional[int] = None,
        business_value: int = 0,
    ) -> List[bytes]:
        replies = [self._ack(seq, accepted=False, error=error)]
        if business_event is not None:
            replies.append(make_status(seq, business_event, business_value, error))
        return replies

    def _cache_control(
        self,
        key: ControlKey,
        payload: bytes,
        replies: Sequence[bytes],
    ) -> None:
        self._control_cache[key] = (bytes(payload), tuple(bytes(reply) for reply in replies))
        self._control_cache.move_to_end(key)
        while len(self._control_cache) > self.cache_size:
            self._control_cache.popitem(last=False)

    def process(self, frame: UnifiedFrame) -> List[bytes]:
        """Consume one CRC-validated frame and return zero or more reply frames."""
        with self._lock:
            if frame.msg_type in CONTROL_TYPES:
                return self._process_control(frame)
            if frame.msg_type == MSG_HEARTBEAT:
                return self._process_heartbeat(frame)
            if frame.msg_type == MSG_SAFE_DISTANCE:
                return self._process_safe_distance(frame)
            if frame.msg_type == MSG_VISION_ERROR:
                return self._process_vision(frame)
            return [make_error(frame.seq, ERROR_UNKNOWN_TYPE)]

    def _process_control(self, frame: UnifiedFrame) -> List[bytes]:
        key = (frame.msg_type, frame.seq)
        cached = self._control_cache.get(key)
        if cached is not None:
            old_payload, old_replies = cached
            self._control_cache.move_to_end(key)
            if old_payload == frame.payload:
                self.duplicate_controls += 1
                return list(old_replies)
            self.sequence_conflicts += 1
            return self._reject_control(frame.seq, ERROR_SEQ_CONFLICT)

        self.control_dispatch_count[key] = self.control_dispatch_count.get(key, 0) + 1
        if frame.msg_type == MSG_WORKFLOW_CTRL:
            replies = self._process_workflow_control(frame.seq, frame.payload)
        elif frame.msg_type == MSG_CAPTURE_CTRL:
            replies = self._process_capture_control(frame.seq, frame.payload)
        else:
            replies = self._process_target_control(frame.seq, frame.payload)
        self._cache_control(key, frame.payload, replies)
        return replies

    def _process_heartbeat(self, frame: UnifiedFrame) -> List[bytes]:
        if len(frame.payload) != 4:
            return [make_error(frame.seq, ERROR_INVALID_PARAM)]
        self.heartbeat_alive = True
        return [make_status(frame.seq, STATUS_HEARTBEAT_ALIVE, 0x01)]

    def _process_workflow_control(self, seq: int, payload: bytes) -> List[bytes]:
        if len(payload) != 1:
            return self._reject_control(seq, ERROR_INVALID_PARAM)
        action = payload[0]
        if (
            action == WORKFLOW_ACTION_START_MEASURE_CONTINUOUS
            and not self.enable_continuous_extension
        ):
            return self._reject_control(seq, ERROR_INVALID_PARAM)
        if action in (
            WORKFLOW_ACTION_START_MEASURE,
            WORKFLOW_ACTION_START_MEASURE_CONTINUOUS,
        ):
            self._clear_session(WORKFLOW_START_ACCEPTED)
            self.workflow_active = True
            self.workflow_session_seq = seq
            self.continuous_safe_distance = (
                action == WORKFLOW_ACTION_START_MEASURE_CONTINUOUS
            )
            replies = [
                self._ack(seq),
                make_status(seq, STATUS_WORKFLOW, WORKFLOW_START_ACCEPTED),
            ]
            # The simulator has no motors, so soft_reset and movement finish now.
            self.workflow_state = WORKFLOW_MEASURE_POSITION_READY
            self.measurement_window = True
            replies.append(make_status(seq, STATUS_WORKFLOW, WORKFLOW_MEASURE_POSITION_READY))
            return replies

        if action == WORKFLOW_ACTION_FINISH_RETURN_HOME:
            if not self.workflow_active:
                return self._reject_control(seq, ERROR_INVALID_STATE)
            replies = [self._ack(seq)]
            self._clear_session(WORKFLOW_RETURN_HOME_DONE)
            replies.append(make_status(seq, STATUS_WORKFLOW, WORKFLOW_RETURN_HOME_DONE))
            return replies

        if action == WORKFLOW_ACTION_ABORT_HOLD:
            replies = [self._ack(seq)]
            self._clear_session(WORKFLOW_ABORTED_HOLD)
            replies.append(make_status(seq, STATUS_WORKFLOW, WORKFLOW_ABORTED_HOLD))
            return replies

        return self._reject_control(seq, ERROR_INVALID_PARAM)

    def _process_safe_distance(self, frame: UnifiedFrame) -> List[bytes]:
        if len(frame.payload) != 3 or frame.payload[2] not in (VISION_INVALID, VISION_VALID):
            return [make_error(frame.seq, ERROR_INVALID_PARAM)]
        monitoring_after_latch = bool(
            self.continuous_safe_distance and self.safe_distance_latched
        )
        if (
            not self.workflow_active
            or (not self.measurement_window and not monitoring_after_latch)
        ):
            return [make_error(frame.seq, ERROR_INVALID_STATE)]

        distance_mm, valid = struct.unpack("<HB", frame.payload)
        cached_payload = self._safe_sample_cache.get(frame.seq)
        if cached_payload is not None:
            self._safe_sample_cache.move_to_end(frame.seq)
            if cached_payload == frame.payload:
                # Samples are never retransmitted by the formal sender.  Treat
                # an exact replay as stale input and, critically, do not let it
                # refresh the continuous safety watchdog.
                return []
            return [make_error(frame.seq, ERROR_SEQ_CONFLICT)]
        self._safe_sample_cache[frame.seq] = bytes(frame.payload)
        # Current firmware remembers only the most recent SAFE SEQ.  The wider
        # replay window belongs to the proposed continuous extension.
        safe_cache_limit = 32 if self.enable_continuous_extension else 1
        while len(self._safe_sample_cache) > safe_cache_limit:
            self._safe_sample_cache.popitem(last=False)

        if valid == VISION_INVALID:
            if self.measurement_window:
                self.safe_streak = 0
            # An invalid sample is never fresh.  Before the initial latch it
            # merely keeps the arm waiting at the measurement pose; after a
            # continuous latch, silence/invalid data lets the independent
            # freshness watchdog fail closed.
            return []
        if distance_mm < self.safe_threshold_mm:
            return self._enter_distance_retreat(
                frame.seq,
                ERROR_SAFE_DISTANCE_TOO_CLOSE,
            )

        sample_time = float(self._clock())
        if (
            self.enable_continuous_extension
            and not monitoring_after_latch
            and self.last_valid_safe_time is not None
            and sample_time - self.last_valid_safe_time
            >= self.safe_freshness_timeout_s
        ):
            # Initial permission also requires genuinely consecutive fresh
            # samples; widely separated observations must not accumulate into
            # a latch merely because every individual value was above the
            # threshold.
            self.safe_streak = 0
        self.last_valid_safe_time = sample_time
        self.last_valid_safe_seq = int(frame.seq) & 0xFF
        if monitoring_after_latch:
            # The first SAFE response/latch is a business transition.  Later
            # safe samples only refresh the watchdog and remain response-free.
            return []

        self.safe_streak += 1
        if self.safe_streak < self.safe_samples_required:
            return []
        self.safe_distance_latched = True
        self.measurement_window = False
        self.workflow_state = WORKFLOW_DISTANCE_SAFE_LATCHED
        self.capture_stage = 0
        workflow_seq = self.workflow_session_seq
        require(workflow_seq is not None, "measurement window has no workflow SEQ")
        return [
            make_status(frame.seq, STATUS_SAFE_DISTANCE, 0x01),
            make_status(workflow_seq, STATUS_WORKFLOW, WORKFLOW_DISTANCE_SAFE_LATCHED),
        ]

    def _enter_distance_retreat(self, sample_seq: int, error: int) -> List[bytes]:
        """Fail closed and model the validated retreat as an immediate action."""

        workflow_seq = self.workflow_session_seq
        require(workflow_seq is not None, "active safety monitor has no workflow SEQ")
        self.safe_streak = 0
        self.measurement_window = False
        self.safe_distance_latched = False
        self.last_valid_safe_time = None
        self.last_valid_safe_seq = None
        self._safe_sample_cache.clear()
        self.capture_stage = 0
        self.selected_view = None
        self.target_enabled = False
        self.target_ready = False
        self.target_session_seq = None
        self.vision_valid = False
        self.align_streak = 0
        self.alignment_confirmed = False
        self.p000_pressed = False
        self.output_on = False
        self.workflow_active = False
        self.continuous_safe_distance = False
        self.workflow_state = WORKFLOW_RETREAT_DONE_WAIT_RESTART
        return [
            make_status(sample_seq, STATUS_SAFE_DISTANCE, 0x00, error),
            make_status(
                workflow_seq,
                STATUS_WORKFLOW,
                WORKFLOW_RETREAT_DONE_WAIT_RESTART,
            ),
        ]

    def service_watchdogs(self, now: Optional[float] = None) -> List[bytes]:
        """Return asynchronous fail-safe statuses due at ``now``.

        The timer exists only for an explicitly negotiated continuous session
        and starts with the valid sample that created the initial latch.
        HEARTBEAT and invalid SAFE_DISTANCE frames never update it.
        """

        with self._lock:
            if (
                not self.workflow_active
                or not self.continuous_safe_distance
                or not self.safe_distance_latched
                or self.last_valid_safe_time is None
                or self.last_valid_safe_seq is None
            ):
                return []
            timestamp = float(self._clock() if now is None else now)
            if timestamp - self.last_valid_safe_time < self.safe_freshness_timeout_s:
                return []
            return self._enter_distance_retreat(
                self.last_valid_safe_seq,
                ERROR_SAFE_DISTANCE_TIMEOUT,
            )

    def _process_capture_control(self, seq: int, payload: bytes) -> List[bytes]:
        if len(payload) != 2:
            return self._reject_control(seq, ERROR_INVALID_PARAM)
        action, point_id = payload
        if action not in (CAPTURE_ACTION_GOTO_VIEW, CAPTURE_ACTION_HOME, CAPTURE_ACTION_SELECT_VIEW):
            return self._reject_control(seq, ERROR_INVALID_PARAM)
        if action == CAPTURE_ACTION_HOME and point_id != 0:
            return self._reject_control(seq, ERROR_INVALID_PARAM)
        if action in (CAPTURE_ACTION_GOTO_VIEW, CAPTURE_ACTION_SELECT_VIEW) and point_id not in (1, 2, 3):
            return self._reject_control(seq, ERROR_INVALID_PARAM)
        if not self.workflow_active or not self.safe_distance_latched or self.target_enabled:
            return self._reject_control(seq, ERROR_INVALID_STATE)

        if action == CAPTURE_ACTION_HOME:
            if self.capture_stage == 0:
                self.capture_stage = 1  # initial HOME; next command must be GOTO 1
            elif self.capture_stage == 4:
                self.capture_stage = 5  # final HOME; SELECT is now permitted
            else:
                return self._reject_control(seq, ERROR_INVALID_STATE)
            return [
                self._ack(seq),
                make_status(seq, STATUS_CAPTURE_HOME_READY, 0x01),
            ]

        if action == CAPTURE_ACTION_GOTO_VIEW:
            expected_point = self.capture_stage if self.capture_stage in (1, 2, 3) else None
            if point_id != expected_point:
                return self._reject_control(seq, ERROR_INVALID_STATE)
            self.capture_stage += 1
            return [
                self._ack(seq),
                make_status(seq, STATUS_CAPTURE_POINT_READY, point_id),
            ]

        if self.capture_stage != 5:
            return self._reject_control(seq, ERROR_INVALID_STATE)
        self.selected_view = point_id
        self.capture_stage = 6
        return [
            self._ack(seq),
            make_status(seq, STATUS_SELECTED_VIEW_READY, point_id),
        ]

    def _target_start_gate_open(self) -> bool:
        return bool(
            self.heartbeat_alive
            and self.workflow_active
            and self.safe_distance_latched
            and self.selected_view in (1, 2, 3)
            and not self.measurement_window
            and not self.target_enabled
        )

    def _process_target_control(self, seq: int, payload: bytes) -> List[bytes]:
        if len(payload) != 1 or payload[0] not in (TARGET_CTRL_STOP_HOLD, TARGET_CTRL_START):
            return self._reject_control(seq, ERROR_INVALID_PARAM)
        enable = payload[0]
        if enable == TARGET_CTRL_START:
            if self.target_enabled:
                return self._reject_control(seq, ERROR_BUSY, STATUS_TARGET_CTRL, 0x00)
            if not self._target_start_gate_open():
                return self._reject_control(
                    seq,
                    ERROR_TARGET_GATE_DENIED,
                    STATUS_TARGET_CTRL,
                    0x00,
                )
            self.target_enabled = True
            self.target_ready = True
            self.target_session_seq = seq
            self.vision_valid = False
            self.align_streak = 0
            self.alignment_confirmed = False
            self.output_on = False
            return [
                self._ack(seq),
                make_status(seq, STATUS_TARGET_CTRL, 0x01),
                make_status(seq, STATUS_TARGET_READY, 0x01),
            ]

        # STOP_HOLD is safe and idempotent even if the target session is off.
        self.target_enabled = False
        self.target_ready = False
        self.vision_valid = False
        self.align_streak = 0
        self.alignment_confirmed = False
        self.output_on = False
        self.p000_pressed = False
        self.target_session_seq = None
        return [
            self._ack(seq),
            make_status(seq, STATUS_TARGET_CTRL, 0x00),
        ]

    def _process_vision(self, frame: UnifiedFrame) -> List[bytes]:
        if len(frame.payload) != 5 or frame.payload[4] not in (VISION_INVALID, VISION_VALID):
            return [make_error(frame.seq, ERROR_INVALID_PARAM)]
        if not self.target_enabled or not self.target_ready or self.target_session_seq is None:
            return [make_error(frame.seq, ERROR_INVALID_STATE)]

        dcx, dcy, valid = struct.unpack("<hhB", frame.payload)
        session_seq = self.target_session_seq
        replies: List[bytes] = []
        if valid == VISION_INVALID:
            if self.output_on:
                self.output_on = False
                replies.append(
                    make_status(session_seq, STATUS_OUTPUT, 0x00, ERROR_VISION_LOST)
                )
            self.vision_valid = False
            self.align_streak = 0
            self.alignment_confirmed = False
            replies.append(
                make_status(session_seq, STATUS_VISION_STATE, 0x00, ERROR_VISION_LOST)
            )
            return replies

        if not self.vision_valid:
            replies.append(make_status(session_seq, STATUS_VISION_STATE, 0x01))
        self.vision_valid = True
        in_tolerance = abs(dcx) <= self.align_tolerance_px and abs(dcy) <= self.align_tolerance_px
        if not in_tolerance:
            self.align_streak = 0
            self.alignment_confirmed = False
            if self.output_on:
                self.output_on = False
                replies.append(make_status(session_seq, STATUS_OUTPUT, 0x00))
            return replies

        self.align_streak += 1
        if self.align_streak >= self.align_samples_required and not self.alignment_confirmed:
            self.alignment_confirmed = True
            replies.append(make_status(session_seq, STATUS_ALIGN_DONE, 0x01))
            if self.p000_pressed and not self.output_on:
                self.output_on = True
                replies.append(make_status(session_seq, STATUS_OUTPUT, 0x01))
        return replies

    def set_p000(self, pressed: bool) -> List[bytes]:
        """Simulate the physical hold-to-enable input and return async statuses."""
        with self._lock:
            pressed = bool(pressed)
            self.p000_pressed = pressed
            if self.target_session_seq is None:
                return []
            if not pressed and self.output_on:
                self.output_on = False
                return [make_status(self.target_session_seq, STATUS_OUTPUT, 0x00)]
            if (
                pressed
                and self.target_enabled
                and self.target_ready
                and self.vision_valid
                and self.alignment_confirmed
                and self.safe_distance_latched
                and not self.output_on
            ):
                self.output_on = True
                return [make_status(self.target_session_seq, STATUS_OUTPUT, 0x01)]
            return []


def frame(msg_type: int, seq: int, payload: bytes) -> UnifiedFrame:
    return parse_unified_frame(build_unified_frame(msg_type, seq, payload))


def run_core_self_check() -> None:
    core = Ra6m5ProtocolCore()

    expect_statuses(
        "heartbeat",
        core.process(frame(MSG_HEARTBEAT, 0x01, struct.pack("<I", 1234))),
        [(STATUS_HEARTBEAT_ALIVE, 0x01, ERROR_NONE)],
    )

    expect_statuses(
        "production firmware rejects proposed continuous action",
        core.process(
            frame(
                MSG_WORKFLOW_CTRL,
                0x0F,
                bytes([WORKFLOW_ACTION_START_MEASURE_CONTINUOUS]),
            )
        ),
        [(STATUS_COMMAND_ACK, 0x00, ERROR_INVALID_PARAM)],
    )

    start = frame(MSG_WORKFLOW_CTRL, 0x10, bytes([WORKFLOW_ACTION_START_MEASURE]))
    start_replies = core.process(start)
    expect_statuses(
        "workflow start",
        start_replies,
        [
            (STATUS_COMMAND_ACK, 0x01, ERROR_NONE),
            (STATUS_WORKFLOW, WORKFLOW_START_ACCEPTED, ERROR_NONE),
            (STATUS_WORKFLOW, WORKFLOW_MEASURE_POSITION_READY, ERROR_NONE),
        ],
    )
    start_duplicate = core.process(start)
    require(start_duplicate == start_replies, "identical control retry must replay identical replies")
    require(
        core.control_dispatch_count[(MSG_WORKFLOW_CTRL, 0x10)] == 1,
        "identical control retry must not execute twice",
    )

    conflict = core.process(
        frame(MSG_WORKFLOW_CTRL, 0x10, bytes([WORKFLOW_ACTION_ABORT_HOLD]))
    )
    expect_statuses(
        "same TYPE+SEQ conflict",
        conflict,
        [(STATUS_COMMAND_ACK, 0x00, ERROR_SEQ_CONFLICT)],
    )
    require(
        core.workflow_state == WORKFLOW_MEASURE_POSITION_READY,
        "SEQ conflict must not mutate workflow state",
    )

    # Invalid depth breaks consecutiveness but is not itself a workflow error.
    require(
        core.process(frame(MSG_SAFE_DISTANCE, 0x11, struct.pack("<HB", 0, VISION_INVALID))) == [],
        "invalid depth sample should keep the measurement window open",
    )
    for seq in (0x12, 0x13):
        require(
            core.process(frame(MSG_SAFE_DISTANCE, seq, struct.pack("<HB", 120, VISION_VALID))) == [],
            "safe distance must not latch before the required consecutive samples",
        )
    expect_statuses(
        "safe-distance latch",
        core.process(frame(MSG_SAFE_DISTANCE, 0x14, struct.pack("<HB", 120, VISION_VALID))),
        [
            (STATUS_SAFE_DISTANCE, 0x01, ERROR_NONE),
            (STATUS_WORKFLOW, WORKFLOW_DISTANCE_SAFE_LATCHED, ERROR_NONE),
        ],
    )
    expect_error(
        "safe sample after closed measurement window",
        core.process(frame(MSG_SAFE_DISTANCE, 0x15, struct.pack("<HB", 120, VISION_VALID))),
        0x15,
        ERROR_INVALID_STATE,
    )

    capture_steps = [
        (0x20, CAPTURE_ACTION_HOME, 0, STATUS_CAPTURE_HOME_READY, 1),
        (0x21, CAPTURE_ACTION_GOTO_VIEW, 1, STATUS_CAPTURE_POINT_READY, 1),
        (0x22, CAPTURE_ACTION_GOTO_VIEW, 2, STATUS_CAPTURE_POINT_READY, 2),
        (0x23, CAPTURE_ACTION_GOTO_VIEW, 3, STATUS_CAPTURE_POINT_READY, 3),
        (0x24, CAPTURE_ACTION_HOME, 0, STATUS_CAPTURE_HOME_READY, 1),
        (0x25, CAPTURE_ACTION_SELECT_VIEW, 2, STATUS_SELECTED_VIEW_READY, 2),
    ]
    first_goto_frame: Optional[UnifiedFrame] = None
    first_goto_replies: Optional[List[bytes]] = None
    for seq, action, point_id, event, value in capture_steps:
        request = frame(MSG_CAPTURE_CTRL, seq, bytes([action, point_id]))
        replies = core.process(request)
        expect_statuses(
            "capture action=0x{:02X} point={}".format(action, point_id),
            replies,
            [
                (STATUS_COMMAND_ACK, 0x01, ERROR_NONE),
                (event, value, ERROR_NONE),
            ],
        )
        if action == CAPTURE_ACTION_GOTO_VIEW and point_id == 1:
            first_goto_frame = request
            first_goto_replies = replies

    require(first_goto_frame is not None and first_goto_replies is not None, "missing GOTO 1 vector")
    require(
        core.process(first_goto_frame) == first_goto_replies,
        "late duplicate GOTO must replay the original result without moving again",
    )
    require(
        core.control_dispatch_count[(MSG_CAPTURE_CTRL, 0x21)] == 1,
        "duplicate GOTO must be dispatched once",
    )

    target_seq = 0x30
    expect_statuses(
        "target start",
        core.process(frame(MSG_TARGET_CTRL, target_seq, bytes([TARGET_CTRL_START]))),
        [
            (STATUS_COMMAND_ACK, 0x01, ERROR_NONE),
            (STATUS_TARGET_CTRL, 0x01, ERROR_NONE),
            (STATUS_TARGET_READY, 0x01, ERROR_NONE),
        ],
    )
    expect_statuses(
        "vision invalid",
        core.process(frame(MSG_VISION_ERROR, 0x31, struct.pack("<hhB", 0, 0, VISION_INVALID))),
        [(STATUS_VISION_STATE, 0x00, ERROR_VISION_LOST)],
    )

    valid_replies: List[bytes] = []
    for seq in (0x32, 0x33, 0x34):
        valid_replies.extend(
            core.process(frame(MSG_VISION_ERROR, seq, struct.pack("<hhB", 2, -3, VISION_VALID)))
        )
    expect_statuses(
        "vision recovery and align",
        valid_replies,
        [
            (STATUS_VISION_STATE, 0x01, ERROR_NONE),
            (STATUS_ALIGN_DONE, 0x01, ERROR_NONE),
        ],
    )
    output_on = core.set_p000(True)
    expect_statuses(
        "P000 output on",
        output_on,
        [(STATUS_OUTPUT, 0x01, ERROR_NONE)],
    )
    require(
        parse_unified_frame(output_on[0]).seq == target_seq,
        "target session status must reuse TARGET START SEQ",
    )
    expect_statuses(
        "vision loss during output",
        core.process(frame(MSG_VISION_ERROR, 0x35, struct.pack("<hhB", 0, 0, VISION_INVALID))),
        [
            (STATUS_OUTPUT, 0x00, ERROR_VISION_LOST),
            (STATUS_VISION_STATE, 0x00, ERROR_VISION_LOST),
        ],
    )
    core.set_p000(False)

    expect_statuses(
        "target stop/hold",
        core.process(frame(MSG_TARGET_CTRL, 0x36, bytes([TARGET_CTRL_STOP_HOLD]))),
        [
            (STATUS_COMMAND_ACK, 0x01, ERROR_NONE),
            (STATUS_TARGET_CTRL, 0x00, ERROR_NONE),
        ],
    )
    expect_statuses(
        "workflow finish/home",
        core.process(frame(MSG_WORKFLOW_CTRL, 0x40, bytes([WORKFLOW_ACTION_FINISH_RETURN_HOME]))),
        [
            (STATUS_COMMAND_ACK, 0x01, ERROR_NONE),
            (STATUS_WORKFLOW, WORKFLOW_RETURN_HOME_DONE, ERROR_NONE),
        ],
    )

    expect_statuses(
        "workflow restart",
        core.process(frame(MSG_WORKFLOW_CTRL, 0x41, bytes([WORKFLOW_ACTION_START_MEASURE]))),
        [
            (STATUS_COMMAND_ACK, 0x01, ERROR_NONE),
            (STATUS_WORKFLOW, WORKFLOW_START_ACCEPTED, ERROR_NONE),
            (STATUS_WORKFLOW, WORKFLOW_MEASURE_POSITION_READY, ERROR_NONE),
        ],
    )
    expect_statuses(
        "workflow abort/hold",
        core.process(frame(MSG_WORKFLOW_CTRL, 0x42, bytes([WORKFLOW_ACTION_ABORT_HOLD]))),
        [
            (STATUS_COMMAND_ACK, 0x01, ERROR_NONE),
            (STATUS_WORKFLOW, WORKFLOW_ABORTED_HOLD, ERROR_NONE),
        ],
    )

    unsafe_core = Ra6m5ProtocolCore()
    unsafe_core.process(frame(MSG_HEARTBEAT, 0x50, struct.pack("<I", 1)))
    unsafe_core.process(frame(MSG_WORKFLOW_CTRL, 0x51, bytes([WORKFLOW_ACTION_START_MEASURE])))
    expect_statuses(
        "unsafe-distance retreat",
        unsafe_core.process(
            frame(MSG_SAFE_DISTANCE, 0x52, struct.pack("<HB", 90, VISION_VALID))
        ),
        [
            (STATUS_SAFE_DISTANCE, 0x00, ERROR_SAFE_DISTANCE_TOO_CLOSE),
            (STATUS_WORKFLOW, WORKFLOW_RETREAT_DONE_WAIT_RESTART, ERROR_NONE),
        ],
    )

    denied_core = Ra6m5ProtocolCore()
    denied_core.process(frame(MSG_HEARTBEAT, 0x60, struct.pack("<I", 1)))
    expect_statuses(
        "target gate denied",
        denied_core.process(frame(MSG_TARGET_CTRL, 0x61, bytes([TARGET_CTRL_START]))),
        [
            (STATUS_COMMAND_ACK, 0x00, ERROR_TARGET_GATE_DENIED),
            (STATUS_TARGET_CTRL, 0x00, ERROR_TARGET_GATE_DENIED),
        ],
    )

    # Continuous monitoring is a protocol-change proposal, not the production
    # RA6M5 contract.  Keep its model isolated behind an explicit switch so it
    # can be reviewed without making the default simulator accept action 0x04.
    continuous_now = [200.0]
    continuous_core = Ra6m5ProtocolCore(
        enable_continuous_extension=True,
        clock=lambda: continuous_now[0],
    )
    expect_statuses(
        "continuous workflow start",
        continuous_core.process(
            frame(
                MSG_WORKFLOW_CTRL,
                0x70,
                bytes([WORKFLOW_ACTION_START_MEASURE_CONTINUOUS]),
            )
        ),
        [
            (STATUS_COMMAND_ACK, 0x01, ERROR_NONE),
            (STATUS_WORKFLOW, WORKFLOW_START_ACCEPTED, ERROR_NONE),
            (STATUS_WORKFLOW, WORKFLOW_MEASURE_POSITION_READY, ERROR_NONE),
        ],
    )
    for seq in (0x71, 0x72):
        require(
            continuous_core.process(
                frame(MSG_SAFE_DISTANCE, seq, struct.pack("<HB", 120, VISION_VALID))
            )
            == [],
            "continuous safe distance must not latch early",
        )
    expect_statuses(
        "continuous safe-distance latch",
        continuous_core.process(
            frame(MSG_SAFE_DISTANCE, 0x73, struct.pack("<HB", 120, VISION_VALID))
        ),
        [
            (STATUS_SAFE_DISTANCE, 0x01, ERROR_NONE),
            (STATUS_WORKFLOW, WORKFLOW_DISTANCE_SAFE_LATCHED, ERROR_NONE),
        ],
    )
    continuous_now[0] += 0.4
    require(
        continuous_core.process(
            frame(MSG_SAFE_DISTANCE, 0x74, struct.pack("<HB", 125, VISION_VALID))
        )
        == [],
        "post-latch continuous refresh should be accepted without another SAFE_OK",
    )
    expect_statuses(
        "continuous freshness timeout retreat",
        continuous_core.service_watchdogs(now=continuous_now[0] + 0.601),
        [
            (STATUS_SAFE_DISTANCE, 0x00, ERROR_SAFE_DISTANCE_TIMEOUT),
            (STATUS_WORKFLOW, WORKFLOW_RETREAT_DONE_WAIT_RESTART, ERROR_NONE),
        ],
    )
    require(
        not continuous_core.safe_distance_latched
        and continuous_core.workflow_state == WORKFLOW_RETREAT_DONE_WAIT_RESTART,
        "freshness timeout must clear the latch and finish the simulated retreat",
    )

    require(core.duplicate_controls >= 2, "duplicate controls were not observed")
    require(core.sequence_conflicts == 1, "SEQ conflict was not observed exactly once")


def split_frames(buffer: bytearray) -> List[bytes]:
    """Extract unified frames while tolerating noise and partial serial reads."""
    frames: List[bytes] = []
    while buffer:
        if buffer[0] != 0xA5:
            del buffer[0]
            continue
        if len(buffer) < 2:
            break
        if buffer[1] != 0x5A:
            del buffer[0]
            continue
        if len(buffer) < 6:
            break
        payload_len = buffer[5]
        if payload_len > UNIFIED_MAX_PAYLOAD:
            del buffer[0]
            continue
        total = 8 + payload_len
        if len(buffer) < total:
            break
        frames.append(bytes(buffer[:total]))
        del buffer[:total]
    return frames


class Ra6m5SerialSimulator(threading.Thread):
    """PTY-backed wrapper around :class:`Ra6m5ProtocolCore`."""

    def __init__(self, master_fd: int, core: Ra6m5ProtocolCore, verbose: bool = False):
        super().__init__(name="ra6m5-protocol-v5-simulator", daemon=True)
        self.master_fd = master_fd
        self.core = core
        self.verbose = bool(verbose)
        self._stop_event = threading.Event()
        self._write_lock = threading.RLock()
        self._buffer = bytearray()
        self._drop_once_types: List[int] = []
        self._dropped_keys: List[ControlKey] = []
        self.received_frames = 0
        self.dropped_frames = 0

    def stop(self) -> None:
        self._stop_event.set()

    def drop_next_control_response(self, msg_type: int) -> None:
        require(msg_type in CONTROL_TYPES, "only control responses may be fault-injected")
        self._drop_once_types.append(msg_type)

    def _write_replies(self, replies: Sequence[bytes]) -> None:
        with self._write_lock:
            for reply in replies:
                os.write(self.master_fd, reply)
                if self.verbose:
                    print("SIM TX {}".format(hex_bytes(reply)), flush=True)

    def set_p000(self, pressed: bool) -> None:
        self._write_replies(self.core.set_p000(pressed))

    def _handle_raw(self, raw: bytes) -> None:
        try:
            request = parse_unified_frame(raw)
        except Exception as exc:
            self.dropped_frames += 1
            if self.verbose:
                print("SIM DROP {}: {}".format(hex_bytes(raw), exc), flush=True)
            return
        self.received_frames += 1
        if self.verbose:
            print("SIM RX {}".format(hex_bytes(raw)), flush=True)
        # STATUS/ERROR are RA6M5-to-Jetson frames.  Ignore an unexpected
        # reverse-direction copy instead of replying and creating a PTY echo
        # loop if a terminal has not yet configured the slave side as raw.
        if request.msg_type in (MSG_STATUS, MSG_ERROR):
            if self.verbose:
                print("SIM IGNORE reverse-direction frame", flush=True)
            return
        should_drop = False
        key = (request.msg_type, request.seq)
        if request.msg_type in self._drop_once_types and key not in self._dropped_keys:
            self._drop_once_types.remove(request.msg_type)
            self._dropped_keys.append(key)
            should_drop = True
        replies = self.core.process(request)
        if should_drop:
            if self.verbose:
                print("SIM FAULT-INJECT drop first control reply key={!r}".format(key), flush=True)
            return
        self._write_replies(replies)

    def run(self) -> None:
        os.set_blocking(self.master_fd, False)
        while not self._stop_event.is_set():
            # Watchdogs are serviced even when Jetson sends no bytes.  This is
            # essential because HEARTBEAT may remain healthy while the safety
            # measurement stream is silent.
            try:
                self._write_replies(self.core.service_watchdogs())
            except OSError:
                break
            try:
                ready, _, _ = select.select([self.master_fd], [], [], 0.02)
            except (OSError, ValueError):
                break
            if not ready:
                continue
            try:
                data = os.read(self.master_fd, 4096)
            except BlockingIOError:
                continue
            except OSError:
                break
            if not data:
                continue
            self._buffer.extend(data)
            for raw in split_frames(self._buffer):
                self._handle_raw(raw)


def wait_until(predicate: Callable[[], bool], timeout_s: float, label: str) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.01)
    raise SmokeCheckError("timeout waiting for {}".format(label))


def label_present(labels: Sequence[str], expected: str) -> bool:
    return any(label == expected or label.startswith(expected + "_ERR_") for label in labels)


def collect_sender_statuses(
    sender: SerialSender,
    expected: Sequence[str],
    timeout_s: float,
    wait_for_control_done: bool = False,
) -> Tuple[List[str], List[str]]:
    labels: List[str] = []
    transport_events: List[str] = []
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        labels.extend(status.label for status in sender.poll_statuses())
        transport_events.extend(sender.pop_transport_events())
        have_expected = all(label_present(labels, item) for item in expected)
        if have_expected and (not wait_for_control_done or not sender.control_busy):
            return labels, transport_events
        time.sleep(0.01)
    raise SmokeCheckError(
        "timeout waiting for statuses {!r}; received={!r}; transport={!r}; last_error={!r}".format(
            list(expected), labels, transport_events, sender.last_error
        )
    )


def run_sender_control(
    sender: SerialSender,
    label: str,
    action: Callable[[], bool],
    expected: Sequence[str],
    timeout_s: float = 2.5,
) -> Tuple[int, List[str], List[str]]:
    require(action(), "{} send failed: {}".format(label, sender.last_error))
    pending = sender.pending_control
    require(pending is not None, "{} did not create a pending control".format(label))
    seq = pending.seq
    labels, events = collect_sender_statuses(
        sender,
        expected,
        timeout_s,
        wait_for_control_done=True,
    )
    print("  {:24s} seq={:3d} -> {}".format(label, seq, ", ".join(labels)), flush=True)
    return seq, labels, events


def run_serial_sender_self_check(verbose: bool = False) -> None:
    master_fd, slave_fd = pty.openpty()
    tty.setraw(slave_fd)
    slave_name = os.ttyname(slave_fd)
    core = Ra6m5ProtocolCore()
    simulator = Ra6m5SerialSimulator(master_fd, core, verbose=verbose)
    simulator.drop_next_control_response(MSG_WORKFLOW_CTRL)
    simulator.start()
    sender: Optional[SerialSender] = None
    all_transport_events: List[str] = []
    try:
        sender = SerialSender(
            slave_name,
            115200,
            protocol="unified",
            timeout=0.02,
            write_timeout=0.05,
            heartbeat_interval=0.15,
            vision_interval=0.10,
            auto_heartbeat=True,
            auto_vision=True,
        )
        os.close(slave_fd)
        slave_fd = -1
        wait_until(lambda: core.heartbeat_alive, 1.0, "independent heartbeat")
        print("  pseudo serial: {}".format(slave_name), flush=True)

        start_seq, _, events = run_sender_control(
            sender,
            "WORKFLOW START",
            sender.send_workflow_start,
            ["COMMAND_ACK_ACCEPTED", "WORKFLOW_START_ACCEPTED", "MEASURE_POSITION_READY"],
        )
        all_transport_events.extend(events)
        require(
            core.control_dispatch_count[(MSG_WORKFLOW_CTRL, start_seq)] == 1,
            "ACK retry executed WORKFLOW START more than once",
        )
        require(core.duplicate_controls >= 1, "fault injection did not cause an exact-frame retry")

        require(sender.send_safe_distance(120, True), sender.last_error)
        time.sleep(0.03)
        require(sender.send_safe_distance(121, True), sender.last_error)
        time.sleep(0.03)
        require(sender.send_safe_distance(122, True), sender.last_error)
        labels, events = collect_sender_statuses(
            sender,
            ["SAFE_DISTANCE_OK", "DISTANCE_SAFE_LATCHED"],
            1.0,
        )
        all_transport_events.extend(events)
        print("  {:24s}         -> {}".format("SAFE DISTANCE x3", ", ".join(labels)), flush=True)

        capture_actions = [
            ("CAPTURE HOME (before)", sender.send_capture_home, ["CAPTURE_HOME_READY"]),
            ("CAPTURE GOTO 1", lambda: sender.send_capture_move(1), ["CAPTURE_POINT_1_READY"]),
            ("CAPTURE GOTO 2", lambda: sender.send_capture_move(2), ["CAPTURE_POINT_2_READY"]),
            ("CAPTURE GOTO 3", lambda: sender.send_capture_move(3), ["CAPTURE_POINT_3_READY"]),
            ("CAPTURE HOME (after)", sender.send_capture_home, ["CAPTURE_HOME_READY"]),
            ("CAPTURE SELECT 2", lambda: sender.send_capture_select(2), ["SELECTED_VIEW_2_READY"]),
        ]
        for label, action, expected in capture_actions:
            _, _, events = run_sender_control(
                sender,
                label,
                action,
                ["COMMAND_ACK_ACCEPTED"] + expected,
            )
            all_transport_events.extend(events)

        target_seq, _, events = run_sender_control(
            sender,
            "TARGET START",
            lambda: sender.send_target_ctrl(True),
            ["COMMAND_ACK_ACCEPTED", "TARGET_CTRL_ON", "TARGET_READY"],
        )
        all_transport_events.extend(events)
        require(sender.active_target_seq == target_seq, "SerialSender did not retain TARGET START SEQ")

        # With no published camera sample, the independent serial worker must
        # keep the RA6M5 vision watchdog alive using explicit valid=0 frames.
        labels, events = collect_sender_statuses(sender, ["VISION_LOST"], 1.0)
        all_transport_events.extend(events)
        print("  {:24s}         -> {}".format("VISION invalid", ", ".join(labels)), flush=True)

        # Publish faster than the 100 ms wire cadence.  The latest-only
        # mailbox may overwrite intermediate camera results; each wire sample
        # is still consumed only once by the background scheduler.
        aligned_labels: List[str] = []
        aligned_deadline = time.monotonic() + 1.5
        while time.monotonic() < aligned_deadline and not label_present(
            aligned_labels,
            "ALIGN_DONE",
        ):
            require(
                sender.publish_vision_sample(2, -3, True, time.monotonic()),
                sender.last_error,
            )
            time.sleep(0.03)
            aligned_labels.extend(status.label for status in sender.poll_statuses())
            all_transport_events.extend(sender.pop_transport_events())
        if not label_present(aligned_labels, "ALIGN_DONE"):
            more, events = collect_sender_statuses(sender, ["ALIGN_DONE"], 1.0)
            aligned_labels.extend(more)
            all_transport_events.extend(events)
        require(label_present(aligned_labels, "VISION_VALID"), "valid vision recovery was not reported")
        print("  {:24s}         -> {}".format("VISION valid x3", ", ".join(aligned_labels)), flush=True)

        simulator.set_p000(True)
        labels, events = collect_sender_statuses(sender, ["OUTPUT_ON"], 1.0)
        all_transport_events.extend(events)
        print("  {:24s}         -> {}".format("P000 press", ", ".join(labels)), flush=True)

        # Stop publishing.  The next scheduler period must invalidate vision
        # without replaying the last coordinates.
        labels, events = collect_sender_statuses(sender, ["OUTPUT_OFF", "VISION_LOST"], 1.0)
        all_transport_events.extend(events)
        print("  {:24s}         -> {}".format("VISION loss/output off", ", ".join(labels)), flush=True)
        simulator.set_p000(False)

        _, _, events = run_sender_control(
            sender,
            "TARGET STOP/HOLD",
            lambda: sender.send_target_ctrl(False),
            ["COMMAND_ACK_ACCEPTED", "TARGET_CTRL_OFF"],
        )
        all_transport_events.extend(events)
        _, _, events = run_sender_control(
            sender,
            "WORKFLOW FINISH",
            sender.send_workflow_finish,
            ["COMMAND_ACK_ACCEPTED", "RETURN_HOME_DONE"],
        )
        all_transport_events.extend(events)

        _, _, events = run_sender_control(
            sender,
            "WORKFLOW RESTART",
            sender.send_workflow_start,
            ["COMMAND_ACK_ACCEPTED", "WORKFLOW_START_ACCEPTED", "MEASURE_POSITION_READY"],
        )
        all_transport_events.extend(events)
        _, _, events = run_sender_control(
            sender,
            "WORKFLOW ABORT",
            sender.send_workflow_abort,
            ["COMMAND_ACK_ACCEPTED", "ABORTED_HOLD"],
        )
        all_transport_events.extend(events)

        require(not sender.control_busy, "a control request remained pending after smoke test")
        require(core.workflow_state == WORKFLOW_ABORTED_HOLD, "ABORT must end in hold, not HOME")
        require(not core.output_on, "P801 simulation must be off after abort")
        require(not core.safe_distance_latched, "abort must clear the distance latch")
        require(simulator.dropped_frames == 0, "simulator dropped a malformed Jetson frame")
        require(
            any("控制命令已接受" in event for event in all_transport_events),
            "SerialSender did not observe control ACKs",
        )
    finally:
        if sender is not None:
            sender.close()
        simulator.stop()
        simulator.join(timeout=0.5)
        if slave_fd >= 0:
            os.close(slave_fd)
        os.close(master_fd)


def _print_server_status(core: Ra6m5ProtocolCore, simulator: Ra6m5SerialSimulator) -> None:
    print(
        "SIM STATUS workflow=0x{:02X} safe_latched={} capture_stage={} "
        "target_enabled={} target_ready={} p000={} output_on={} rx_frames={}".format(
            core.workflow_state,
            int(core.safe_distance_latched),
            core.capture_stage,
            int(core.target_enabled),
            int(core.target_ready),
            int(core.p000_pressed),
            int(core.output_on),
            simulator.received_frames,
        ),
        flush=True,
    )


def run_serial_server(verbose: bool = False) -> int:
    """Expose the RA6M5 model as a PTY for an external Qt process."""
    master_fd, slave_fd = pty.openpty()
    tty.setraw(slave_fd)
    slave_name = os.ttyname(slave_fd)
    core = Ra6m5ProtocolCore()
    simulator = Ra6m5SerialSimulator(master_fd, core, verbose=verbose)
    simulator.start()
    stdin_open = True
    try:
        print("SIMULATED_SERIAL={}".format(slave_name), flush=True)
        print("QT_ARGS=--serial-port {} --serial-protocol unified".format(slave_name), flush=True)
        print("COMMANDS: status | p000 on | p000 off | quit", flush=True)
        while True:
            if not stdin_open:
                time.sleep(0.25)
                continue
            try:
                ready, _, _ = select.select([sys.stdin], [], [], 0.25)
            except (OSError, ValueError):
                stdin_open = False
                continue
            if not ready:
                continue
            line = sys.stdin.readline()
            if not line:
                stdin_open = False
                continue
            command = line.strip().lower()
            if not command:
                continue
            if command in ("quit", "exit", "q"):
                break
            if command == "status":
                _print_server_status(core, simulator)
                continue
            if command in ("p000 on", "p000 1", "p000 press"):
                simulator.set_p000(True)
                print("SIM P000=ON", flush=True)
                continue
            if command in ("p000 off", "p000 0", "p000 release"):
                simulator.set_p000(False)
                print("SIM P000=OFF", flush=True)
                continue
            if command in ("help", "?"):
                print("命令: status | p000 on | p000 off | quit", flush=True)
                continue
            print("未知命令：{}；输入 help 查看命令".format(command), flush=True)
        return 0
    except KeyboardInterrupt:
        print("\nSIM server stopped", flush=True)
        return 0
    finally:
        simulator.stop()
        simulator.join(timeout=0.5)
        os.close(slave_fd)
        os.close(master_fd)


def run_qt_demo(verbose: bool = False) -> int:
    """Launch Qt with a private simulated RA6M5 and simulated camera frames."""
    master_fd, slave_fd = pty.openpty()
    tty.setraw(slave_fd)
    slave_name = os.ttyname(slave_fd)
    core = Ra6m5ProtocolCore()
    simulator = Ra6m5SerialSimulator(master_fd, core, verbose=verbose)
    simulator.start()
    process: Optional[subprocess.Popen] = None
    try:
        command = [
            "/usr/bin/python3",
            "-m",
            "app.main",
            "--demo",
            "--detector-mode",
            "spot",
            "--serial-port",
            slave_name,
            "--baudrate",
            "115200",
            "--serial-protocol",
            "unified",
            "--serial-safe-distance-source",
            "fixed",
            "--serial-safe-distance-mm",
            "120",
            "--serial-safe-distance-valid",
            "1",
        ]
        environment = os.environ.copy()
        environment.pop("PYTHONHOME", None)
        environment.pop("PYTHONNOUSERSITE", None)
        environment.setdefault("DISPLAY", ":0")
        print("STARTING_QT_SIM serial={}".format(slave_name), flush=True)
        print("Close the Qt window to stop the simulator.", flush=True)
        process = subprocess.Popen(
            command,
            cwd=os.path.join(ROOT, "arm_mole_qt"),
            env=environment,
            stdout=subprocess.DEVNULL,
            start_new_session=True,
        )
        return int(process.wait())
    except KeyboardInterrupt:
        print("\nStopping Qt simulator...", flush=True)
        if process is not None and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=1.0)
        return 130
    finally:
        simulator.stop()
        simulator.join(timeout=0.5)
        os.close(slave_fd)
        os.close(master_fd)


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Hardware-free RA6M5 unified-protocol simulator/self-check"
    )
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument(
        "--core-only",
        action="store_true",
        help="run deterministic protocol checks without opening a pseudo serial port",
    )
    mode.add_argument(
        "--serve",
        action="store_true",
        help="keep a pseudo serial port open so the Qt application can connect",
    )
    mode.add_argument(
        "--qt-demo",
        action="store_true",
        help="launch the Qt demo with a private simulated RA6M5 serial port",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="print every raw simulator RX/TX frame",
    )
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    if args.qt_demo:
        return run_qt_demo(verbose=args.verbose)
    if args.serve:
        return run_serial_server(verbose=args.verbose)
    try:
        print("[1/2] protocol-v5 state-machine self-check", flush=True)
        run_core_self_check()
        print(
            "      PASS: production 0x01 flow, 0x04 rejection, ACK order, gates, "
            "views, target, vision, retry/conflict; proposal model checked separately",
            flush=True,
        )
        if args.core_only:
            print("RESULT: PASS (core only)", flush=True)
            return 0

        print("[2/2] SerialSender <-> pseudo RA6M5 end-to-end smoke test", flush=True)
        run_serial_sender_self_check(verbose=args.verbose)
        print("RESULT: PASS formal unified protocol smoke test", flush=True)
        return 0
    except (SmokeCheckError, OSError) as exc:
        print("RESULT: FAIL {}".format(exc), file=sys.stderr, flush=True)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
