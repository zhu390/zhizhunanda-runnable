"""Thread-safe serial transport for the Jetson <-> RA6M5 protocol."""

from collections import OrderedDict
from dataclasses import dataclass
import math
from threading import Event, RLock, Thread, current_thread
import time
from typing import Dict, List, Optional, Tuple

from .vision_protocol import (
    CAPTURE_ACTION_DONE_HOME,
    CAPTURE_ACTION_MOVE_TO_POINT,
    CAPTURE_ACTION_SELECT_POINT,
    ERROR_NONE,
    MSG_CAPTURE_CTRL,
    MSG_ERROR,
    MSG_TARGET_CTRL,
    MSG_WORKFLOW_CTRL,
    Ra6Status,
    Ra6StatusParser,
    STATUS_CAPTURE_HOME_READY,
    STATUS_CAPTURE_POINT_READY,
    STATUS_COMMAND_ACK,
    STATUS_ALIGN_DONE,
    STATUS_OUTPUT,
    STATUS_SAFE_DISTANCE,
    STATUS_SELECTED_VIEW_READY,
    STATUS_TARGET_CTRL,
    STATUS_TARGET_READY,
    STATUS_VISION_STATE,
    STATUS_WORKFLOW,
    WORKFLOW_FAULT_HOLD,
    WORKFLOW_IDLE,
    WORKFLOW_ABORTED_HOLD,
    WORKFLOW_ACTION_ABORT_HOLD,
    WORKFLOW_ACTION_FINISH_RETURN_HOME,
    WORKFLOW_ACTION_START_MEASURE,
    WORKFLOW_ACTION_START_MEASURE_CONTINUOUS,
    WORKFLOW_MEASURE_POSITION_READY,
    WORKFLOW_DISTANCE_SAFE_LATCHED,
    WORKFLOW_RETREAT_DONE_WAIT_RESTART,
    WORKFLOW_RETURN_HOME_DONE,
    WORKFLOW_START_ACCEPTED,
    build_unified_frame,
    pack_laser_output,
    pack_target_ctrl,
    pack_unified_capture_ctrl,
    pack_unified_heartbeat,
    pack_unified_safe_distance,
    pack_unified_target_ctrl,
    pack_unified_vision_error,
    pack_unified_workflow_ctrl,
    pack_vision_error,
    pixel_error_from_target,
)


CONTROL_ACK_TIMEOUT_S = 0.300
CONTROL_MAX_RETRIES = 3
# RA6M5 currently keeps the most recent eight control requests, while an
# older/future firmware recommendation allows as many as 64.  Streaming
# HEARTBEAT/VISION frames also consume the shared 8-bit SEQ and can therefore
# wrap long before those control entries are evicted.  Keep a conservative
# local lease for recently transmitted control keys so a new command never
# reuses a still-cacheable (TYPE, SEQ) during this transport lifetime.
#
# This is intentionally not a retry mechanism: an explicit SEQ_CONFLICT still
# fails closed.  Firmware must provide a session/TTL boundary to cover a
# Jetson process restart, where this in-memory lease cannot know the old cache.
CONTROL_SEQ_REUSE_GUARD = 64
IO_FAILURE_LIMIT = 3
# CH34x on Jetson can briefly exceed 20 ms while the RGB-D pipeline starts.
# 200 ms still stays well inside the RA6M5 600 ms heartbeat watchdog, while
# avoiding a false terminal short/uncertain-write latch under normal load.
SERIAL_WRITE_TIMEOUT_S = 0.200
WORKFLOW_START_ACTIONS = (
    WORKFLOW_ACTION_START_MEASURE,
)

UNSUPPORTED_CONTINUOUS_WORKFLOW_MESSAGE = (
    "当前 RA6M5 正式协议只定义 START_MEASURE action=0x01；"
    "START_MEASURE_CONTINUOUS action=0x04 仍是未批准的协议提案"
)


class _SerialShortWriteError(IOError):
    """A partial frame is already on the wire and must stop all later TX."""


@dataclass
class PendingControl:
    """One in-flight control request and its exact retransmission frame."""

    label: str
    msg_type: int
    seq: int
    payload: bytes
    frame: bytes
    expected_event: int
    expected_value: int
    completion_timeout_s: float
    first_sent_at: float
    last_sent_at: float
    retries: int = 0
    acknowledged: bool = False
    acknowledged_at: Optional[float] = None
    target_ctrl_on_seen: bool = False


@dataclass(frozen=True)
class PublishedVisionSample:
    """One latest-only camera result bound to a TARGET_CTRL session."""

    target_seq: int
    dcx: int
    dcy: int
    valid: bool
    source_time: float


class SerialSender:
    """Serial transport with heartbeat and protocol-v5 control reliability.

    In unified mode the heartbeat runs in its own thread so camera inference or
    a busy Qt event loop cannot accidentally exceed RA6M5's 600 ms watchdog.
    Only one normal control command may be in flight.  ACK timeout retries use
    the exact same bytes (and therefore the same SEQ and payload).
    """

    def __init__(
        self,
        port: str = "COM3",
        baudrate: int = 115200,
        timeout: float = 0.02,
        write_timeout: float = SERIAL_WRITE_TIMEOUT_S,
        protocol: str = "unified",
        heartbeat_interval: float = 0.15,
        vision_interval: float = 0.15,
        vision_max_age: float = 0.5,
        vision_loss_grace: float = 0.25,
        auto_heartbeat: bool = True,
        auto_vision: bool = False,
    ):
        import serial  # type: ignore

        if protocol not in ("unified", "legacy"):
            raise ValueError("protocol must be unified or legacy")
        self._ser = serial.Serial(
            port,
            baudrate,
            timeout=timeout,
            write_timeout=write_timeout,
        )
        self.protocol = protocol
        self._status_parser = Ra6StatusParser(allow_legacy=protocol == "legacy")
        # Jetson TX uses only 0x01..0xFF.  SEQ=0 is reserved for RA6M5
        # asynchronous faults, but the receive parser must continue accepting
        # it.
        self._seq = 1
        self._start_time = time.monotonic()
        self._tx_lock = RLock()
        # Set while still holding _tx_lock as soon as write() becomes
        # uncertain/partial.  This closes the tiny gap before the richer
        # transport event is recorded under _control_lock.
        self._tx_terminal_stop = Event()
        self._rx_lock = RLock()
        self._control_lock = RLock()
        self._closing = Event()
        self._pending_control: Optional[PendingControl] = None
        self._recent_control_keys: "OrderedDict[Tuple[int, int], None]" = OrderedDict()
        self._active_target_seq: Optional[int] = None
        self._closing_target_seq: Optional[int] = None
        # Qt publishes only the newest camera result into this mailbox.  Each
        # source sample is transmitted at most once; an empty camera interval
        # is silence, not a fabricated target loss.  Explicit invalid samples
        # are kept separately so a newer recovery sample cannot hide a real
        # one-frame loss before the serial worker observes it.  A previously
        # transmitted valid sample produces one final valid=0 when it ages out,
        # but its coordinates are never replayed.
        self.vision_interval = max(0.10, min(0.20, float(vision_interval)))
        self.vision_max_age = max(0.10, min(0.50, float(vision_max_age)))
        self.vision_loss_grace = max(0.0, min(0.50, float(vision_loss_grace)))
        self.vision_loss_timeout = self.vision_max_age + self.vision_loss_grace
        self._vision_mailbox: Optional[PublishedVisionSample] = None
        self._vision_pending_invalid: Optional[PublishedVisionSample] = None
        self._vision_latest_source_time = 0.0
        self._vision_last_consumed_source_time = 0.0
        self._vision_last_expired_source_time = 0.0
        self._vision_initial_invalid_sent = False
        self._next_vision_at = 0.0
        self._active_workflow_seq: Optional[int] = None
        self._measure_window_open = False
        self._continuous_safe_distance = False
        # Maps every current measurement sample SEQ to its valid bit.  A SAFE
        # permission may only answer a sample that Jetson actually sent with
        # valid=1; invalid samples can never unlock a workflow.
        self._safe_sample_seqs: Dict[int, bool] = {}
        self._last_safe_distance_seq: Optional[int] = None
        self._safe_decision_received = False
        self._workflow_phase = -1
        self._transport_events: List[str] = []
        self._received_statuses: List[Ra6Status] = []
        self._tx_failure_count = 0
        self._rx_failure_count = 0
        self._io_failure_latched = False
        self.last_error = ""

        self.heartbeat_interval = max(0.10, min(0.20, float(heartbeat_interval)))
        self._heartbeat_stop = Event()
        self._heartbeat_thread: Optional[Thread] = None
        self._background_vision_enabled = bool(
            self.protocol == "unified" and auto_heartbeat and auto_vision
        )
        if self.protocol == "unified" and auto_heartbeat:
            self._heartbeat_thread = Thread(
                target=self._heartbeat_loop,
                name="ra6m5-heartbeat",
                daemon=True,
            )
            self._heartbeat_thread.start()

    @property
    def is_open(self) -> bool:
        return bool(
            not self._closing.is_set()
            and not self._tx_terminal_stop.is_set()
            and not self._io_failure_latched
            and self._ser
            and self._ser.is_open
        )

    @property
    def control_busy(self) -> bool:
        with self._control_lock:
            return self._pending_control is not None

    @property
    def pending_control(self) -> Optional[PendingControl]:
        with self._control_lock:
            return self._pending_control

    @property
    def active_target_seq(self) -> Optional[int]:
        with self._control_lock:
            return self._active_target_seq

    @property
    def active_workflow_seq(self) -> Optional[int]:
        with self._control_lock:
            return self._active_workflow_seq

    @property
    def background_vision_enabled(self) -> bool:
        """Whether the independent serial worker owns VISION_ERROR cadence."""

        return self._background_vision_enabled

    @property
    def measure_window_open(self) -> bool:
        with self._control_lock:
            return self._measure_window_open

    @property
    def last_safe_distance_seq(self) -> Optional[int]:
        """SEQ assigned to the most recently transmitted SAFE_DISTANCE."""

        with self._control_lock:
            return self._last_safe_distance_seq

    def _format_frame(self, frame: bytes) -> str:
        return " ".join(f"{byte:02X}" for byte in frame)

    def _write_frame(self, label: str, frame: bytes) -> None:
        """Write one complete frame through the transport's sole TX entry."""

        frame = bytes(frame)
        write_attempted = False
        try:
            with self._tx_lock:
                if self._closing.is_set():
                    raise IOError("serial transport is closing")
                if self._tx_terminal_stop.is_set() or self._io_failure_latched:
                    raise IOError("serial transport is locked after an I/O failure")
                if not self._ser or not self._ser.is_open:
                    raise IOError("serial port is closed")
                # Keep the complete frame and the short-write check inside the
                # same global TX critical section.  No message type writes to
                # pyserial anywhere else in this transport.
                write_attempted = True
                try:
                    written = self._ser.write(frame)
                    if written != len(frame):
                        raise _SerialShortWriteError(
                            "serial short write: {} of {} bytes".format(
                                written,
                                len(frame),
                            )
                        )
                except Exception:
                    # Must happen before releasing _tx_lock: a waiting sender
                    # must see the stop flag before it can append a new frame
                    # to bytes whose completion is unknown.
                    self._tx_terminal_stop.set()
                    raise
        except Exception as exc:
            if not self._closing.is_set():
                self._record_io_failure(
                    "TX",
                    exc,
                    # Once pyserial.write() was entered, an exception cannot
                    # prove that zero bytes reached the UART.  Treat it like a
                    # partial frame and require a reconnect before any next TX.
                    terminal=write_attempted,
                )
            raise
        self._record_io_success("TX")
        print(f"TX {label}: {self._format_frame(frame)}", flush=True)

    def _next_seq(self) -> int:
        with self._tx_lock:
            seq = self._seq & 0xFF
            if seq == 0:
                seq = 1
            self._seq = 1 if seq == 0xFF else seq + 1
            return seq

    def _next_control_seq(self, msg_type: int) -> int:
        """Allocate a nonzero SEQ outside the recent RA6M5 control window.

        Sequence gaps are valid on a serial protocol (they are
        indistinguishable from a lost frame).  Skipping a leased key is safer
        than reusing it after HEARTBEAT/VISION traffic wraps the global 8-bit
        counter while RA6M5 still remembers the old control request.
        """

        control_type = int(msg_type) & 0xFF
        for _ in range(0xFF):
            seq = self._next_seq()
            with self._control_lock:
                if (control_type, seq) not in self._recent_control_keys:
                    return seq
        raise RuntimeError("no control SEQ is available outside the reuse guard")

    def _remember_control_key_locked(self, msg_type: int, seq: int) -> None:
        key = (int(msg_type) & 0xFF, int(seq) & 0xFF)
        self._recent_control_keys[key] = None
        self._recent_control_keys.move_to_end(key)
        while len(self._recent_control_keys) > CONTROL_SEQ_REUSE_GUARD:
            self._recent_control_keys.popitem(last=False)

    def _tick_ms(self) -> int:
        return int((time.monotonic() - self._start_time) * 1000.0) & 0xFFFFFFFF

    def _heartbeat_loop(self) -> None:
        next_heartbeat = 0.0
        while not self._heartbeat_stop.is_set():
            now = time.monotonic()
            if self.is_open:
                if now >= next_heartbeat:
                    self.send_heartbeat()
                    next_heartbeat = now + self.heartbeat_interval
                self._drain_receive_to_queue()
                self.service_control(now)
                if self._background_vision_enabled:
                    self.service_vision(time.monotonic())
            self._heartbeat_stop.wait(0.02)

    def _reset_vision_mailbox_locked(self) -> None:
        self._vision_mailbox = None
        self._vision_pending_invalid = None
        self._vision_latest_source_time = 0.0
        self._vision_last_consumed_source_time = 0.0
        self._vision_last_expired_source_time = 0.0
        self._vision_initial_invalid_sent = False
        self._next_vision_at = 0.0

    def publish_vision_sample(
        self,
        dcx: int,
        dcy: int,
        valid: bool,
        source_time: float,
    ) -> bool:
        """Publish one result for background transmission without writing UART.

        The mailbox is bound to the active TARGET_CTRL SEQ.  Repeated or
        out-of-order Qt signals are ignored so an older valid coordinate can
        never overwrite a newer invalid observation.
        """

        if self.protocol != "unified":
            self.last_error = "legacy protocol has no background vision scheduler"
            return False
        try:
            timestamp = float(source_time)
            sample_dcx = int(dcx)
            sample_dcy = int(dcy)
        except (TypeError, ValueError, OverflowError) as exc:
            self.last_error = "invalid vision mailbox sample: {}".format(exc)
            return False
        if not math.isfinite(timestamp) or timestamp <= 0.0:
            self.last_error = "vision source_time must be a positive finite monotonic timestamp"
            return False
        if not (-32768 <= sample_dcx <= 32767 and -32768 <= sample_dcy <= 32767):
            self.last_error = "vision dcx/dcy must fit int16"
            return False

        with self._control_lock:
            target_seq = self._active_target_seq
            if target_seq is None:
                self.last_error = "VISION_ERROR mailbox is only allowed after TARGET_READY"
                return False
            if timestamp <= self._vision_latest_source_time:
                self.last_error = "ignored duplicate or out-of-order vision sample"
                return False
            self._vision_latest_source_time = timestamp
            sample = PublishedVisionSample(
                target_seq=target_seq,
                dcx=sample_dcx if valid else 0,
                dcy=sample_dcy if valid else 0,
                valid=bool(valid),
                source_time=timestamp,
            )
            self._vision_mailbox = sample
            if not sample.valid:
                # Do not let a newer valid observation erase a real loss that
                # arrived between two serial-worker iterations.  The pending
                # invalid is retained until the next allowed wire period, then
                # the newest valid sample may follow in a later period.
                self._vision_pending_invalid = sample
        self.last_error = ""
        return True

    def service_vision(self, now: Optional[float] = None) -> None:
        """Transmit new vision state without inventing periodic invalid frames.

        A camera sample is sent at most once.  Explicit target loss is retained
        even if a recovery result arrives before the serial worker sees it,
        while every frame still observes the configured 100--200 ms rate
        limit.  After a valid sample has been sent, silence is preserved until
        either a new sample arrives or the last valid observation becomes
        stale; staleness emits exactly one valid=0 notification.
        """

        if self.protocol != "unified":
            return
        timestamp = time.monotonic() if now is None else float(now)
        with self._control_lock:
            target_seq = self._active_target_seq
            if target_seq is None:
                self._next_vision_at = 0.0
                return
            sample = self._vision_mailbox

            dcx = 0
            dcy = 0
            valid = False
            should_send = False

            pending_invalid = self._vision_pending_invalid
            if pending_invalid is not None:
                # A real detector invalidation must not wait behind the normal
                # mailbox and must not be overwritten by a newer valid sample.
                # It still respects the minimum wire interval so a 30 FPS loss
                # stream cannot flood the shared UART at 50 Hz.
                if timestamp < self._next_vision_at:
                    return
                self._vision_pending_invalid = None
                self._vision_last_consumed_source_time = max(
                    self._vision_last_consumed_source_time,
                    pending_invalid.source_time,
                )
                self._vision_initial_invalid_sent = True
                should_send = True
            elif (
                sample is not None
                and sample.valid
                and sample.target_seq == target_seq
                and timestamp - sample.source_time > self.vision_loss_timeout
                and sample.source_time != self._vision_last_expired_source_time
            ):
                # Whether or not this valid sample was transmitted before it
                # expired, notify RA6M5 once and never replay its coordinates.
                self._vision_last_consumed_source_time = max(
                    self._vision_last_consumed_source_time,
                    sample.source_time,
                )
                self._vision_last_expired_source_time = sample.source_time
                self._vision_initial_invalid_sent = True
                should_send = True
            elif sample is None:
                # Open a TARGET_READY session in a known fail-closed state,
                # then stay silent until the camera publishes something.
                if not self._vision_initial_invalid_sent:
                    self._vision_initial_invalid_sent = True
                    should_send = True
            elif sample.source_time > self._vision_last_consumed_source_time:
                if timestamp < self._next_vision_at:
                    return
                self._vision_last_consumed_source_time = sample.source_time
                age = timestamp - sample.source_time
                if (
                    sample.valid
                    and self.vision_max_age < age <= self.vision_loss_timeout
                ):
                    # Never transmit an already-stale coordinate, but do not
                    # turn a short inference/UI scheduling hiccup into an
                    # explicit target loss.  A newer sample can replace it;
                    # otherwise the expiration branch emits one invalid after
                    # the bounded grace window.
                    return
                valid = bool(
                    sample.target_seq == target_seq
                    and sample.valid
                    and 0.0 <= age <= self.vision_max_age
                )
                if valid:
                    dcx = sample.dcx
                    dcy = sample.dcy
                else:
                    self._vision_initial_invalid_sent = True
                    if sample.valid:
                        self._vision_last_expired_source_time = sample.source_time
                should_send = True

            if not should_send:
                return

            # Advance from "now", never by repeated += interval catch-up.  A
            # temporarily slow UART therefore cannot produce a burst after it
            # recovers.  Explicit/stale invalids also start a fresh period so a
            # recovery frame cannot immediately follow them on the wire.
            self._next_vision_at = timestamp + self.vision_interval

            # Keep the session permission check and complete frame write under
            # the same state lock.  RX cannot close TARGET_CTRL in the gap.
            self.send_vision_error(dcx, dcy, valid=valid)

    def _record_transport_event(self, message: str) -> None:
        with self._control_lock:
            self._transport_events.append(message)

    def _record_io_success(self, direction: str) -> None:
        with self._control_lock:
            if direction == "TX":
                self._tx_failure_count = 0
            else:
                self._rx_failure_count = 0

    def _record_io_failure(
        self,
        direction: str,
        exc: Exception,
        *,
        terminal: bool = False,
    ) -> None:
        with self._control_lock:
            if direction == "TX":
                self._tx_failure_count += 1
                failures = self._tx_failure_count
            else:
                self._rx_failure_count += 1
                failures = self._rx_failure_count
            if self._io_failure_latched:
                return
            if not terminal and failures < IO_FAILURE_LIMIT:
                return
            self._io_failure_latched = True
            if terminal:
                if isinstance(exc, _SerialShortWriteError):
                    message = "串口帧短写：{}".format(exc)
                else:
                    message = (
                        "串口TX写入失败且无法确认是否已写入部分帧：{}".format(exc)
                    )
                self._transport_events.append(
                    message + "；已立即停止后续发送，防止残帧与下一帧交叠"
                )
            else:
                self._transport_events.append(
                    "串口I/O连续失败：{} 已连续 {} 次失败，最后错误：{}".format(
                        direction,
                        failures,
                        exc,
                    )
                )
            # Stop claiming link health.  RA6M5 will also independently enter
            # its heartbeat-timeout hold if Qt cannot service this event.
            self._heartbeat_stop.set()

    def pop_transport_events(self) -> List[str]:
        with self._control_lock:
            events = list(self._transport_events)
            self._transport_events.clear()
        return events

    def send_heartbeat(self, tick_ms: Optional[int] = None) -> bool:
        if self.protocol != "unified":
            self.last_error = "legacy protocol has no heartbeat frame"
            return False
        try:
            tick = self._tick_ms() if tick_ms is None else int(tick_ms)
            frame = pack_unified_heartbeat(self._next_seq(), tick)
            self._write_frame(f"HEARTBEAT tick={tick}", frame)
            self.last_error = ""
            return True
        except Exception as exc:
            self.last_error = str(exc)
            return False

    def send_safe_distance(self, distance_mm: int, valid: bool = True) -> bool:
        if self.protocol != "unified":
            self.last_error = "legacy protocol has no safe-distance frame"
            return False
        with self._control_lock:
            self._last_safe_distance_seq = None
            if not self._measure_window_open or self._active_workflow_seq is None:
                self.last_error = (
                    "SAFE_DISTANCE is only allowed after the current "
                    "MEASURE_POSITION_READY"
                )
                return False
            # Keep the permission check, sample registration and complete TX in
            # the same state critical section.  The RX worker therefore cannot
            # close the measurement window on SAFE/LATCHED in the gap between
            # this check and the actual serial write.
            seq = self._next_seq()
            try:
                frame = pack_unified_safe_distance(seq, int(distance_mm), valid)
                self._safe_sample_seqs[seq] = bool(valid)
                self._last_safe_distance_seq = seq
                while len(self._safe_sample_seqs) > 32:
                    self._safe_sample_seqs.pop(next(iter(self._safe_sample_seqs)))
                self._write_frame(
                    f"SAFE_DISTANCE distance={int(distance_mm)} valid={1 if valid else 0}",
                    frame,
                )
                self.last_error = ""
                return True
            except Exception as exc:
                self._safe_sample_seqs.pop(seq, None)
                if self._last_safe_distance_seq == seq:
                    self._last_safe_distance_seq = None
                self.last_error = str(exc)
                return False

    def send_vision_error(self, dcx: int, dcy: int, valid: bool = True) -> bool:
        try:
            if self.protocol == "unified":
                frame = pack_unified_vision_error(self._next_seq(), dcx, dcy, valid)
                label = f"VISION_ERROR dcx={dcx} dcy={dcy} valid={1 if valid else 0}"
            else:
                if not valid:
                    self.last_error = "legacy protocol cannot represent VISION_ERROR valid=0"
                    return False
                frame = pack_vision_error(dcx, dcy)
                label = f"VISION_ERROR dcx={dcx} dcy={dcy}"
            self._write_frame(label, frame)
            self.last_error = ""
            return True
        except Exception as exc:
            self.last_error = str(exc)
            return False

    def _begin_unified_control(
        self,
        *,
        label: str,
        msg_type: int,
        payload: bytes,
        frame: bytes,
        seq: int,
        expected_event: int,
        expected_value: int,
        completion_timeout_s: float,
        priority: bool = False,
    ) -> bool:
        if self.protocol != "unified":
            self.last_error = f"legacy protocol has no {label} control"
            return False
        now = time.monotonic()
        with self._control_lock:
            if self._pending_control is not None and not priority:
                self.last_error = "control command busy: {} seq={}".format(
                    self._pending_control.label,
                    self._pending_control.seq,
                )
                return False
            if self._pending_control is not None and priority:
                self._transport_events.append(
                    "优先级控制替换待完成命令：{} seq={}".format(
                        self._pending_control.label,
                        self._pending_control.seq,
                    )
                )
            self._pending_control = PendingControl(
                label=label,
                msg_type=msg_type,
                seq=seq,
                payload=bytes(payload),
                frame=bytes(frame),
                expected_event=expected_event,
                expected_value=expected_value,
                completion_timeout_s=float(completion_timeout_s),
                first_sent_at=now,
                last_sent_at=now,
            )
            # Record only a command that has acquired the pending-control
            # slot.  Busy calls may consume a global SEQ, as before, but must
            # not evict a key that RA6M5 may actually still cache.
            self._remember_control_key_locked(msg_type, seq)
            if msg_type == MSG_WORKFLOW_CTRL:
                self._active_workflow_seq = seq
                self._measure_window_open = False
                self._safe_sample_seqs.clear()
                self._safe_decision_received = False
                if payload and payload[0] in WORKFLOW_START_ACTIONS:
                    self._workflow_phase = -1
                    self._continuous_safe_distance = (
                        payload[0] == WORKFLOW_ACTION_START_MEASURE_CONTINUOUS
                    )
                    self._active_target_seq = None
                    self._closing_target_seq = None
                    self._reset_vision_mailbox_locked()
                else:
                    self._continuous_safe_distance = False
                    self._closing_target_seq = self._active_target_seq
                    self._active_target_seq = None
                    self._reset_vision_mailbox_locked()
        try:
            self._write_frame(f"{label} seq={seq}", frame)
            self.last_error = ""
            return True
        except Exception as exc:
            with self._control_lock:
                if self._pending_control is not None and self._pending_control.seq == seq:
                    self._pending_control = None
                if msg_type == MSG_WORKFLOW_CTRL and self._active_workflow_seq == seq:
                    self._active_workflow_seq = None
                    self._measure_window_open = False
                    self._continuous_safe_distance = False
                    self._safe_sample_seqs.clear()
                    self._safe_decision_received = False
            self.last_error = str(exc)
            return False

    def send_capture_ctrl(self, action: int, point_id: int) -> bool:
        try:
            seq = self._next_control_seq(MSG_CAPTURE_CTRL)
            frame = pack_unified_capture_ctrl(seq, action, point_id)
        except Exception as exc:
            self.last_error = str(exc)
            return False
        if action == CAPTURE_ACTION_MOVE_TO_POINT:
            expected_event, expected_value, label = STATUS_CAPTURE_POINT_READY, point_id, "CAPTURE_GOTO"
        elif action == CAPTURE_ACTION_DONE_HOME:
            expected_event, expected_value, label = STATUS_CAPTURE_HOME_READY, 1, "CAPTURE_HOME"
        elif action == CAPTURE_ACTION_SELECT_POINT:
            expected_event, expected_value, label = STATUS_SELECTED_VIEW_READY, point_id, "CAPTURE_SELECT"
        else:  # validation above normally catches this
            self.last_error = "unsupported capture action"
            return False
        return self._begin_unified_control(
            label=f"{label} point={point_id}",
            msg_type=MSG_CAPTURE_CTRL,
            payload=bytes([action, point_id]),
            frame=frame,
            seq=seq,
            expected_event=expected_event,
            expected_value=expected_value,
            completion_timeout_s=15.0,
        )

    def send_capture_move(self, point_id: int) -> bool:
        return self.send_capture_ctrl(CAPTURE_ACTION_MOVE_TO_POINT, point_id)

    def send_capture_home(self) -> bool:
        return self.send_capture_ctrl(CAPTURE_ACTION_DONE_HOME, 0)

    def send_capture_select(self, point_id: int) -> bool:
        return self.send_capture_ctrl(CAPTURE_ACTION_SELECT_POINT, point_id)

    def send_target_ctrl(self, enabled: bool) -> bool:
        if self.protocol == "legacy":
            try:
                frame = pack_target_ctrl(enabled)
                self._write_frame("TARGET_CTRL legacy", frame)
                self.last_error = ""
                return True
            except Exception as exc:
                self.last_error = str(exc)
                return False
        try:
            seq = self._next_control_seq(MSG_TARGET_CTRL)
            frame = pack_unified_target_ctrl(seq, enabled)
        except Exception as exc:
            self.last_error = str(exc)
            return False
        return self._begin_unified_control(
            label="TARGET_START" if enabled else "TARGET_STOP_HOLD",
            msg_type=MSG_TARGET_CTRL,
            payload=bytes([0x01 if enabled else 0x00]),
            frame=frame,
            seq=seq,
            expected_event=STATUS_TARGET_READY if enabled else STATUS_TARGET_CTRL,
            expected_value=0x01 if enabled else 0x00,
            completion_timeout_s=2.0,
        )

    def send_workflow_ctrl(self, action: int) -> bool:
        if action == WORKFLOW_ACTION_START_MEASURE_CONTINUOUS:
            self.last_error = UNSUPPORTED_CONTINUOUS_WORKFLOW_MESSAGE
            return False
        try:
            seq = self._next_control_seq(MSG_WORKFLOW_CTRL)
            frame = pack_unified_workflow_ctrl(seq, action)
        except Exception as exc:
            self.last_error = str(exc)
            return False
        if action in WORKFLOW_START_ACTIONS:
            label = (
                "WORKFLOW_START_MEASURE_CONTINUOUS"
                if action == WORKFLOW_ACTION_START_MEASURE_CONTINUOUS
                else "WORKFLOW_START_MEASURE"
            )
            expected_value = WORKFLOW_MEASURE_POSITION_READY
            timeout_s = 15.0
            priority = False
        elif action == WORKFLOW_ACTION_FINISH_RETURN_HOME:
            # Keep the wire log short and identical to the RA6M5 integration
            # checklist: ``TX WORKFLOW_FINISH seq=xx: A5 5A ...``.
            label = "WORKFLOW_FINISH"
            expected_value = WORKFLOW_RETURN_HOME_DONE
            timeout_s = 15.0
            priority = False
        else:
            label = "WORKFLOW_ABORT_HOLD"
            expected_value = WORKFLOW_ABORTED_HOLD
            timeout_s = 2.0
            priority = True
        return self._begin_unified_control(
            label=label,
            msg_type=MSG_WORKFLOW_CTRL,
            payload=bytes([action]),
            frame=frame,
            seq=seq,
            expected_event=STATUS_WORKFLOW,
            expected_value=expected_value,
            completion_timeout_s=timeout_s,
            priority=priority,
        )

    def send_workflow_start(self, continuous: bool = False) -> bool:
        if continuous:
            self.last_error = UNSUPPORTED_CONTINUOUS_WORKFLOW_MESSAGE
            return False
        return self.send_workflow_ctrl(WORKFLOW_ACTION_START_MEASURE)

    def send_workflow_finish(self) -> bool:
        return self.send_workflow_ctrl(WORKFLOW_ACTION_FINISH_RETURN_HOME)

    def send_workflow_abort(self) -> bool:
        return self.send_workflow_ctrl(WORKFLOW_ACTION_ABORT_HOLD)

    def _observe_control_status(self, status: Ra6Status) -> bool:
        with self._control_lock:
            pending = self._pending_control
            if status.protocol not in ("unified", "unified_error"):
                return True

            if status.protocol == "unified_error" or status.msg_type == MSG_ERROR:
                if pending is not None:
                    self._transport_events.append(
                        "异步故障中止待完成控制：{} seq={} {}".format(
                            pending.label,
                            pending.seq,
                            status.label,
                        )
                    )
                self._pending_control = None
                self._active_target_seq = None
                self._closing_target_seq = None
                self._reset_vision_mailbox_locked()
                self._active_workflow_seq = None
                self._measure_window_open = False
                self._continuous_safe_distance = False
                self._safe_sample_seqs.clear()
                self._safe_decision_received = False
                return True

            # Positive workflow states are meaningful only for this workflow
            # session.  FAULT_HOLD is accepted conservatively even when the
            # sequence cannot be associated, because stopping is always safe.
            if status.func == STATUS_WORKFLOW:
                if (
                    status.value != WORKFLOW_FAULT_HOLD
                    and (
                        self._active_workflow_seq is None
                        or status.seq != self._active_workflow_seq
                    )
                ):
                    self._transport_events.append(
                        "丢弃错误流程会话 SEQ 状态：{} seq={} active_seq={}".format(
                            status.label,
                            status.seq,
                            self._active_workflow_seq,
                        )
                    )
                    return False

                # Workflow completion states are not interchangeable merely
                # because their SEQ matches.  Bind every command phase to the
                # action that is actually in flight; otherwise a delayed or
                # malformed RETURN_HOME_DONE could, for example, complete a
                # fresh START_MEASURE session and make Qt unlock/reset the
                # wrong workflow.  FAULT_HOLD remains a global conservative
                # event and is deliberately handled outside this whitelist.
                workflow_control_values = {
                    WORKFLOW_START_ACCEPTED,
                    WORKFLOW_MEASURE_POSITION_READY,
                    WORKFLOW_RETURN_HOME_DONE,
                    WORKFLOW_ABORTED_HOLD,
                }
                pending_workflow_values = {
                    bytes([WORKFLOW_ACTION_START_MEASURE]): {
                        WORKFLOW_START_ACCEPTED,
                        WORKFLOW_MEASURE_POSITION_READY,
                    },
                    bytes([WORKFLOW_ACTION_START_MEASURE_CONTINUOUS]): {
                        WORKFLOW_START_ACCEPTED,
                        WORKFLOW_MEASURE_POSITION_READY,
                    },
                    bytes([WORKFLOW_ACTION_FINISH_RETURN_HOME]): {
                        WORKFLOW_RETURN_HOME_DONE,
                    },
                    bytes([WORKFLOW_ACTION_ABORT_HOLD]): {
                        WORKFLOW_ABORTED_HOLD,
                    },
                }
                is_matching_workflow_request = (
                    pending is not None
                    and pending.msg_type == MSG_WORKFLOW_CTRL
                    and status.seq == pending.seq
                )
                allowed_workflow_values = (
                    pending_workflow_values.get(pending.payload, set())
                    if is_matching_workflow_request and pending is not None
                    else set()
                )
                if (
                    status.value != WORKFLOW_FAULT_HOLD
                    and (
                        (
                            is_matching_workflow_request
                            and status.value not in allowed_workflow_values
                        )
                        or (
                            status.value in workflow_control_values
                            and status.value not in allowed_workflow_values
                        )
                    )
                ):
                    pending_label = pending.label if pending is not None else "none"
                    self._transport_events.append(
                        "丢弃不属于当前流程动作的状态：{} pending={} seq={}".format(
                            status.label,
                            pending_label,
                            status.seq,
                        )
                    )
                    return False

                is_pending_workflow_status = (
                    pending is not None
                    and pending.msg_type == MSG_WORKFLOW_CTRL
                    and status.seq == pending.seq
                )
                if (
                    is_pending_workflow_status
                    and not pending.acknowledged
                    and status.func != STATUS_COMMAND_ACK
                ):
                    self._transport_events.append(
                        "协议时序错误：{} seq={} 在 COMMAND_ACK 前收到 {}，状态未交给业务层".format(
                            pending.label,
                            pending.seq,
                            status.label,
                        )
                    )
                    return False
                if (
                    status.value == WORKFLOW_DISTANCE_SAFE_LATCHED
                    and not self._safe_decision_received
                ):
                    self._transport_events.append(
                        "丢弃未收到本轮 SAFE_DISTANCE_OK 的距离锁存状态：seq={}".format(
                            status.seq
                        )
                    )
                    return False
                if (
                    status.error_code == ERROR_NONE
                    and status.value <= self._workflow_phase
                ):
                    self._transport_events.append(
                        "丢弃重复或倒序流程状态：{} phase={}".format(
                            status.label,
                            self._workflow_phase,
                        )
                    )
                    return False
                if status.error_code == ERROR_NONE:
                    self._workflow_phase = status.value

            # SAFE/UNSAFE must answer a sample sent in the current measurement
            # window.  Clearing this list on every new workflow prevents a
            # delayed response from an earlier patient/session unlocking Qt.
            if status.func == STATUS_SAFE_DISTANCE:
                if status.seq not in self._safe_sample_seqs:
                    self._transport_events.append(
                        "丢弃非本轮测距样本响应：{} seq={}".format(
                            status.label,
                            status.seq,
                        )
                    )
                    return False
                if (
                    status.value == 0x01
                    and status.error_code == ERROR_NONE
                    and not self._safe_sample_seqs[status.seq]
                ):
                    self._transport_events.append(
                        "丢弃RA6M5对 valid=0 测距样本返回的SAFE许可：seq={}".format(
                            status.seq
                        )
                    )
                    return False
                safe_now = status.value == 0x01 and status.error_code == ERROR_NONE
                self._safe_decision_received = safe_now
                self._measure_window_open = bool(
                    safe_now and self._continuous_safe_distance
                )
                if self._measure_window_open:
                    # Keep the most recent acknowledged sample so a later
                    # watchdog timeout can be tied to this session/SEQ.
                    self._safe_sample_seqs = {status.seq: True}
                else:
                    self._safe_sample_seqs.clear()

            if status.func in (
                STATUS_ALIGN_DONE,
                STATUS_OUTPUT,
                STATUS_VISION_STATE,
            ):
                allowed_seq = self._active_target_seq
                if (
                    status.func == STATUS_OUTPUT
                    and status.value == 0x00
                    and self._closing_target_seq is not None
                ):
                    allowed_seq = self._closing_target_seq
                if allowed_seq is None or status.seq != allowed_seq:
                    self._transport_events.append(
                        "丢弃错误定靶会话 SEQ 状态：{} seq={} active_seq={}".format(
                            status.label,
                            status.seq,
                            self._active_target_seq,
                        )
                    )
                    return False
                if status.func == STATUS_OUTPUT and status.value == 0x00:
                    self._closing_target_seq = None

            if status.func == STATUS_WORKFLOW and status.value in (
                WORKFLOW_START_ACCEPTED,
                WORKFLOW_MEASURE_POSITION_READY,
            ):
                self._active_target_seq = None
                self._closing_target_seq = None
                self._reset_vision_mailbox_locked()

            if status.func == STATUS_WORKFLOW and status.value == WORKFLOW_MEASURE_POSITION_READY:
                self._measure_window_open = True
                self._safe_sample_seqs.clear()
                self._safe_decision_received = False

            if status.func == STATUS_WORKFLOW and status.value in (
                WORKFLOW_RETREAT_DONE_WAIT_RESTART,
                WORKFLOW_RETURN_HOME_DONE,
                WORKFLOW_ABORTED_HOLD,
                WORKFLOW_FAULT_HOLD,
                WORKFLOW_IDLE,
            ):
                self._measure_window_open = False
                self._continuous_safe_distance = False
                self._safe_sample_seqs.clear()

            if status.func == STATUS_WORKFLOW and status.value == WORKFLOW_DISTANCE_SAFE_LATCHED:
                self._measure_window_open = bool(self._continuous_safe_distance)
                if not self._measure_window_open:
                    self._safe_sample_seqs.clear()

            if status.func == STATUS_WORKFLOW and status.value in (
                WORKFLOW_RETREAT_DONE_WAIT_RESTART,
                WORKFLOW_FAULT_HOLD,
            ):
                if pending is not None:
                    self._transport_events.append(
                        "全局流程状态中止待完成控制：{} seq={} {}".format(
                            pending.label,
                            pending.seq,
                            status.label,
                        )
                    )
                self._pending_control = None
                pending = None
                self._active_target_seq = None
                self._closing_target_seq = None
                self._reset_vision_mailbox_locked()

            control_status_events = (
                STATUS_COMMAND_ACK,
                STATUS_CAPTURE_POINT_READY,
                STATUS_CAPTURE_HOME_READY,
                STATUS_SELECTED_VIEW_READY,
                STATUS_TARGET_CTRL,
                STATUS_TARGET_READY,
            )
            if pending is None:
                if status.func in control_status_events:
                    self._transport_events.append(
                        "丢弃无待处理请求的陈旧控制状态：{} seq={}".format(
                            status.label,
                            status.seq,
                        )
                    )
                    return False
                if status.func == STATUS_WORKFLOW and status.value in (
                    WORKFLOW_RETREAT_DONE_WAIT_RESTART,
                    WORKFLOW_RETURN_HOME_DONE,
                    WORKFLOW_ABORTED_HOLD,
                    WORKFLOW_FAULT_HOLD,
                    WORKFLOW_IDLE,
                ):
                    self._active_workflow_seq = None
                return True

            allowed_pending_events = {STATUS_COMMAND_ACK}
            if pending.msg_type == MSG_WORKFLOW_CTRL:
                allowed_pending_events.add(STATUS_WORKFLOW)
            elif pending.msg_type == MSG_CAPTURE_CTRL:
                allowed_pending_events.add(pending.expected_event)
            elif pending.msg_type == MSG_TARGET_CTRL:
                allowed_pending_events.add(STATUS_TARGET_CTRL)
                if pending.payload == b"\x01":
                    allowed_pending_events.add(STATUS_TARGET_READY)

            belongs_to_pending = status.func in allowed_pending_events
            if (
                status.func in control_status_events
                and not belongs_to_pending
            ):
                self._transport_events.append(
                    "丢弃不属于当前控制类型的状态：{} pending={} seq={}".format(
                        status.label,
                        pending.label,
                        status.seq,
                    )
                )
                return False
            if status.seq != pending.seq and belongs_to_pending:
                if (
                    (
                        status.func == pending.expected_event
                        and status.value == pending.expected_value
                    )
                    or status.func in control_status_events
                    or (
                        pending.msg_type == MSG_WORKFLOW_CTRL
                        and status.func == STATUS_WORKFLOW
                    )
                ):
                    self._transport_events.append(
                        "丢弃控制完成状态错误 SEQ：{} got={} expected={}".format(
                            status.label,
                            status.seq,
                            pending.seq,
                        )
                    )
                    return False
                return True

            if status.func == STATUS_COMMAND_ACK:
                if status.value == 0x01 and status.error_code == ERROR_NONE:
                    if not pending.acknowledged:
                        pending.acknowledged = True
                        pending.acknowledged_at = time.monotonic()
                        self._transport_events.append(
                            "控制命令已接受：{} seq={}".format(pending.label, pending.seq)
                        )
                else:
                    self._transport_events.append(
                        "控制命令被拒绝：{} seq={} {}".format(
                            pending.label,
                            pending.seq,
                            status.label,
                        )
                    )
                    self._pending_control = None
                    if pending.msg_type == MSG_WORKFLOW_CTRL:
                        self._active_workflow_seq = None
                        self._measure_window_open = False
                        self._continuous_safe_distance = False
                        self._safe_sample_seqs.clear()
                        self._safe_decision_received = False
                return True

            if belongs_to_pending and not pending.acknowledged:
                self._transport_events.append(
                    "协议时序错误：{} seq={} 在 COMMAND_ACK 前收到 {}，状态未交给业务层".format(
                        pending.label,
                        pending.seq,
                        status.label,
                    )
                )
                return False

            if belongs_to_pending and status.error_code != ERROR_NONE:
                self._transport_events.append(
                    "控制命令失败：{} seq={} {}".format(pending.label, pending.seq, status.label)
                )
                self._pending_control = None
                if pending.msg_type == MSG_WORKFLOW_CTRL:
                    self._active_workflow_seq = None
                    self._measure_window_open = False
                    self._continuous_safe_distance = False
                    self._safe_sample_seqs.clear()
                    self._safe_decision_received = False
                return True

            if (
                pending.msg_type == MSG_TARGET_CTRL
                and pending.payload == b"\x01"
                and status.func == STATUS_TARGET_CTRL
                and status.value == 0x01
            ):
                pending.target_ctrl_on_seen = True

            if (
                pending.msg_type == MSG_TARGET_CTRL
                and pending.payload == b"\x01"
                and status.func == STATUS_TARGET_READY
                and not pending.target_ctrl_on_seen
            ):
                self._transport_events.append(
                    "协议时序错误：TARGET_READY seq={} 早于 TARGET_CTRL_ON".format(
                        pending.seq
                    )
                )
                return False

            if status.func == pending.expected_event and status.value == pending.expected_value:
                if pending.msg_type == MSG_TARGET_CTRL:
                    if pending.payload == b"\x01":
                        self._active_target_seq = pending.seq
                        self._closing_target_seq = None
                        self._reset_vision_mailbox_locked()
                    else:
                        self._closing_target_seq = self._active_target_seq
                        self._active_target_seq = None
                        self._reset_vision_mailbox_locked()
                elif pending.msg_type == MSG_WORKFLOW_CTRL:
                    if pending.payload and pending.payload[0] in WORKFLOW_START_ACTIONS:
                        self._active_workflow_seq = pending.seq
                        self._measure_window_open = True
                        self._continuous_safe_distance = (
                            pending.payload[0] == WORKFLOW_ACTION_START_MEASURE_CONTINUOUS
                        )
                        self._safe_decision_received = False
                    else:
                        self._active_workflow_seq = None
                        self._measure_window_open = False
                        self._continuous_safe_distance = False
                        self._safe_sample_seqs.clear()
                        self._safe_decision_received = False
                self._transport_events.append(
                    "控制动作完成：{} seq={} {}".format(
                        pending.label,
                        pending.seq,
                        status.label,
                    )
                )
                self._pending_control = None
            return True

    def service_control(self, now: Optional[float] = None) -> None:
        """Retry an unacknowledged control or report a terminal timeout."""
        if now is None:
            now = time.monotonic()
        retry: Optional[Tuple[str, bytes]] = None
        with self._control_lock:
            pending = self._pending_control
            if pending is None:
                return
            if not pending.acknowledged:
                if now - pending.last_sent_at < CONTROL_ACK_TIMEOUT_S:
                    return
                if pending.retries >= CONTROL_MAX_RETRIES:
                    self._transport_events.append(
                        "控制命令 ACK 超时：{} seq={}，已重发 {} 次".format(
                            pending.label,
                            pending.seq,
                            pending.retries,
                        )
                    )
                    self._pending_control = None
                    if pending.msg_type == MSG_WORKFLOW_CTRL:
                        self._active_workflow_seq = None
                        self._measure_window_open = False
                        self._continuous_safe_distance = False
                        self._safe_sample_seqs.clear()
                        self._safe_decision_received = False
                    return
                pending.retries += 1
                pending.last_sent_at = now
                retry = (
                    "{} RETRY {}/{} seq={}".format(
                        pending.label,
                        pending.retries,
                        CONTROL_MAX_RETRIES,
                        pending.seq,
                    ),
                    pending.frame,
                )
            else:
                started = pending.acknowledged_at or pending.first_sent_at
                if now - started >= pending.completion_timeout_s:
                    self._transport_events.append(
                        "控制动作完成超时：{} seq={}，请取消/保持后人工检查".format(
                            pending.label,
                            pending.seq,
                        )
                    )
                    self._pending_control = None
                    if pending.msg_type == MSG_WORKFLOW_CTRL:
                        self._active_workflow_seq = None
                        self._measure_window_open = False
                        self._continuous_safe_distance = False
                        self._safe_sample_seqs.clear()
                        self._safe_decision_received = False
        if retry is not None:
            try:
                self._write_frame(retry[0], retry[1])
            except Exception as exc:
                self.last_error = str(exc)

    def send_laser_output(self, enabled: bool = True) -> bool:
        """Legacy-only direct laser command; forbidden in unified mode."""
        if self.protocol == "unified":
            self.last_error = (
                "unified protocol forbids direct P801 output; wait for ALIGN_DONE and physical P000"
            )
            return False
        try:
            frame = pack_laser_output(enabled)
            self._write_frame("LASER_OUTPUT legacy", frame)
            self.last_error = ""
            return True
        except Exception as exc:
            self.last_error = str(exc)
            return False

    def send_target_error(
        self,
        target_x: int,
        target_y: int,
        frame_width: int,
        frame_height: int,
        valid: bool = True,
    ) -> bool:
        dcx, dcy = pixel_error_from_target(
            target_x,
            target_y,
            frame_width,
            frame_height,
        )
        return self.send_vision_error(dcx, dcy, valid=valid)

    def _read_available_statuses(self) -> List[Ra6Status]:
        try:
            with self._rx_lock:
                waiting = int(getattr(self._ser, "in_waiting", 0))
                if waiting <= 0:
                    self._record_io_success("RX")
                    return []
                data = self._ser.read(waiting)
                statuses = self._status_parser.feed(data)
            self._record_io_success("RX")
            print(f"RX RAW: {self._format_frame(data)}", flush=True)
            accepted_statuses = []
            for status in statuses:
                if not self._observe_control_status(status):
                    continue
                accepted_statuses.append(status)
                if status.protocol == "legacy":
                    detail = f"func=0x{status.func:02X} value=0x{status.value:02X}"
                else:
                    detail = (
                        f"seq={status.seq} event=0x{status.func:02X} "
                        f"value=0x{status.value:02X} error=0x{status.error_code:02X}"
                    )
                print(f"RX STATUS {status.label}: {detail}", flush=True)
            self.last_error = ""
            return accepted_statuses
        except Exception as exc:
            self.last_error = str(exc)
            self._record_io_failure("RX", exc)
            return []

    def _drain_receive_to_queue(self) -> None:
        statuses = self._read_available_statuses()
        if not statuses:
            return
        with self._rx_lock:
            self._received_statuses.extend(statuses)

    def poll_statuses(self) -> List[Ra6Status]:
        """Consume statuses collected by the background serial I/O worker."""
        self._drain_receive_to_queue()
        with self._rx_lock:
            statuses = list(self._received_statuses)
            self._received_statuses.clear()
        return statuses

    def send_moles(
        self,
        coordinates: List[Tuple[int, int]],
        frame_size: Optional[Tuple[int, int]] = None,
    ) -> bool:
        if not coordinates:
            return self.send_no_target()
        if frame_size is None:
            self.last_error = "frame_size is required by the vision error protocol"
            return False
        x, y = coordinates[0]
        width, height = frame_size
        return self.send_target_error(x, y, width, height)

    def send_no_target(self) -> bool:
        """Explicitly invalidate vision in unified mode."""
        if self.protocol == "unified":
            return self.send_vision_error(0, 0, valid=False)
        self.last_error = "legacy protocol cannot represent a no-target sample"
        return False

    def close(self) -> None:
        # Block every new write before stopping the background worker.  Taking
        # the same TX lock around serial.close() also waits for an already
        # active complete-frame write, so close can never cut it in half.
        self._closing.set()
        self._heartbeat_stop.set()
        with self._control_lock:
            self._active_target_seq = None
            self._reset_vision_mailbox_locked()
        if (
            self._heartbeat_thread is not None
            and self._heartbeat_thread.is_alive()
            and self._heartbeat_thread is not current_thread()
        ):
            self._heartbeat_thread.join(timeout=0.5)
        with self._tx_lock:
            with self._rx_lock:
                if self._ser and self._ser.is_open:
                    self._ser.close()
