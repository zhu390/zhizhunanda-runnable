"""Serial frame helpers for the Jetson <-> RA6M5 vision protocol."""

import struct
from dataclasses import dataclass
from typing import List, Optional, Tuple


JETSON_SOF = 0xFF
JETSON_EOF = 0xFE
JETSON_LEN_VISION_ERROR = 0x05
JETSON_FUNC_VISION_ERROR = 0x03
JETSON_VISION_ERROR_FRAME_SIZE = 9

TARGET_CTRL_SOF = 0xAA
TARGET_CTRL_EOF = 0xBB
TARGET_CTRL_FUNC = 0x01
TARGET_CTRL_ENABLE = 0x01
TARGET_CTRL_DISABLE = 0x00
TARGET_CTRL_FRAME_SIZE = 4
LASER_OUTPUT_FUNC = 0x03
LASER_OUTPUT_ENABLE = 0x01
LASER_OUTPUT_DISABLE = 0x00

RA6_SOF = 0xCC
RA6_EOF = 0xDD
RA6_STATUS_FRAME_SIZE = 4

RA6_TO_JETSON_READY = 0x01
RA6_TO_JETSON_ALIGN_DONE = 0x02
RA6_TO_JETSON_OUTPUT = 0x03
RA6_TO_JETSON_TARGET_CTRL = 0x04
RA6_TO_JETSON_ERROR = 0xFE

UNIFIED_SOF0 = 0xA5
UNIFIED_SOF1 = 0x5A
UNIFIED_VERSION = 0x01
UNIFIED_MAX_PAYLOAD = 32
UNIFIED_HEADER_SIZE = 6
UNIFIED_MIN_FRAME_SIZE = 8

MSG_HEARTBEAT = 0x01
MSG_TARGET_CTRL = 0x02
MSG_VISION_ERROR = 0x03
MSG_CAPTURE_CTRL = 0x04
MSG_SAFE_DISTANCE = 0x05
MSG_WORKFLOW_CTRL = 0x06
MSG_STATUS = 0x81
MSG_ERROR = 0xFE

STATUS_READY = 0x01
STATUS_ALIGN_DONE = 0x02
STATUS_OUTPUT = 0x03
STATUS_TARGET_CTRL = 0x04
STATUS_HEARTBEAT = 0x05
STATUS_SAFE_DISTANCE = 0x06
STATUS_VISION_STATE = 0x07
STATUS_CAPTURE_POINT_REACHED = 0x10
STATUS_CAPTURE_HOME_DONE = 0x11
STATUS_TARGET_PRESTART_REACHED = 0x12
STATUS_WORKFLOW = 0x20
STATUS_COMMAND_ACK = 0x21

# Canonical protocol-v5 names.  Keep the older aliases above so existing
# bring-up scripts do not break while the formal Qt path uses these names.
STATUS_TARGET_READY = STATUS_READY
STATUS_HEARTBEAT_ALIVE = STATUS_HEARTBEAT
STATUS_CAPTURE_POINT_READY = STATUS_CAPTURE_POINT_REACHED
STATUS_CAPTURE_HOME_READY = STATUS_CAPTURE_HOME_DONE
STATUS_SELECTED_VIEW_READY = STATUS_TARGET_PRESTART_REACHED

CAPTURE_ACTION_MOVE_TO_POINT = 0x01
CAPTURE_ACTION_DONE_HOME = 0x02
CAPTURE_ACTION_SELECT_POINT = 0x03
CAPTURE_ACTION_GOTO_VIEW = CAPTURE_ACTION_MOVE_TO_POINT
CAPTURE_ACTION_HOME = CAPTURE_ACTION_DONE_HOME
CAPTURE_ACTION_SELECT_VIEW = CAPTURE_ACTION_SELECT_POINT

TARGET_CTRL_STOP_HOLD = 0x00
TARGET_CTRL_START = 0x01
VISION_INVALID = 0x00
VISION_VALID = 0x01

WORKFLOW_ACTION_START_MEASURE = 0x01
WORKFLOW_ACTION_FINISH_RETURN_HOME = 0x02
WORKFLOW_ACTION_ABORT_HOLD = 0x03
WORKFLOW_ACTION_START_MEASURE_CONTINUOUS = 0x04

WORKFLOW_IDLE = 0x00
WORKFLOW_START_ACCEPTED = 0x01
WORKFLOW_MEASURE_POSITION_READY = 0x02
WORKFLOW_DISTANCE_SAFE_LATCHED = 0x03
WORKFLOW_RETREAT_DONE_WAIT_RESTART = 0x04
WORKFLOW_RETURN_HOME_DONE = 0x05
WORKFLOW_ABORTED_HOLD = 0x06
WORKFLOW_FAULT_HOLD = 0x07

ERROR_NONE = 0x00
ERROR_VERSION_MISMATCH = 0x01
ERROR_UNKNOWN_TYPE = 0x02
ERROR_INVALID_PARAM = 0x03
ERROR_POSE_INVALID = 0x04
ERROR_BUSY = 0x05
ERROR_MOTION_FAILED = 0x06
ERROR_HEARTBEAT_TIMEOUT = 0x07
ERROR_SAFETY_INPUT = 0x08
ERROR_SAFE_DISTANCE_TOO_CLOSE = 0x09
ERROR_VISION_LOST = 0x0A
ERROR_SOFT_RESET_FAILED = 0x0B
ERROR_INVALID_STATE = 0x0C
ERROR_SEQ_CONFLICT = 0x0D
ERROR_TARGET_GATE_DENIED = 0x0E
ERROR_MOTION_ABORTED = 0x0F
ERROR_SAFE_DISTANCE_TIMEOUT = 0x10

# Backward-compatible aliases used by older local scripts.
ERROR_UNKNOWN_MSG = ERROR_UNKNOWN_TYPE
ERROR_BAD_PAYLOAD = ERROR_INVALID_PARAM
ERROR_IK_FAILED = ERROR_MOTION_FAILED
ERROR_LIMIT_OR_ESTOP = ERROR_SAFETY_INPUT


class ProtocolError(ValueError):
    """Raised when a complete frame does not match the protocol."""


@dataclass(frozen=True)
class Ra6Status:
    func: int
    value: int
    error_code: int = 0
    protocol: str = "legacy"
    seq: Optional[int] = None
    msg_type: Optional[int] = None

    @property
    def label(self) -> str:
        return describe_ra6_status(
            self.func,
            self.value,
            error_code=self.error_code,
            protocol=self.protocol,
        )

    @property
    def ok(self) -> bool:
        return self.error_code == ERROR_NONE

    @property
    def is_command_ack(self) -> bool:
        return self.protocol == "unified" and self.func == STATUS_COMMAND_ACK

    @property
    def command_accepted(self) -> bool:
        return self.is_command_ack and self.value == 0x01 and self.ok


@dataclass(frozen=True)
class UnifiedFrame:
    msg_type: int
    seq: int
    payload: bytes

    @property
    def is_status(self) -> bool:
        return self.msg_type == MSG_STATUS

    @property
    def is_error(self) -> bool:
        return self.msg_type == MSG_ERROR


def _checksum(data: bytes) -> int:
    return sum(data) & 0xFF


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for byte in bytes(data):
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def _require_u8(name: str, value: int) -> int:
    value = int(value)
    if not 0 <= value <= 0xFF:
        raise ValueError(f"{name} must fit uint8_t: {value}")
    return value


def _require_i16(name: str, value: int) -> int:
    value = int(value)
    if not -32768 <= value <= 32767:
        raise ValueError(f"{name} must fit int16_t: {value}")
    return value


def _require_u16(name: str, value: int) -> int:
    value = int(round(float(value)))
    if not 0 <= value <= 0xFFFF:
        raise ValueError(f"{name} must fit uint16_t: {value}")
    return value


def _require_u32(name: str, value: int) -> int:
    value = int(value)
    if not 0 <= value <= 0xFFFFFFFF:
        raise ValueError(f"{name} must fit uint32_t: {value}")
    return value


def pixel_error_from_target(
    target_x: int,
    target_y: int,
    frame_width: int,
    frame_height: int,
) -> Tuple[int, int]:
    """Convert image coordinates into dcx/dcy relative to the image center."""
    if frame_width <= 0 or frame_height <= 0:
        raise ValueError("frame size must be positive")
    dcx = int(target_x) - int(frame_width) // 2
    dcy = int(target_y) - int(frame_height) // 2
    return dcx, dcy


def pack_vision_error(dcx: int, dcy: int) -> bytes:
    """Pack [FF][05][03][dcx_i16_le][dcy_i16_le][checksum][FE]."""
    dcx = _require_i16("dcx", dcx)
    dcy = _require_i16("dcy", dcy)
    data = struct.pack("<hh", dcx, dcy)
    body = bytes([JETSON_LEN_VISION_ERROR, JETSON_FUNC_VISION_ERROR]) + data
    return bytes([JETSON_SOF]) + body + bytes([_checksum(body), JETSON_EOF])


def build_unified_frame(msg_type: int, seq: int, payload: bytes = b"") -> bytes:
    """Pack [A5][5A][01][TYPE][SEQ][LEN][PAYLOAD][CRC16_LE]."""
    msg_type = _require_u8("msg_type", msg_type)
    seq = _require_u8("seq", seq)
    payload = bytes(payload)
    if len(payload) > UNIFIED_MAX_PAYLOAD:
        raise ValueError(f"payload too long: {len(payload)} > {UNIFIED_MAX_PAYLOAD}")
    body = bytes([UNIFIED_VERSION, msg_type, seq, len(payload)]) + payload
    crc = crc16_modbus(body)
    return bytes([UNIFIED_SOF0, UNIFIED_SOF1]) + body + struct.pack("<H", crc)


def pack_unified_heartbeat(seq: int, tick_ms: int) -> bytes:
    tick_ms = _require_u32("tick_ms", tick_ms)
    return build_unified_frame(MSG_HEARTBEAT, seq, struct.pack("<I", tick_ms))


def pack_unified_target_ctrl(seq: int, enabled: bool) -> bytes:
    return build_unified_frame(MSG_TARGET_CTRL, seq, bytes([0x01 if enabled else 0x00]))


def pack_unified_vision_error(seq: int, dcx: int, dcy: int, valid: bool = True) -> bytes:
    dcx = _require_i16("dcx", dcx)
    dcy = _require_i16("dcy", dcy)
    payload = struct.pack("<hhB", dcx, dcy, 0x01 if valid else 0x00)
    return build_unified_frame(MSG_VISION_ERROR, seq, payload)


def pack_unified_capture_ctrl(seq: int, action: int, point_id: int) -> bytes:
    action = _require_u8("action", action)
    point_id = _require_u8("point_id", point_id)
    if action in (CAPTURE_ACTION_MOVE_TO_POINT, CAPTURE_ACTION_SELECT_POINT):
        if point_id not in (1, 2, 3):
            raise ValueError("point_id must be 1/2/3 for capture action 0x{:02X}".format(action))
    elif action == CAPTURE_ACTION_DONE_HOME:
        if point_id != 0:
            raise ValueError("point_id must be 0 for capture done/home")
    else:
        raise ValueError("unknown capture action: 0x{:02X}".format(action))
    return build_unified_frame(MSG_CAPTURE_CTRL, seq, bytes([action, point_id]))


def pack_unified_safe_distance(seq: int, distance_mm: int, valid: bool = True) -> bytes:
    distance_mm = _require_u16("distance_mm", distance_mm)
    payload = struct.pack("<HB", distance_mm, 0x01 if valid else 0x00)
    return build_unified_frame(MSG_SAFE_DISTANCE, seq, payload)


def pack_unified_workflow_ctrl(seq: int, action: int) -> bytes:
    action = _require_u8("action", action)
    if action not in (
        WORKFLOW_ACTION_START_MEASURE,
        WORKFLOW_ACTION_START_MEASURE_CONTINUOUS,
        WORKFLOW_ACTION_FINISH_RETURN_HOME,
        WORKFLOW_ACTION_ABORT_HOLD,
    ):
        raise ValueError("unknown workflow action: 0x{:02X}".format(action))
    return build_unified_frame(MSG_WORKFLOW_CTRL, seq, bytes([action]))


def pack_target_ctrl(enabled: bool) -> bytes:
    """Pack [AA][01][01/00][BB] target state-machine control frame."""
    value = TARGET_CTRL_ENABLE if enabled else TARGET_CTRL_DISABLE
    return bytes([TARGET_CTRL_SOF, TARGET_CTRL_FUNC, value, TARGET_CTRL_EOF])


def pack_laser_output(enabled: bool) -> bytes:
    """Pack [AA][03][01/00][BB] laser output control frame."""
    value = LASER_OUTPUT_ENABLE if enabled else LASER_OUTPUT_DISABLE
    return bytes([TARGET_CTRL_SOF, LASER_OUTPUT_FUNC, value, TARGET_CTRL_EOF])


def parse_target_ctrl_frame(frame: bytes) -> bool:
    raw = bytes(frame)
    if len(raw) != TARGET_CTRL_FRAME_SIZE:
        raise ProtocolError(f"target ctrl frame size must be 4 bytes: {len(raw)}")
    if raw[0] != TARGET_CTRL_SOF:
        raise ProtocolError(f"bad target ctrl SOF: 0x{raw[0]:02X}")
    if raw[1] != TARGET_CTRL_FUNC:
        raise ProtocolError(f"bad target ctrl FUNC: 0x{raw[1]:02X}")
    if raw[2] not in (TARGET_CTRL_DISABLE, TARGET_CTRL_ENABLE):
        raise ProtocolError(f"bad target ctrl VALUE: 0x{raw[2]:02X}")
    if raw[3] != TARGET_CTRL_EOF:
        raise ProtocolError(f"bad target ctrl EOF: 0x{raw[3]:02X}")
    return raw[2] == TARGET_CTRL_ENABLE


def unpack_vision_error(frame: bytes) -> Tuple[int, int]:
    raw = bytes(frame)
    if len(raw) != JETSON_VISION_ERROR_FRAME_SIZE:
        raise ProtocolError(f"vision frame size must be 9 bytes: {len(raw)}")
    if raw[0] != JETSON_SOF:
        raise ProtocolError(f"bad vision SOF: 0x{raw[0]:02X}")
    if raw[1] != JETSON_LEN_VISION_ERROR:
        raise ProtocolError(f"bad vision LEN: 0x{raw[1]:02X}")
    if raw[2] != JETSON_FUNC_VISION_ERROR:
        raise ProtocolError(f"bad vision FUNC: 0x{raw[2]:02X}")
    if raw[-1] != JETSON_EOF:
        raise ProtocolError(f"bad vision EOF: 0x{raw[-1]:02X}")
    if raw[-2] != _checksum(raw[1:-2]):
        raise ProtocolError(
            f"bad vision checksum: got 0x{raw[-2]:02X}, "
            f"expected 0x{_checksum(raw[1:-2]):02X}"
        )
    return struct.unpack("<hh", raw[3:7])


def pack_ra6_status(func: int, value: int) -> bytes:
    func = _require_u8("func", func)
    value = _require_u8("value", value)
    return bytes([RA6_SOF, func, value, RA6_EOF])


def parse_ra6_status_frame(frame: bytes) -> Ra6Status:
    raw = bytes(frame)
    if len(raw) != RA6_STATUS_FRAME_SIZE:
        raise ProtocolError(f"status frame size must be 4 bytes: {len(raw)}")
    if raw[0] != RA6_SOF:
        raise ProtocolError(f"bad status SOF: 0x{raw[0]:02X}")
    if raw[3] != RA6_EOF:
        raise ProtocolError(f"bad status EOF: 0x{raw[3]:02X}")
    return Ra6Status(func=raw[1], value=raw[2])


def parse_unified_frame(frame: bytes) -> UnifiedFrame:
    raw = bytes(frame)
    if len(raw) < UNIFIED_MIN_FRAME_SIZE:
        raise ProtocolError(f"unified frame too short: {len(raw)}")
    if raw[0] != UNIFIED_SOF0 or raw[1] != UNIFIED_SOF1:
        raise ProtocolError("bad unified SOF")
    if raw[2] != UNIFIED_VERSION:
        raise ProtocolError(f"bad unified version: 0x{raw[2]:02X}")
    payload_len = raw[5]
    if payload_len > UNIFIED_MAX_PAYLOAD:
        raise ProtocolError(f"unified payload too long: {payload_len}")
    expected_size = UNIFIED_MIN_FRAME_SIZE + payload_len
    if len(raw) != expected_size:
        raise ProtocolError(f"unified frame size must be {expected_size} bytes: {len(raw)}")
    body = raw[2:-2]
    got_crc = struct.unpack("<H", raw[-2:])[0]
    expected_crc = crc16_modbus(body)
    if got_crc != expected_crc:
        raise ProtocolError(
            f"bad unified crc: got 0x{got_crc:04X}, expected 0x{expected_crc:04X}"
        )
    return UnifiedFrame(msg_type=raw[3], seq=raw[4], payload=raw[6:-2])


def unified_frame_to_status(frame: UnifiedFrame) -> Optional[Ra6Status]:
    if frame.msg_type == MSG_STATUS:
        if len(frame.payload) != 3:
            raise ProtocolError(f"STATUS payload size must be 3: {len(frame.payload)}")
        event, value, error_code = frame.payload
        valid_values = {
            STATUS_TARGET_READY: (0x01,),
            STATUS_ALIGN_DONE: (0x01,),
            STATUS_OUTPUT: (0x00, 0x01),
            STATUS_TARGET_CTRL: (0x00, 0x01),
            STATUS_HEARTBEAT_ALIVE: (0x01,),
            STATUS_SAFE_DISTANCE: (0x00, 0x01),
            STATUS_VISION_STATE: (0x00, 0x01),
            STATUS_CAPTURE_POINT_READY: (0x01, 0x02, 0x03),
            STATUS_CAPTURE_HOME_READY: (0x01,),
            STATUS_SELECTED_VIEW_READY: (0x01, 0x02, 0x03),
            STATUS_WORKFLOW: tuple(range(WORKFLOW_IDLE, WORKFLOW_FAULT_HOLD + 1)),
            STATUS_COMMAND_ACK: (0x00, 0x01),
        }
        if event not in valid_values:
            raise ProtocolError(f"unknown STATUS event: 0x{event:02X}")
        if value not in valid_values[event]:
            raise ProtocolError(
                f"invalid STATUS value 0x{value:02X} for event 0x{event:02X}"
            )
        if not ERROR_NONE <= error_code <= ERROR_SAFE_DISTANCE_TIMEOUT:
            raise ProtocolError(f"unknown STATUS error code: 0x{error_code:02X}")
        return Ra6Status(
            func=event,
            value=value,
            error_code=error_code,
            protocol="unified",
            seq=frame.seq,
            msg_type=frame.msg_type,
        )
    if frame.msg_type == MSG_ERROR:
        if len(frame.payload) != 1:
            raise ProtocolError(f"ERROR payload size must be 1: {len(frame.payload)}")
        error_code = frame.payload[0]
        if not ERROR_NONE <= error_code <= ERROR_MOTION_ABORTED:
            raise ProtocolError(f"unknown ERROR code: 0x{error_code:02X}")
        return Ra6Status(
            func=RA6_TO_JETSON_ERROR,
            value=error_code,
            error_code=error_code,
            protocol="unified_error",
            seq=frame.seq,
            msg_type=frame.msg_type,
        )
    return None


def describe_ra6_status(
    func: int,
    value: int,
    error_code: int = 0,
    protocol: str = "legacy",
) -> str:
    func = int(func)
    value = int(value)
    error_code = int(error_code)
    if protocol in ("unified", "unified_error"):
        if protocol == "unified_error":
            return "ERROR_{}".format(describe_unified_error_code(error_code))
        return describe_unified_status_event(func, value, error_code)
    if func == RA6_TO_JETSON_READY and value == 0x01:
        return "READY"
    if func == RA6_TO_JETSON_ALIGN_DONE and value == 0x01:
        return "ALIGN_DONE"
    if func == RA6_TO_JETSON_OUTPUT and value == 0x01:
        return "OUTPUT_ON"
    if func == RA6_TO_JETSON_OUTPUT and value == 0x00:
        return "OUTPUT_OFF"
    if func == RA6_TO_JETSON_TARGET_CTRL and value == 0x01:
        return "TARGET_CTRL_ON"
    if func == RA6_TO_JETSON_TARGET_CTRL and value == 0x00:
        return "TARGET_CTRL_OFF"
    if func == RA6_TO_JETSON_ERROR and value == 0x01:
        return "ERROR"
    return f"UNKNOWN(func=0x{func:02X}, value=0x{value:02X})"


def describe_unified_status_event(event: int, value: int, error_code: int = 0) -> str:
    event = int(event)
    value = int(value)
    error_code = int(error_code)
    suffix = "" if error_code == ERROR_NONE else "_ERR_{}".format(
        describe_unified_error_code(error_code)
    )
    if event == STATUS_TARGET_READY and value == 0x01:
        return "TARGET_READY" + suffix
    if event == STATUS_ALIGN_DONE and value == 0x01:
        return "ALIGN_DONE" + suffix
    if event == STATUS_OUTPUT and value == 0x01:
        return "OUTPUT_ON" + suffix
    if event == STATUS_OUTPUT and value == 0x00:
        return "OUTPUT_OFF" + suffix
    if event == STATUS_TARGET_CTRL and value == 0x01:
        return "TARGET_CTRL_ON" + suffix
    if event == STATUS_TARGET_CTRL and value == 0x00:
        return "TARGET_CTRL_OFF" + suffix
    if event == STATUS_HEARTBEAT and value == 0x01:
        return "HEARTBEAT_ONLINE" + suffix
    if event == STATUS_SAFE_DISTANCE and value == 0x01:
        return "SAFE_DISTANCE_OK" + suffix
    if event == STATUS_SAFE_DISTANCE and value == 0x00:
        return "SAFE_DISTANCE_UNSAFE" + suffix
    if event == STATUS_VISION_STATE and value == 0x01:
        return "VISION_VALID" + suffix
    if event == STATUS_VISION_STATE and value == 0x00:
        return "VISION_LOST" + suffix
    if event == STATUS_CAPTURE_POINT_READY and value in (1, 2, 3):
        return "CAPTURE_POINT_{}_READY".format(value) + suffix
    if event == STATUS_CAPTURE_HOME_READY and value == 0x01:
        return "CAPTURE_HOME_READY" + suffix
    if event == STATUS_SELECTED_VIEW_READY and value in (1, 2, 3):
        return "SELECTED_VIEW_{}_READY".format(value) + suffix
    if event == STATUS_WORKFLOW:
        labels = {
            WORKFLOW_IDLE: "WORKFLOW_IDLE",
            WORKFLOW_START_ACCEPTED: "WORKFLOW_START_ACCEPTED",
            WORKFLOW_MEASURE_POSITION_READY: "MEASURE_POSITION_READY",
            WORKFLOW_DISTANCE_SAFE_LATCHED: "DISTANCE_SAFE_LATCHED",
            WORKFLOW_RETREAT_DONE_WAIT_RESTART: "RETREAT_DONE_WAIT_RESTART",
            WORKFLOW_RETURN_HOME_DONE: "RETURN_HOME_DONE",
            WORKFLOW_ABORTED_HOLD: "ABORTED_HOLD",
            WORKFLOW_FAULT_HOLD: "FAULT_HOLD",
        }
        if value in labels:
            return labels[value] + suffix
    if event == STATUS_COMMAND_ACK and value == 0x01:
        return "COMMAND_ACK_ACCEPTED" + suffix
    if event == STATUS_COMMAND_ACK and value == 0x00:
        return "COMMAND_ACK_REJECTED" + suffix
    return "UNKNOWN_STATUS(event=0x{:02X}, value=0x{:02X}, error=0x{:02X})".format(
        event,
        value,
        error_code,
    )


def describe_unified_error_code(error_code: int) -> str:
    error_code = int(error_code)
    labels = {
        ERROR_NONE: "NONE",
        ERROR_VERSION_MISMATCH: "VERSION_MISMATCH",
        ERROR_UNKNOWN_TYPE: "UNKNOWN_TYPE",
        ERROR_INVALID_PARAM: "INVALID_PARAM",
        ERROR_POSE_INVALID: "POSE_INVALID",
        ERROR_BUSY: "BUSY",
        ERROR_MOTION_FAILED: "MOTION_FAILED",
        ERROR_HEARTBEAT_TIMEOUT: "HEARTBEAT_TIMEOUT",
        ERROR_SAFETY_INPUT: "SAFETY_INPUT",
        ERROR_SAFE_DISTANCE_TOO_CLOSE: "SAFE_DISTANCE_TOO_CLOSE",
        ERROR_VISION_LOST: "VISION_LOST",
        ERROR_SOFT_RESET_FAILED: "SOFT_RESET_FAILED",
        ERROR_INVALID_STATE: "INVALID_STATE",
        ERROR_SEQ_CONFLICT: "SEQ_CONFLICT",
        ERROR_TARGET_GATE_DENIED: "TARGET_GATE_DENIED",
        ERROR_MOTION_ABORTED: "MOTION_ABORTED",
        ERROR_SAFE_DISTANCE_TIMEOUT: "SAFE_DISTANCE_TIMEOUT",
    }
    return labels.get(error_code, "0x{:02X}".format(error_code))


class Ra6StatusParser:
    """Incremental parser for sticky/split unified frames.

    Legacy ``CC ... DD`` parsing is opt-in.  The formal Jetson competition
    path must not accept legacy status frames because doing so can advance the
    workflow without VERSION/LEN/CRC/SEQ validation.
    """

    def __init__(self, allow_legacy: bool = False) -> None:
        self._buffer = bytearray()
        self.allow_legacy = bool(allow_legacy)

    def feed(self, data: bytes) -> List[Ra6Status]:
        statuses: List[Ra6Status] = []
        self._buffer.extend(bytes(data))
        while self._buffer:
            first = self._buffer[0]
            if self.allow_legacy and first == RA6_SOF:
                if len(self._buffer) < RA6_STATUS_FRAME_SIZE:
                    break
                frame = bytes(self._buffer[:RA6_STATUS_FRAME_SIZE])
                try:
                    status = parse_ra6_status_frame(frame)
                except ProtocolError:
                    del self._buffer[0]
                    continue
                del self._buffer[:RA6_STATUS_FRAME_SIZE]
                statuses.append(status)
                continue

            if first != UNIFIED_SOF0:
                del self._buffer[0]
                continue
            if len(self._buffer) < 2:
                break
            if self._buffer[1] != UNIFIED_SOF1:
                del self._buffer[0]
                continue
            if len(self._buffer) < UNIFIED_HEADER_SIZE:
                break
            if self._buffer[2] != UNIFIED_VERSION:
                del self._buffer[0]
                continue
            payload_len = self._buffer[5]
            if payload_len > UNIFIED_MAX_PAYLOAD:
                del self._buffer[0]
                continue
            frame_size = UNIFIED_MIN_FRAME_SIZE + payload_len
            if len(self._buffer) < frame_size:
                break
            frame = bytes(self._buffer[:frame_size])
            try:
                parsed = parse_unified_frame(frame)
            except ProtocolError:
                # Keep scanning from the next byte.  Deleting the whole
                # candidate could swallow a valid A5 5A frame following a
                # corrupted LEN/CRC field.
                del self._buffer[0]
                continue
            del self._buffer[:frame_size]
            try:
                status = unified_frame_to_status(parsed)
            except ProtocolError:
                # The outer frame was CRC-valid, so a bad event/value belongs
                # to this complete frame.  Do not scan its payload as though
                # it were a nested serial frame.
                continue
            if status is not None:
                statuses.append(status)
        return statuses

    def reset(self) -> None:
        self._buffer.clear()
