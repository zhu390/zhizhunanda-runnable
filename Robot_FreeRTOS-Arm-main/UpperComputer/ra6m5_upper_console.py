from __future__ import annotations

import argparse
from collections.abc import Callable, Sequence
import datetime as dt
import queue
import threading
import time
from dataclasses import dataclass
from pathlib import Path

import serial
from serial.tools import list_ports
from PySide6.QtCore import QSettings, QTimer
from PySide6.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QTabWidget,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

APP_TITLE = "RA6M5 Robot Serial Console"
DEFAULT_BAUD = 115200
ARM_HISTORY_LIMIT = 15
SETTINGS_ORG = "RA6M5Robot"
SETTINGS_APP = "UpperConsole"
DEFAULT_DEMO_SEQUENCE = "30,-20:5;15,-10:5;8,-5:5;0,0:0"
DEFAULT_DEMO_PERIOD_MS = "200"
DEFAULT_WORKFLOW_SAFE_DISTANCE_MM = "160"
DEFAULT_WORKFLOW_VISION_SEQUENCE = "20,-15:2;8,-6:2;0,0:0"
DEFAULT_SAFE_DISTANCE_SEQUENCE = "130:8;85:12;115:8"
DEFAULT_CAPTURE_RECIPES = {
    "左视图": "# 左视图\nsoft_reset\nauto 45 -130 -15\nabs_rotate 0 85\nread_all",
    "正视图": "# 正视图\nsoft_reset\nauto 0 -130 -15\nabs_rotate 0 90\nread_all",
    "右视图": "# 右视图\nsoft_reset\nauto -45 -130 -15\nabs_rotate 0 105\nread_all",
}
CAPTURE_RECIPE_KEYS = {"左视图": "left", "正视图": "front", "右视图": "right"}
JETSON_SOF = 0xFF
JETSON_EOF = 0xFE
JETSON_LEN = 0x05
JETSON_FUNC_VISION_ERROR = 0x03
JETSON_CTRL_SOF = 0xAA
JETSON_CTRL_EOF = 0xBB
JETSON_FUNC_TARGET_CTRL = 0x01
JETSON_PROTOCOL_OLD = "old"
JETSON_PROTOCOL_NEW = "new"
JETSON_UNIFIED_SOF0 = 0xA5
JETSON_UNIFIED_SOF1 = 0x5A
JETSON_UNIFIED_VERSION = 0x01
JETSON_MSG_HEARTBEAT = 0x01
JETSON_MSG_TARGET_CTRL = 0x02
JETSON_MSG_VISION_ERROR = 0x03
JETSON_MSG_CAPTURE_CTRL = 0x04
JETSON_MSG_SAFE_DISTANCE = 0x05
JETSON_MSG_WORKFLOW_CTRL = 0x06
JETSON_MSG_STATUS = 0x81
JETSON_MSG_ERROR = 0xFE
EVENT_READY = 0x01
EVENT_ALIGN_DONE = 0x02
EVENT_OUTPUT = 0x03
EVENT_TARGET_CTRL = 0x04
EVENT_SAFE_DISTANCE = 0x06
EVENT_VISION_STATE = 0x07
EVENT_CAPTURE_POINT = 0x10
EVENT_CAPTURE_HOME = 0x11
EVENT_SELECTED_VIEW = 0x12
EVENT_WORKFLOW = 0x20
EVENT_COMMAND_ACK = 0x21
WORKFLOW_START = 0x01
WORKFLOW_FINISH = 0x02
WORKFLOW_ABORT = 0x03
WORKFLOW_START_ACCEPTED = 0x01
WORKFLOW_MEASURE_READY = 0x02
WORKFLOW_SAFE_LATCHED = 0x03
WORKFLOW_RETREAT_WAIT_RESTART = 0x04
WORKFLOW_RETURN_HOME_DONE = 0x05
WORKFLOW_ABORTED_HOLD = 0x06
WORKFLOW_FAULT_HOLD = 0x07
WORKFLOW_RETREAT_STEP_READY = 0x08
RA6_SOF = 0xCC
RA6_EOF = 0xDD
STATUS_NAMES = {
    (0x01, 0x01): "READY",
    (0x02, 0x01): "ALIGN_DONE",
    (0x03, 0x01): "OUTPUT_ON",
    (0x03, 0x00): "OUTPUT_OFF",
    (0x04, 0x01): "TARGET_CTRL_ON",
    (0x04, 0x00): "TARGET_CTRL_OFF",
    (0x06, 0x01): "SAFE_DISTANCE_OK",
    (0x06, 0x00): "SAFE_DISTANCE_TOO_CLOSE",
    (0x10, 0x01): "CAPTURE_POINT_LEFT",
    (0x10, 0x02): "CAPTURE_POINT_FRONT",
    (0x10, 0x03): "CAPTURE_POINT_RIGHT",
    (0x11, 0x01): "CAPTURE_DONE_HOME",
    (0x12, 0x00): "TARGET_PRESTART_CURRENT",
    (0x12, 0x01): "TARGET_PRESTART_LEFT",
    (0x12, 0x02): "TARGET_PRESTART_FRONT",
    (0x12, 0x03): "TARGET_PRESTART_RIGHT",
    (EVENT_WORKFLOW, WORKFLOW_START_ACCEPTED): "WORKFLOW_START_ACCEPTED",
    (EVENT_WORKFLOW, WORKFLOW_MEASURE_READY): "WORKFLOW_MEASURE_READY",
    (EVENT_WORKFLOW, WORKFLOW_SAFE_LATCHED): "WORKFLOW_SAFE_LATCHED",
    (EVENT_WORKFLOW, WORKFLOW_RETREAT_WAIT_RESTART): "WORKFLOW_RETREAT_WAIT_RESTART",
    (EVENT_WORKFLOW, WORKFLOW_RETURN_HOME_DONE): "WORKFLOW_RETURN_HOME_DONE",
    (EVENT_WORKFLOW, WORKFLOW_ABORTED_HOLD): "WORKFLOW_ABORTED_HOLD",
    (EVENT_WORKFLOW, WORKFLOW_FAULT_HOLD): "WORKFLOW_FAULT_HOLD",
    (EVENT_WORKFLOW, WORKFLOW_RETREAT_STEP_READY): "WORKFLOW_RETREAT_STEP_READY",
    (EVENT_COMMAND_ACK, 0x00): "COMMAND_ACK",
    (EVENT_COMMAND_ACK, 0x01): "COMMAND_ACK",
    (0xFE, 0x03): "INVALID_PARAM",
    (0xFE, 0x05): "BUSY",
    (0xFE, 0x07): "HEARTBEAT_TIMEOUT",
    (0xFE, 0x08): "SAFETY_ERROR",
    (0xFE, 0x01): "ERROR",
    (0xFE, 0x09): "SAFE_DISTANCE_TOO_CLOSE",
    (0xFE, 0x0A): "VISION_LOST",
    (0xFE, 0x0B): "SOFT_RESET_FAILED",
    (0xFE, 0x0C): "INVALID_STATE",
    (0xFE, 0x0D): "SEQ_CONFLICT",
    (0xFE, 0x0E): "TARGET_GATE_DENIED",
    (0xFE, 0x0F): "MOTION_ABORTED",
}

FORMAL_ERROR_MESSAGES = {
    0x01: "协议版本不匹配",
    0x02: "未知消息类型",
    0x03: "参数无效",
    0x04: "机械臂位姿无效",
    0x05: "机械臂忙",
    0x06: "机械臂运动失败",
    0x07: "Jetson 心跳超时",
    0x08: "安全保护触发",
    0x09: "安全距离不足",
    0x0A: "视觉数据丢失",
    0x0B: "软件复位失败",
    0x0C: "状态机状态不允许该命令",
    0x0D: "通信序号冲突",
    0x0E: "激光输出安全条件未满足",
    0x0F: "机械臂运动已中止",
}


@dataclass(frozen=True)
class Ra6Status:
    raw: bytes
    func: int
    value: int
    name: str
    seq: int | None = None
    event: int | None = None
    error: int = 0
    msg_type: int | None = None
    is_formal: bool = False


def bytes_to_hex(data: bytes) -> str:
    return " ".join(f"{byte:02X}" for byte in data)


def build_arm_command(command: str) -> bytes:
    return (command.strip() + "\r\n").encode("ascii", errors="replace")


def _int16_le(value: int) -> bytes:
    if value < -32768 or value > 32767:
        raise ValueError("vision error must fit int16")
    return int(value).to_bytes(2, "little", signed=True)


def build_vision_error_frame(dcx: int, dcy: int) -> bytes:
    payload = _int16_le(dcx) + _int16_le(dcy)
    checksum = (JETSON_LEN + JETSON_FUNC_VISION_ERROR + sum(payload)) & 0xFF
    return bytes([JETSON_SOF, JETSON_LEN, JETSON_FUNC_VISION_ERROR]) + payload + bytes([checksum, JETSON_EOF])


def build_target_control_frame(enable: bool) -> bytes:
    return bytes([JETSON_CTRL_SOF, JETSON_FUNC_TARGET_CTRL, 0x01 if enable else 0x00, JETSON_CTRL_EOF])


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def build_unified_frame(msg_type: int, seq: int, payload: bytes) -> bytes:
    if len(payload) > 255:
        raise ValueError("unified payload too large")
    body = bytes([JETSON_UNIFIED_VERSION, msg_type & 0xFF, seq & 0xFF, len(payload)]) + payload
    crc = crc16_modbus(body)
    return bytes([JETSON_UNIFIED_SOF0, JETSON_UNIFIED_SOF1]) + body + crc.to_bytes(2, "little")


def build_unified_heartbeat_frame(seq: int, tick_ms: int) -> bytes:
    return build_unified_frame(JETSON_MSG_HEARTBEAT, seq, int(tick_ms & 0xFFFFFFFF).to_bytes(4, "little"))


def build_unified_target_control_frame(enable: bool, seq: int) -> bytes:
    return build_unified_frame(JETSON_MSG_TARGET_CTRL, seq, bytes([0x01 if enable else 0x00]))


def build_unified_vision_error_frame(dcx: int, dcy: int, seq: int) -> bytes:
    return build_unified_frame(JETSON_MSG_VISION_ERROR, seq, _int16_le(dcx) + _int16_le(dcy) + b"\x01")


def build_unified_capture_control_frame(action: int, point_id: int, seq: int) -> bytes:
    if action not in (0x01, 0x02, 0x03, 0x04):
        raise ValueError("capture action must be 1, 2, 3, or 4")
    if action in (0x02, 0x04):
        if point_id != 0:
            raise ValueError("capture finish/current action requires point_id=0")
    elif point_id not in (1, 2, 3):
        raise ValueError("capture point_id must be 1, 2, or 3")
    return build_unified_frame(JETSON_MSG_CAPTURE_CTRL, seq, bytes([action & 0xFF, point_id & 0xFF]))


def build_unified_safe_distance_frame(distance_mm: int, valid: bool, seq: int) -> bytes:
    if distance_mm < 0 or distance_mm > 65535:
        raise ValueError("safe distance must fit uint16")
    payload = int(distance_mm).to_bytes(2, "little") + bytes([0x01 if valid else 0x00])
    return build_unified_frame(JETSON_MSG_SAFE_DISTANCE, seq, payload)


def build_unified_workflow_control_frame(action: int, seq: int) -> bytes:
    if action not in (WORKFLOW_START, WORKFLOW_FINISH, WORKFLOW_ABORT):
        raise ValueError("workflow action must be START, FINISH, or ABORT")
    return build_unified_frame(JETSON_MSG_WORKFLOW_CTRL, seq, bytes([action]))


def should_log_serial_tx(label: str) -> bool:
    return not (
        label.startswith("HEARTBEAT")
        or label.startswith("SAFE_DISTANCE")
        or label.startswith("FORMAL_VISION")
    )


def should_display_arm_line(line: str) -> bool:
    noisy_fragments = (
        "[JETSON_RX] heartbeat seq=",
        "[JETSON_RX] safe_distance=",
        "[TARGET] safe distance=",
    )
    return not any(fragment in line for fragment in noisy_fragments)


def _status_from_func_value(
    raw: bytes,
    func: int,
    value: int,
    *,
    seq: int | None = None,
    error: int = 0,
    msg_type: int | None = None,
    is_formal: bool = False,
) -> Ra6Status:
    return Ra6Status(
        raw=raw,
        func=func,
        value=value,
        name=STATUS_NAMES.get((func, value), "UNKNOWN"),
        seq=seq,
        event=func,
        error=error,
        msg_type=msg_type,
        is_formal=is_formal,
    )


def parse_ra6_status_stream(data: bytes) -> tuple[list[Ra6Status], bytes]:
    frames: list[Ra6Status] = []
    idx = 0
    consumed = 0
    while idx < len(data):
        if idx + 3 < len(data) and data[idx] == RA6_SOF:
            raw = data[idx:idx + 4]
            if raw[3] == RA6_EOF:
                frames.append(_status_from_func_value(raw, raw[1], raw[2]))
                idx += 4
                consumed = idx
                continue
        if idx + 7 < len(data) and data[idx] == JETSON_UNIFIED_SOF0 and data[idx + 1] == JETSON_UNIFIED_SOF1:
            if data[idx + 2] != JETSON_UNIFIED_VERSION:
                idx += 1
                consumed = idx
                continue
            payload_len = data[idx + 5]
            total_len = 8 + payload_len
            if idx + total_len > len(data):
                break
            raw = data[idx:idx + total_len]
            body = raw[2:6 + payload_len]
            rx_crc = int.from_bytes(raw[6 + payload_len:8 + payload_len], "little")
            if crc16_modbus(body) != rx_crc:
                idx += 1
                consumed = idx
                continue
            msg_type = raw[3]
            seq = raw[4]
            payload = raw[6:6 + payload_len]
            if msg_type == JETSON_MSG_STATUS and len(payload) == 3:
                frames.append(
                    _status_from_func_value(
                        raw,
                        payload[0],
                        payload[1],
                        seq=seq,
                        error=payload[2],
                        msg_type=msg_type,
                        is_formal=True,
                    )
                )
            elif msg_type == JETSON_MSG_ERROR and len(payload) >= 1:
                frames.append(
                    _status_from_func_value(
                        raw,
                        0xFE,
                        payload[0],
                        seq=seq,
                        error=payload[0],
                        msg_type=msg_type,
                        is_formal=True,
                    )
                )
            idx += total_len
            consumed = idx
            continue
        idx += 1
        consumed = idx
    return frames, data[consumed:]


def parse_ra6_status_frames(data: bytes) -> list[Ra6Status]:
    return parse_ra6_status_stream(data)[0]


def parse_vision_sequence(text: str) -> list[tuple[int, int, int]]:
    items: list[tuple[int, int, int]] = []
    for part in text.split(";"):
        part = part.strip()
        if not part:
            continue
        xy_text, _, count_text = part.partition(":")
        x_text, comma, y_text = xy_text.partition(",")
        if not comma:
            raise ValueError(f"bad vision sequence item: {part}")
        count = int(count_text) if count_text else 1
        if count < 0:
            raise ValueError("vision sequence count must be >= 0")
        items.append((int(x_text.strip()), int(y_text.strip()), count))
    return items


def parse_safe_distance_sequence(text: str) -> list[tuple[int, int]]:
    items: list[tuple[int, int]] = []
    for part in text.split(";"):
        part = part.strip()
        if not part:
            continue
        distance_text, _, count_text = part.partition(":")
        distance_mm = int(distance_text.strip())
        count = int(count_text) if count_text else 1
        if distance_mm < 0 or distance_mm > 65535:
            raise ValueError("safe distance must fit uint16")
        if count < 0:
            raise ValueError("safe distance sequence count must be >= 0")
        items.append((distance_mm, count))
    return items


def parse_formal_vision_sequence(text: str) -> tuple[tuple[int, int, float], ...]:
    items: list[tuple[int, int, float]] = []
    for part in text.split(";"):
        part = part.strip()
        if not part:
            continue
        xy_text, _, duration_text = part.partition(":")
        x_text, comma, y_text = xy_text.partition(",")
        if not comma:
            raise ValueError(f"视觉序列格式错误: {part}")
        duration_s = float(duration_text) if duration_text else 1.5
        if duration_s < 0:
            raise ValueError("视觉序列持续时间不能小于 0")
        dcx = int(x_text.strip())
        dcy = int(y_text.strip())
        _int16_le(dcx)
        _int16_le(dcy)
        items.append((dcx, dcy, duration_s))
    if not items:
        raise ValueError("视觉序列不能为空")
    return tuple(items)


class FormalWorkflowProtocolError(RuntimeError):
    pass


class FormalWorkflowStopped(RuntimeError):
    pass


def formal_error_message(error_code: int) -> str:
    return FORMAL_ERROR_MESSAGES.get(error_code, f"未知错误 0x{error_code:02X}")


class FormalStatusMailbox:
    def __init__(self) -> None:
        self._condition = threading.Condition()
        self._pending: list[Ra6Status] = []

    def clear(self) -> None:
        with self._condition:
            self._pending.clear()

    def publish(self, status: Ra6Status) -> None:
        if not status.is_formal:
            return
        with self._condition:
            self._pending.append(status)
            if len(self._pending) > 256:
                del self._pending[:-256]
            self._condition.notify_all()

    def wait(
        self,
        event: int,
        value: int | None,
        seq: int,
        timeout_s: float,
        stop_event: threading.Event,
    ) -> Ra6Status:
        deadline = time.monotonic() + timeout_s
        with self._condition:
            while True:
                if stop_event.is_set():
                    raise FormalWorkflowStopped("比赛流程已由用户终止")

                for index, status in enumerate(self._pending):
                    if status.msg_type == JETSON_MSG_ERROR:
                        self._pending.pop(index)
                        raise FormalWorkflowProtocolError(
                            f"RA6M5 异步错误 0x{status.error:02X}: {formal_error_message(status.error)}"
                        )
                    if status.error != 0 and status.seq in (0, seq):
                        self._pending.pop(index)
                        raise FormalWorkflowProtocolError(
                            f"RA6M5 拒绝/故障 0x{status.error:02X}: {formal_error_message(status.error)}"
                        )
                    if (
                        status.seq == seq
                        and status.event == event
                        and (value is None or status.value == value)
                    ):
                        return self._pending.pop(index)

                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    value_text = "*" if value is None else f"0x{value:02X}"
                    raise TimeoutError(
                        f"等待 RA6M5 状态超时: SEQ=0x{seq:02X}, EVENT=0x{event:02X}, VALUE={value_text}"
                    )
                self._condition.wait(min(0.1, remaining))


@dataclass(frozen=True)
class FormalWorkflowConfig:
    view_id: int
    safe_distance_mm: int
    vision_steps: tuple[tuple[int, int, float], ...]


class FormalWorkflowRunner:
    def __init__(
        self,
        *,
        send_control: Callable[[int, bytes, str, float], int],
        wait_status: Callable[[int, int | None, int, float], Ra6Status],
        send_distance: Callable[[int], None],
        start_vision: Callable[[Sequence[tuple[int, int, float]]], None],
        stop_vision: Callable[[], None],
        set_stage: Callable[[str], None],
        should_stop: Callable[[], bool],
        pause: Callable[[float], None],
    ) -> None:
        self.send_control = send_control
        self.wait_status = wait_status
        self.send_distance = send_distance
        self.start_vision = start_vision
        self.stop_vision = stop_vision
        self.set_stage = set_stage
        self.should_stop = should_stop
        self.pause = pause

    def _check_stop(self) -> None:
        if self.should_stop():
            raise FormalWorkflowStopped("比赛流程已由用户终止")

    def _wait(self, event: int, value: int | None, seq: int, timeout_s: float) -> Ra6Status:
        self._check_stop()
        return self.wait_status(event, value, seq, timeout_s)

    def run(self, config: FormalWorkflowConfig) -> None:
        self._check_stop()
        self.set_stage("正在启动比赛状态机")
        start_seq = self.send_control(
            JETSON_MSG_WORKFLOW_CTRL,
            bytes([WORKFLOW_START]),
            "启动比赛流程",
            5.0,
        )
        self._wait(EVENT_WORKFLOW, WORKFLOW_START_ACCEPTED, start_seq, 5.0)

        self.set_stage("机械臂正在回 HOME 并前往公共测距位")
        self._wait(EVENT_WORKFLOW, WORKFLOW_MEASURE_READY, start_seq, 65.0)

        self.set_stage("公共测距位已到达，正在确认安全距离")
        for _ in range(3):
            self._check_stop()
            self.send_distance(config.safe_distance_mm)
            self.pause(0.2)
        self._wait(EVENT_WORKFLOW, WORKFLOW_SAFE_LATCHED, start_seq, 5.0)

        self.set_stage("安全距离已确认，机械臂正在回 HOME")
        capture_seq = self.send_control(
            JETSON_MSG_CAPTURE_CTRL,
            b"\x02\x00",
            "拍摄流程回 HOME",
            5.0,
        )
        self._wait(EVENT_CAPTURE_HOME, 1, capture_seq, 50.0)

        for point_id, name in ((1, "左视图"), (2, "正视图"), (3, "右视图")):
            self.set_stage(f"机械臂正在前往{name}拍摄点")
            capture_seq = self.send_control(
                JETSON_MSG_CAPTURE_CTRL,
                bytes([0x01, point_id]),
                f"前往{name}拍摄点",
                5.0,
            )
            self._wait(EVENT_CAPTURE_POINT, point_id, capture_seq, 25.0)
            self.set_stage(f"{name}拍摄点已到达，正在模拟图像采集")
            self.pause(1.0)
            self.set_stage(f"{name}图像采集完成")

        self.set_stage("三视图采集完成，机械臂正在回 HOME")
        capture_seq = self.send_control(
            JETSON_MSG_CAPTURE_CTRL,
            b"\x02\x00",
            "三视图完成后回 HOME",
            5.0,
        )
        self._wait(EVENT_CAPTURE_HOME, 1, capture_seq, 50.0)

        view_name = {1: "左视图", 2: "正视图", 3: "右视图"}[config.view_id]
        self.set_stage(f"医生已选择{view_name}，机械臂正在前往定靶起点")
        select_seq = self.send_control(
            JETSON_MSG_CAPTURE_CTRL,
            bytes([0x03, config.view_id]),
            f"选择{view_name}",
            5.0,
        )
        self._wait(EVENT_SELECTED_VIEW, config.view_id, select_seq, 25.0)

        self.set_stage("定靶起点已到达，正在启动视觉定靶")
        target_seq = self.send_control(
            JETSON_MSG_TARGET_CTRL,
            b"\x01",
            "启动视觉定靶",
            5.0,
        )
        self._wait(EVENT_TARGET_CTRL, 1, target_seq, 5.0)
        self._wait(EVENT_READY, 1, target_seq, 5.0)

        self.set_stage("正在发送视觉误差并等待机械臂对准")
        self.start_vision(config.vision_steps)
        try:
            self._wait(EVENT_ALIGN_DONE, 1, target_seq, 30.0)
            self.set_stage("视觉对准完成，请按住 P000 开启激光")
            self._wait(EVENT_OUTPUT, 1, target_seq, 60.0)
            self.set_stage("激光已开启，请松开 P000")
            self._wait(EVENT_OUTPUT, 0, target_seq, 60.0)
        finally:
            self.stop_vision()

        self.set_stage("激光已关闭，机械臂正在回 HOME")
        finish_seq = self.send_control(
            JETSON_MSG_WORKFLOW_CTRL,
            bytes([WORKFLOW_FINISH]),
            "完成流程并回 HOME",
            5.0,
        )
        self._wait(EVENT_WORKFLOW, WORKFLOW_RETURN_HOME_DONE, finish_seq, 50.0)
        self.set_stage("流程完成，机械臂已回 HOME")


def protocol_self_test() -> None:
    assert build_vision_error_frame(-7, -50) == bytes.fromhex("FF 05 03 F9 FF CE FF CD FE")
    assert build_target_control_frame(True) == bytes.fromhex("AA 01 01 BB")
    assert build_target_control_frame(False) == bytes.fromhex("AA 01 00 BB")
    assert build_unified_heartbeat_frame(1, 1234) == bytes.fromhex("A5 5A 01 01 01 04 D2 04 00 00 19 AF")
    assert build_unified_target_control_frame(True, 2) == bytes.fromhex("A5 5A 01 02 02 01 01 79 E8")
    assert build_unified_vision_error_frame(-7, -50, 3) == bytes.fromhex("A5 5A 01 03 03 05 F9 FF CE FF 01 39 EF")
    assert build_unified_capture_control_frame(0x01, 1, 5) == build_unified_frame(0x04, 5, bytes([0x01, 0x01]))
    assert build_unified_capture_control_frame(0x02, 0, 6) == build_unified_frame(0x04, 6, bytes([0x02, 0x00]))
    assert build_unified_capture_control_frame(0x03, 3, 7) == build_unified_frame(0x04, 7, bytes([0x03, 0x03]))
    assert build_unified_capture_control_frame(0x04, 0, 8) == build_unified_frame(0x04, 8, bytes([0x04, 0x00]))
    assert build_unified_safe_distance_frame(120, True, 4) == build_unified_frame(0x05, 4, bytes.fromhex("78 00 01"))
    assert build_arm_command(" soft_reset ") == b"soft_reset\r\n"
    frames = parse_ra6_status_frames(bytes.fromhex("00 CC 04 01 DD 99 CC 03 00 DD"))
    assert [(item.func, item.value, item.name) for item in frames] == [
        (0x04, 0x01, "TARGET_CTRL_ON"),
        (0x03, 0x00, "OUTPUT_OFF"),
    ]
    unified = build_unified_frame(0x81, 7, bytes([0x02, 0x01, 0x00]))
    assert [(item.func, item.value, item.name) for item in parse_ra6_status_frames(unified)] == [
        (0x02, 0x01, "ALIGN_DONE"),
    ]
    safe_status = build_unified_frame(0x81, 8, bytes([0x06, 0x00, 0x00]))
    assert [(item.func, item.value, item.name) for item in parse_ra6_status_frames(safe_status)] == [
        (0x06, 0x00, "SAFE_DISTANCE_TOO_CLOSE"),
    ]
    safe_error = build_unified_frame(0xFE, 9, bytes([0x09]))
    assert [(item.func, item.value, item.name) for item in parse_ra6_status_frames(safe_error)] == [
        (0xFE, 0x09, "SAFE_DISTANCE_TOO_CLOSE"),
    ]
    capture_status = build_unified_frame(0x81, 10, bytes([0x10, 0x03, 0x00]))
    assert [(item.func, item.value, item.name) for item in parse_ra6_status_frames(capture_status)] == [
        (0x10, 0x03, "CAPTURE_POINT_RIGHT"),
    ]
    prestart_status = build_unified_frame(0x81, 11, bytes([0x12, 0x02, 0x00]))
    assert [(item.func, item.value, item.name) for item in parse_ra6_status_frames(prestart_status)] == [
        (0x12, 0x02, "TARGET_PRESTART_FRONT"),
    ]
    current_prestart_status = build_unified_frame(0x81, 12, bytes([0x12, 0x00, 0x00]))
    assert [(item.func, item.value, item.name) for item in parse_ra6_status_frames(current_prestart_status)] == [
        (0x12, 0x00, "TARGET_PRESTART_CURRENT"),
    ]
    bad_crc = bytearray(unified)
    bad_crc[-1] ^= 0xFF
    assert parse_ra6_status_frames(bytes(bad_crc)) == []
    assert parse_vision_sequence("15,-10:2;8,-5:1;0,0:0") == [(15, -10, 2), (8, -5, 1), (0, 0, 0)]
    assert parse_safe_distance_sequence("130:8;85:12;115:8") == [(130, 8), (85, 12), (115, 8)]
    assert parse_safe_distance_sequence("120") == [(120, 1)]
    assert should_log_serial_tx("HEARTBEAT") is False
    assert should_log_serial_tx("SAFE_DISTANCE") is False
    assert should_log_serial_tx("TARGET_CTRL_START_NEW") is True
    assert should_display_arm_line("[JETSON_RX] heartbeat seq=84") is False
    assert should_display_arm_line("[JETSON_RX] safe_distance=120 valid=1") is False
    assert should_display_arm_line("[TARGET] safe distance=120 valid=1 too_close=0") is False
    assert should_display_arm_line("[JETSON_RX] target_ctrl=1") is True


class SessionLogger:
    def __init__(self, log_dir: Path) -> None:
        log_dir.mkdir(parents=True, exist_ok=True)
        stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.path = log_dir / f"ra6m5_upper_console_{stamp}.log"
        self.lock = threading.Lock()

    def write(self, line: str) -> None:
        with self.lock:
            with self.path.open("a", encoding="utf-8") as handle:
                handle.write(line + "\n")


class SerialChannel:
    def __init__(self, name: str, app: "QtUpperConsole") -> None:
        self.name = name
        self.app = app
        self.ser: serial.Serial | None = None
        self.stop_event = threading.Event()
        self.write_lock = threading.Lock()

    def open(self, port: str, baud: int) -> None:
        self.close()
        self.ser = serial.Serial(port=port, baudrate=baud, timeout=0.05, write_timeout=1.0)
        self.stop_event.clear()
        threading.Thread(target=self._reader, daemon=True).start()
        self.app.emit_log(self.name, "INFO", f"opened {port} @ {baud}")

    def close(self) -> None:
        self.stop_event.set()
        if self.ser is not None:
            try:
                self.ser.close()
            except serial.SerialException:
                pass
        self.ser = None

    def is_open(self) -> bool:
        return self.ser is not None and self.ser.is_open

    def write(self, data: bytes, label: str) -> None:
        if self.ser is None or not self.ser.is_open:
            raise RuntimeError(f"{self.name} serial is not open")
        with self.write_lock:
            self.ser.write(data)
        if should_log_serial_tx(label):
            display = data.decode("ascii", errors="ignore").strip() if self.name == "ARM" else bytes_to_hex(data)
            self.app.emit_log(self.name, "TX", f"{label}: {display}")

    def _reader(self) -> None:
        while not self.stop_event.is_set():
            if self.ser is None:
                return
            try:
                data = self.ser.read(256)
            except serial.SerialException as exc:
                self.app.emit_log(self.name, "ERROR", str(exc))
                return
            if data:
                self.app.on_serial_data(self.name, data)


class QtUpperConsole(QMainWindow):
    def __init__(self, log_dir: Path | None = None) -> None:
        super().__init__()
        self.setWindowTitle(APP_TITLE)
        self.resize(1500, 900)
        self.settings = QSettings(SETTINGS_ORG, SETTINGS_APP)
        self.logger = SessionLogger(log_dir or (Path.cwd() / "logs"))
        self.log_queue: queue.Queue[str] = queue.Queue()
        self.status_update_queue: queue.Queue[tuple[str, str, str | None]] = queue.Queue()
        self._jetson_protocol = JETSON_PROTOCOL_OLD
        self.arm = SerialChannel("ARM", self)
        self.jetson = SerialChannel("JETSON", self)
        self.vision_stop = threading.Event()
        self.heartbeat_stop = threading.Event()
        self.heartbeat_stop.set()
        self.heartbeat_thread: threading.Thread | None = None
        self.safe_distance_stop = threading.Event()
        self.safe_distance_stop.set()
        self.safe_distance_thread: threading.Thread | None = None
        self.demo_stop = threading.Event()
        self.automatic_demo_lock = threading.Lock()
        self.arm_rx_buffer = bytearray()
        self.arm_events = {
            name: threading.Event()
            for name in (
                "SOFT_RESET_PASS",
                "SOFT_RESET_FAIL",
                "AUTO_FINISHED",
                "AUTO_FAILED",
                "VIEW_ARC_FINISHED",
                "VIEW_ARC_FAILED",
                "MOTION_ABORTED",
            )
        }
        self.view_arc_stop = threading.Event()
        self.view_arc_stop.set()
        self.view_arc_thread: threading.Thread | None = None
        self.automatic_demo_owner: str | None = None
        self.formal_status_mailbox = FormalStatusMailbox()
        self.formal_workflow_stop = threading.Event()
        self.formal_workflow_stop.set()
        self.formal_workflow_thread: threading.Thread | None = None
        self.formal_workflow_active = False
        self.formal_abort_sent = threading.Event()
        self.formal_vision_stop = threading.Event()
        self.formal_vision_stop.set()
        self.formal_vision_thread: threading.Thread | None = None
        self.workflow_stage_queue: queue.Queue[str] = queue.Queue()
        self.ra6_events = {name: threading.Event() for name in STATUS_NAMES.values()}
        self.capture_step_index = {name: 0 for name in DEFAULT_CAPTURE_RECIPES}
        self.status_labels: dict[str, QLabel] = {}
        self.status_dots: dict[str, QLabel] = {}
        self.unified_seq = int(time.time_ns() % 255) + 1
        self.seq_lock = threading.Lock()
        self.jetson_rx_buffer = bytearray()
        self._build_ui()
        self._set_status("PROTO", self.current_protocol_label())
        self._set_status("HEARTBEAT", "OFF")
        self._update_protocol_widgets()
        self.refresh_ports()
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._flush_logs)
        self.timer.start(80)
        self.emit_log("APP", "INFO", f"log file: {self.logger.path}")

    def _build_ui(self) -> None:
        root = QWidget()
        self.setCentralWidget(root)
        main = QHBoxLayout(root)
        left = QVBoxLayout()
        right = QVBoxLayout()
        main.addLayout(left, 0)
        main.addLayout(right, 1)
        left.addWidget(self._serial_group())
        self.left_tabs = QTabWidget()
        self.left_tabs.addTab(self._arm_group(), "机械臂控制")
        self.left_tabs.addTab(self._jetson_group(), "Jetson/定靶")
        self.left_tabs.addTab(self._capture_group(), "拍摄点位")
        left.addWidget(self.left_tabs, 1)
        left.addStretch(1)
        right.addWidget(self._status_group())
        log_bar = QHBoxLayout()
        log_bar.addWidget(QLabel("运行日志"))
        log_bar.addStretch(1)
        log_bar.addWidget(self._button("清空日志", self.clear_visible_log))
        right.addLayout(log_bar)
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        right.addWidget(self.log_text, 1)

    def _serial_group(self) -> QGroupBox:
        group = QGroupBox("串口连接")
        grid = QGridLayout(group)
        self.arm_port = QComboBox()
        self.jetson_port = QComboBox()
        self.baud_edit = QLineEdit(str(DEFAULT_BAUD))
        grid.addWidget(QLabel("ARM控制口"), 0, 0)
        grid.addWidget(self.arm_port, 0, 1)
        grid.addWidget(self._button("打开", self.open_arm), 0, 2)
        grid.addWidget(self._button("关闭", lambda: self.close_channel(self.arm)), 0, 3)
        grid.addWidget(QLabel("Jetson模拟口"), 1, 0)
        grid.addWidget(self.jetson_port, 1, 1)
        grid.addWidget(self._button("打开", self.open_jetson), 1, 2)
        grid.addWidget(self._button("关闭", lambda: self.close_channel(self.jetson)), 1, 3)
        grid.addWidget(QLabel("波特率"), 2, 0)
        grid.addWidget(self.baud_edit, 2, 1)
        grid.addWidget(self._button("刷新串口", self.refresh_ports), 2, 2)
        grid.addWidget(self._button("安全停止", self.safety_stop), 2, 3)
        return group

    def _arm_group(self) -> QGroupBox:
        group = QGroupBox("机械臂控制")
        layout = QVBoxLayout(group)
        grid = QGridLayout()
        commands = ["soft_reset", "read_all", "target_enable", "target_disable", "laser_on", "laser_off", "gripper_open", "gripper_stop"]
        for idx, command in enumerate(commands):
            grid.addWidget(self._button(command, lambda c=command: self.send_arm_command(c)), idx // 2, idx % 2)
        layout.addLayout(grid)

        self.auto_x = QLineEdit("0")
        self.auto_y = QLineEdit("-50")
        self.auto_z = QLineEdit("0")
        row = QHBoxLayout()
        row.addWidget(QLabel("auto"))
        row.addWidget(self.auto_x)
        row.addWidget(self.auto_y)
        row.addWidget(self.auto_z)
        row.addWidget(self._button("发送", self.send_auto))
        layout.addLayout(row)

        self.joint_id = QLineEdit("0")
        self.joint_angle = QLineEdit("0")
        row = QHBoxLayout()
        row.addWidget(QLabel("关节"))
        row.addWidget(self.joint_id)
        row.addWidget(self.joint_angle)
        row.addWidget(self._button("abs", lambda: self.send_joint("abs_rotate")))
        row.addWidget(self._button("rel", lambda: self.send_joint("rel_rotate")))
        layout.addLayout(row)

        self.circle_time = QLineEdit("8000")
        self.circle_radius = QLineEdit("5")
        row = QHBoxLayout()
        row.addWidget(QLabel("画圆"))
        row.addWidget(self.circle_time)
        row.addWidget(self.circle_radius)
        row.addWidget(self._button("发送", self.send_circle))
        layout.addLayout(row)

        self.view_arc_time = QLineEdit("8000")
        row = QHBoxLayout()
        row.addWidget(QLabel("三视图弧线"))
        row.addWidget(self.view_arc_time)
        row.addWidget(QLabel("ms"))
        row.addWidget(self._button("一键三视图弧线", self.start_view_arc_demo))
        row.addWidget(self._button("停止弧线", self.stop_view_arc_demo))
        layout.addLayout(row)

        self.custom_cmd = QLineEdit()
        row = QHBoxLayout()
        row.addWidget(self.custom_cmd)
        row.addWidget(self._button("发送自定义", lambda: self.send_arm_command(self.custom_cmd.text())))
        layout.addLayout(row)

        self.arm_history = QComboBox()
        self.arm_history.setEditable(True)
        self.arm_history.setMaxVisibleItems(ARM_HISTORY_LIMIT)
        self.arm_history.setInsertPolicy(QComboBox.InsertPolicy.NoInsert)
        if self.arm_history.lineEdit() is not None:
            self.arm_history.lineEdit().setPlaceholderText("输入或选择最近15条ARM指令")
        row = QHBoxLayout()
        row.addWidget(QLabel("历史"))
        row.addWidget(self.arm_history)
        row.addWidget(self._button("发送历史", self.send_history_command))
        layout.addLayout(row)
        return group

    def _jetson_group(self) -> QGroupBox:
        group = QGroupBox("Jetson 模拟器")
        layout = QVBoxLayout(group)
        self.protocol_mode = QComboBox()
        self.protocol_mode.addItem("旧协议 AA/FF/CC", JETSON_PROTOCOL_OLD)
        self.protocol_mode.addItem("新协议 A5 5A + CRC16", JETSON_PROTOCOL_NEW)
        saved_protocol = self._settings_text("jetson/protocol", JETSON_PROTOCOL_OLD)
        protocol_index = self.protocol_mode.findData(saved_protocol)
        self.protocol_mode.setCurrentIndex(protocol_index if protocol_index >= 0 else 0)
        self._jetson_protocol = str(self.protocol_mode.currentData() or JETSON_PROTOCOL_OLD)
        self.protocol_mode.currentIndexChanged.connect(self.on_protocol_changed)

        self.heartbeat_enable = QCheckBox("心跳保持")
        self.heartbeat_enable.setChecked(self._settings_bool("jetson/heartbeat_enable", False))
        self.heartbeat_enable.toggled.connect(self.on_heartbeat_toggled)
        self.heartbeat_period_ms = QLineEdit(self._settings_text("jetson/heartbeat_ms", "200"))
        self.safe_distance_enable = QCheckBox("安全距离保持")
        self.safe_distance_enable.setChecked(self._settings_bool("jetson/safe_distance_enable", False))
        self.safe_distance_enable.toggled.connect(self.on_safe_distance_toggled)
        self.safe_distance_mm = QLineEdit(self._settings_text("jetson/safe_distance_mm", "160"))
        self.safe_distance_valid = QCheckBox("有效")
        self.safe_distance_valid.setChecked(self._settings_bool("jetson/safe_distance_valid", True))

        row = QHBoxLayout()
        row.addWidget(QLabel("协议"))
        row.addWidget(self.protocol_mode, 1)
        row.addWidget(self.heartbeat_enable)
        row.addWidget(QLabel("心跳ms"))
        row.addWidget(self.heartbeat_period_ms)
        layout.addLayout(row)
        row = QHBoxLayout()
        row.addWidget(QLabel("安全距离mm"))
        row.addWidget(self.safe_distance_mm)
        row.addWidget(self.safe_distance_valid)
        row.addWidget(self.safe_distance_enable)
        row.addWidget(self._button("单发安全距离", self.send_safe_distance_once))
        layout.addLayout(row)
        row = QHBoxLayout()
        row.addWidget(self._button("启动状态机 AA 01 01 BB", lambda: self.send_target_ctrl(True)))
        row.addWidget(self._button("关闭状态机 AA 01 00 BB", self.stop_target_ctrl_with_reset))
        layout.addLayout(row)
        row = QHBoxLayout()
        row.addWidget(self._button("左视图拍摄", lambda: self.send_capture_ctrl(0x01, 1)))
        row.addWidget(self._button("正视图拍摄", lambda: self.send_capture_ctrl(0x01, 2)))
        row.addWidget(self._button("右视图拍摄", lambda: self.send_capture_ctrl(0x01, 3)))
        row.addWidget(self._button("三视图完成回HOME", lambda: self.send_capture_ctrl(0x02, 0)))
        layout.addLayout(row)
        row = QHBoxLayout()
        row.addWidget(self._button("选择左视图定靶点", lambda: self.send_capture_ctrl(0x03, 1)))
        row.addWidget(self._button("选择正视图定靶点", lambda: self.send_capture_ctrl(0x03, 2)))
        row.addWidget(self._button("选择右视图定靶点", lambda: self.send_capture_ctrl(0x03, 3)))
        layout.addLayout(row)
        self.safe_distance_sequence = QLineEdit(self._settings_text("jetson/safe_distance_sequence", DEFAULT_SAFE_DISTANCE_SEQUENCE))
        row = QHBoxLayout()
        row.addWidget(QLabel("安全距离序列"))
        row.addWidget(self.safe_distance_sequence, 1)
        row.addWidget(self._button("运行距离序列", self.run_safe_distance_sequence))
        row.addWidget(self._button("停止距离序列", self.stop_safe_distance))
        layout.addLayout(row)
        self.dcx = QLineEdit("0")
        self.dcy = QLineEdit("0")
        self.period_ms = QLineEdit("200")
        row = QHBoxLayout()
        row.addWidget(QLabel("dcx"))
        row.addWidget(self.dcx)
        row.addWidget(QLabel("dcy"))
        row.addWidget(self.dcy)
        row.addWidget(QLabel("ms"))
        row.addWidget(self.period_ms)
        layout.addLayout(row)
        row = QHBoxLayout()
        row.addWidget(self._button("单发视觉误差", self.send_vision_once))
        row.addWidget(self._button("开始周期发送", self.start_periodic_vision))
        row.addWidget(self._button("停止周期发送", self.stop_periodic_vision))
        layout.addLayout(row)
        self.sequence = QLineEdit("15,-10:2;8,-5:1;0,0:0")
        row = QHBoxLayout()
        row.addWidget(self.sequence)
        row.addWidget(self._button("运行序列", self.run_vision_sequence))
        layout.addLayout(row)
        layout.addWidget(self._target_demo_group())
        layout.addWidget(self._formal_workflow_group())
        return group

    def _target_demo_group(self) -> QGroupBox:
        group = QGroupBox("一键定靶演示")
        layout = QVBoxLayout(group)
        self.demo_sequence = QLineEdit(self._settings_text("demo/sequence", DEFAULT_DEMO_SEQUENCE))
        self.demo_period_ms = QLineEdit(self._settings_text("demo/period_ms", DEFAULT_DEMO_PERIOD_MS))
        row = QHBoxLayout()
        row.addWidget(QLabel("序列"))
        row.addWidget(self.demo_sequence, 1)
        row.addWidget(QLabel("ms"))
        row.addWidget(self.demo_period_ms)
        layout.addLayout(row)

        self.demo_send_enable = QCheckBox("先启动状态机")
        self.demo_wait_ready = QCheckBox("等待READY")
        self.demo_prompt_p000 = QCheckBox("对准后提示按P000")
        self.demo_auto_stop = QCheckBox("输出关闭后自动停止")
        self.demo_use_current_prestart = QCheckBox("当前位置作为定靶起点")
        for checkbox, key, default in (
            (self.demo_send_enable, "demo/send_enable", True),
            (self.demo_wait_ready, "demo/wait_ready", True),
            (self.demo_prompt_p000, "demo/prompt_p000", True),
            (self.demo_auto_stop, "demo/auto_stop", True),
            (self.demo_use_current_prestart, "demo/use_current_prestart", False),
        ):
            checkbox.setChecked(self._settings_bool(key, default))
        row = QHBoxLayout()
        for checkbox in (self.demo_send_enable, self.demo_wait_ready, self.demo_prompt_p000, self.demo_auto_stop):
            row.addWidget(checkbox)
        layout.addLayout(row)
        row = QHBoxLayout()
        row.addWidget(self.demo_use_current_prestart)
        row.addStretch(1)
        layout.addLayout(row)

        row = QHBoxLayout()
        row.addWidget(self._button("一键定靶演示", self.start_target_demo))
        row.addWidget(self._button("停止演示", self.stop_target_demo))
        row.addWidget(self._button("保存演示配置", self.save_demo_settings))
        layout.addLayout(row)
        return group

    def _formal_workflow_group(self) -> QGroupBox:
        group = QGroupBox("比赛全流程模拟")
        layout = QVBoxLayout(group)

        self.workflow_view = QComboBox()
        self.workflow_view.addItem("左视图", 1)
        self.workflow_view.addItem("正视图", 2)
        self.workflow_view.addItem("右视图", 3)
        saved_view = int(self._settings_text("workflow/view_id", "2"))
        view_index = self.workflow_view.findData(saved_view)
        self.workflow_view.setCurrentIndex(view_index if view_index >= 0 else 1)

        self.workflow_safe_distance = QLineEdit(
            self._settings_text("workflow/safe_distance_mm", DEFAULT_WORKFLOW_SAFE_DISTANCE_MM)
        )
        row = QHBoxLayout()
        row.addWidget(QLabel("选中视图"))
        row.addWidget(self.workflow_view)
        row.addWidget(QLabel("安全距离 mm"))
        row.addWidget(self.workflow_safe_distance)
        layout.addLayout(row)

        self.workflow_vision_sequence = QLineEdit(
            self._settings_text("workflow/vision_sequence", DEFAULT_WORKFLOW_VISION_SEQUENCE)
        )
        row = QHBoxLayout()
        row.addWidget(QLabel("视觉序列（每段秒）"))
        row.addWidget(self.workflow_vision_sequence, 1)
        layout.addLayout(row)

        self.workflow_stage = QLabel("未运行")
        row = QHBoxLayout()
        row.addWidget(QLabel("当前阶段"))
        row.addWidget(self.workflow_stage, 1)
        layout.addLayout(row)

        row = QHBoxLayout()
        row.addWidget(self._button("开始一键比赛全流程", lambda: self.start_formal_workflow_demo()))
        row.addWidget(self._button("终止并保持", lambda: self.stop_formal_workflow_demo()))
        row.addWidget(self._button("保存比赛流程配置", self.save_formal_workflow_settings))
        layout.addLayout(row)
        return group

    def save_formal_workflow_settings(self) -> None:
        self.settings.setValue("workflow/view_id", int(self.workflow_view.currentData()))
        self.settings.setValue("workflow/safe_distance_mm", self.workflow_safe_distance.text())
        self.settings.setValue("workflow/vision_sequence", self.workflow_vision_sequence.text())
        self.settings.sync()
        self.emit_log("WORKFLOW", "INFO", "比赛全流程配置已保存")

    def _set_workflow_stage(self, text: str) -> None:
        self.workflow_stage_queue.put(text)
        self.emit_log("WORKFLOW", "STAGE", text)

    def _acquire_automatic_demo(self, owner: str) -> bool:
        with self.automatic_demo_lock:
            if self.automatic_demo_owner is not None:
                return False
            self.automatic_demo_owner = owner
            return True

    def _release_automatic_demo(self, owner: str) -> None:
        with self.automatic_demo_lock:
            if self.automatic_demo_owner == owner:
                self.automatic_demo_owner = None

    def _wait_formal_status(
        self,
        event: int,
        value: int | None,
        seq: int,
        timeout_s: float,
    ) -> Ra6Status:
        return self.formal_status_mailbox.wait(
            event,
            value,
            seq,
            timeout_s,
            self.formal_workflow_stop,
        )

    def _send_formal_control(
        self,
        msg_type: int,
        payload: bytes,
        label: str,
        timeout_s: float = 5.0,
    ) -> int:
        if self.current_jetson_protocol() != JETSON_PROTOCOL_NEW:
            raise FormalWorkflowProtocolError("请先选择正式协议 A5 5A + CRC16")
        if not self.jetson.is_open():
            raise FormalWorkflowProtocolError("Jetson模拟串口未打开")

        seq = self._next_unified_seq()
        frame = build_unified_frame(msg_type, seq, payload)
        self.jetson.write(frame, f"正式控制 {label} SEQ=0x{seq:02X}")
        ack = self._wait_formal_status(EVENT_COMMAND_ACK, None, seq, timeout_s)
        if ack.value != 1 or ack.error != 0:
            raise FormalWorkflowProtocolError(
                f"{label}被 RA6M5 拒绝: 0x{ack.error:02X} {formal_error_message(ack.error)}"
            )
        self.emit_log("WORKFLOW", "ACK", f"{label}已接受，SEQ=0x{seq:02X}")
        return seq

    def _send_formal_distance(self, distance_mm: int) -> None:
        if not self.jetson.is_open():
            raise FormalWorkflowProtocolError("发送安全距离时 Jetson模拟串口已断开")
        seq = self._next_unified_seq()
        frame = build_unified_safe_distance_frame(distance_mm, True, seq)
        self.jetson.write(frame, f"SAFE_DISTANCE 比赛流程 {distance_mm}mm SEQ=0x{seq:02X}")

    def _workflow_pause(self, seconds: float) -> None:
        if self.formal_workflow_stop.wait(seconds):
            raise FormalWorkflowStopped("比赛流程已由用户终止")

    def _start_formal_heartbeat(self) -> None:
        self.heartbeat_period_ms.setText("200")
        self.settings.setValue("jetson/heartbeat_ms", "200")
        self.settings.setValue("jetson/heartbeat_enable", True)
        self.settings.sync()
        self.heartbeat_enable.blockSignals(True)
        self.heartbeat_enable.setChecked(True)
        self.heartbeat_enable.blockSignals(False)
        self.start_heartbeat()

    def _start_formal_vision(self, steps: Sequence[tuple[int, int, float]]) -> None:
        self._stop_formal_vision()
        self.formal_vision_stop.clear()
        self.formal_vision_thread = threading.Thread(
            target=self._formal_vision_loop,
            args=(tuple(steps),),
            daemon=True,
        )
        self.formal_vision_thread.start()

    def _stop_formal_vision(self) -> None:
        self.formal_vision_stop.set()
        thread = self.formal_vision_thread
        if thread is not None and thread is not threading.current_thread() and thread.is_alive():
            thread.join(timeout=1.0)
        self.formal_vision_thread = None

    def _formal_vision_loop(self, steps: tuple[tuple[int, int, float], ...]) -> None:
        try:
            for dcx, dcy, duration_s in steps:
                entered = time.monotonic()
                while not self.formal_vision_stop.is_set() and not self.formal_workflow_stop.is_set():
                    seq = self._next_unified_seq()
                    frame = build_unified_vision_error_frame(dcx, dcy, seq)
                    self.jetson.write(
                        frame,
                        f"FORMAL_VISION dcx={dcx} dcy={dcy} SEQ=0x{seq:02X}",
                    )
                    if duration_s > 0 and time.monotonic() - entered >= duration_s:
                        break
                    self.formal_vision_stop.wait(0.2)
                if self.formal_vision_stop.is_set() or self.formal_workflow_stop.is_set():
                    return
        except Exception as exc:
            self.emit_log("WORKFLOW", "ERROR", f"视觉误差发送失败：{exc}")
            self.formal_workflow_stop.set()

    def _send_formal_abort_best_effort(self) -> None:
        if self.formal_abort_sent.is_set():
            return
        self.formal_abort_sent.set()
        if not self.jetson.is_open():
            self.emit_log("WORKFLOW", "WARN", "无法发送 ABORT：Jetson模拟串口未打开")
            return
        try:
            seq = self._next_unified_seq()
            frame = build_unified_workflow_control_frame(WORKFLOW_ABORT, seq)
            self.jetson.write(frame, f"WORKFLOW ABORT SEQ=0x{seq:02X}")
            self.emit_log("WORKFLOW", "SAFETY", "已请求 RA6M5 终止流程并保持当前位置，激光应立即关闭")
        except Exception as exc:
            self.emit_log("WORKFLOW", "ERROR", f"发送 ABORT 失败：{exc}")

    def start_formal_workflow_demo(self) -> None:
        if not self.jetson.is_open():
            self._set_workflow_stage("无法启动：Jetson模拟串口未打开")
            self.emit_log("WORKFLOW", "ERROR", "比赛全流程无法启动：Jetson模拟串口未打开")
            return
        if self.current_jetson_protocol() != JETSON_PROTOCOL_NEW:
            self._set_workflow_stage("无法启动：请选择正式协议 A5 5A + CRC16")
            self.emit_log("WORKFLOW", "ERROR", "比赛全流程只支持正式协议 A5 5A + CRC16")
            return
        try:
            safe_distance_mm = int(self.workflow_safe_distance.text())
            if not 150 <= safe_distance_mm <= 65535:
                raise ValueError("安全距离不得低于固件阈值 150 mm")
            view_id = int(self.workflow_view.currentData())
            if view_id not in (1, 2, 3):
                raise ValueError("请选择左、正或右视图")
            vision_steps = parse_formal_vision_sequence(self.workflow_vision_sequence.text())
        except Exception as exc:
            self._set_workflow_stage(f"参数错误：{exc}")
            self.emit_log("WORKFLOW", "ERROR", f"比赛全流程参数错误：{exc}")
            return
        if not self._acquire_automatic_demo("formal"):
            self._set_workflow_stage("无法启动：另一个自动演示正在运行")
            self.emit_log("WORKFLOW", "WARN", "另一个自动演示正在运行，请先停止后再启动比赛全流程")
            return

        self.save_formal_workflow_settings()
        self.stop_safe_distance()
        self.formal_status_mailbox.clear()
        self.formal_workflow_stop.clear()
        self.formal_abort_sent.clear()
        self.formal_workflow_active = True
        self._start_formal_heartbeat()
        config = FormalWorkflowConfig(view_id, safe_distance_mm, vision_steps)
        self.formal_workflow_thread = threading.Thread(
            target=self._formal_workflow_thread_main,
            args=(config,),
            daemon=True,
        )
        self.formal_workflow_thread.start()
        self.emit_log("WORKFLOW", "INFO", "一键比赛全流程已启动")

    def _formal_workflow_thread_main(self, config: FormalWorkflowConfig) -> None:
        completed = False
        runner = FormalWorkflowRunner(
            send_control=self._send_formal_control,
            wait_status=self._wait_formal_status,
            send_distance=self._send_formal_distance,
            start_vision=self._start_formal_vision,
            stop_vision=self._stop_formal_vision,
            set_stage=self._set_workflow_stage,
            should_stop=self.formal_workflow_stop.is_set,
            pause=self._workflow_pause,
        )
        try:
            runner.run(config)
            completed = True
            self.emit_log("WORKFLOW", "DONE", "比赛全流程完成，机械臂已回 HOME")
        except FormalWorkflowStopped:
            self._set_workflow_stage("流程已终止，机械臂保持当前位置")
            self.emit_log("WORKFLOW", "SAFETY", "用户终止比赛流程，机械臂保持当前位置")
        except Exception as exc:
            self._set_workflow_stage(f"流程失败：{exc}")
            self.emit_log("WORKFLOW", "ERROR", f"比赛全流程失败：{exc}")
        finally:
            self._stop_formal_vision()
            if not completed:
                self._send_formal_abort_best_effort()
            self.stop_heartbeat()
            self.formal_workflow_active = False
            self._release_automatic_demo("formal")

    def stop_formal_workflow_demo(self) -> None:
        if not self.formal_workflow_active:
            self.emit_log("WORKFLOW", "INFO", "当前没有正在运行的比赛全流程")
            return
        self._set_workflow_stage("正在终止，机械臂将保持当前位置")
        self.formal_workflow_stop.set()
        self._stop_formal_vision()
        self._send_formal_abort_best_effort()

    def _capture_group(self) -> QGroupBox:
        group = QGroupBox("人脸三视图拍摄点位")
        layout = QVBoxLayout(group)

        self.capture_view = QComboBox()
        self.capture_view.addItems(DEFAULT_CAPTURE_RECIPES.keys())
        self.capture_view.currentTextChanged.connect(self.load_capture_recipe)
        row = QHBoxLayout()
        row.addWidget(QLabel("视图"))
        row.addWidget(self.capture_view, 1)
        layout.addLayout(row)

        self.capture_recipe = QTextEdit()
        self.capture_recipe.setMinimumHeight(180)
        layout.addWidget(self.capture_recipe, 1)

        row = QHBoxLayout()
        row.addWidget(self._button("保存配方", self.save_capture_recipe))
        row.addWidget(self._button("重置默认", self.reset_capture_recipe))
        row.addWidget(self._button("从头开始", self.reset_capture_steps))
        row.addWidget(self._button("发送下一条", self.send_next_capture_command))
        layout.addLayout(row)

        self.load_capture_recipe(self.capture_view.currentText())
        return group

    def _status_group(self) -> QGroupBox:
        group = QGroupBox("状态")
        grid = QGridLayout(group)
        keys = ["ARM_PORT", "JETSON_PORT", "PROTO", "HEARTBEAT", "POSE_VALID", "TARGET_CTRL", "READY", "ALIGN_DONE", "SAFE_DIST", "CONFIRMING", "OUTPUT", "ERROR"]
        for idx, key in enumerate(keys):
            dot = QLabel()
            dot.setFixedSize(14, 14)
            self.status_dots[key] = dot
            self._set_status_dot(key, "gray")
            label = QLabel(f"{key}: -")
            label.setMinimumWidth(190)
            self.status_labels[key] = label
            cell = QHBoxLayout()
            cell.addWidget(dot)
            cell.addWidget(label)
            wrapper = QWidget()
            wrapper.setLayout(cell)
            grid.addWidget(wrapper, idx // 3, idx % 3)
        return group

    def _settings_text(self, key: str, default: str) -> str:
        return str(self.settings.value(key, default))

    def _settings_bool(self, key: str, default: bool) -> bool:
        value = self.settings.value(key, default)
        if isinstance(value, bool):
            return value
        return str(value).lower() in {"1", "true", "yes", "on"}

    def _capture_settings_key(self, view: str) -> str:
        return f"capture/{CAPTURE_RECIPE_KEYS.get(view, 'left')}"

    def _button(self, text: str, callback) -> QPushButton:
        button = QPushButton(text)
        button.clicked.connect(lambda _checked=False, cb=callback: cb())
        return button

    def current_jetson_protocol(self) -> str:
        return self._jetson_protocol

    def current_protocol_label(self) -> str:
        return "NEW" if self.current_jetson_protocol() == JETSON_PROTOCOL_NEW else "OLD"

    def _update_protocol_widgets(self) -> None:
        enabled = self.current_jetson_protocol() == JETSON_PROTOCOL_NEW
        self.heartbeat_enable.setEnabled(enabled)
        self.heartbeat_period_ms.setEnabled(enabled)
        if hasattr(self, "safe_distance_enable"):
            self.safe_distance_enable.setEnabled(enabled)
            self.safe_distance_mm.setEnabled(enabled)
            self.safe_distance_valid.setEnabled(enabled)
            self.safe_distance_sequence.setEnabled(enabled)

    def _next_unified_seq(self) -> int:
        with self.seq_lock:
            self.unified_seq = (self.unified_seq + 1) & 0xFF
            if self.unified_seq == 0:
                self.unified_seq = 1
            return self.unified_seq

    def on_protocol_changed(self) -> None:
        protocol = str(self.protocol_mode.currentData() or JETSON_PROTOCOL_OLD)
        self._jetson_protocol = protocol
        if protocol != JETSON_PROTOCOL_NEW and self.formal_workflow_active:
            self.stop_formal_workflow_demo()
        self.settings.setValue("jetson/protocol", protocol)
        self.settings.sync()
        self._set_status("PROTO", self.current_protocol_label())
        self._update_protocol_widgets()
        if protocol == JETSON_PROTOCOL_NEW:
            self.emit_log("JETSON", "INFO", "protocol switched to NEW A5 5A + CRC16")
            if self.heartbeat_enable.isChecked():
                self.start_heartbeat()
            if self.safe_distance_enable.isChecked():
                self.start_safe_distance()
        else:
            self.stop_heartbeat()
            self.stop_safe_distance()
            self.emit_log("JETSON", "INFO", "protocol switched to OLD AA/FF/CC; heartbeat is disabled")

    def on_heartbeat_toggled(self, checked: bool) -> None:
        self.settings.setValue("jetson/heartbeat_enable", checked)
        self.settings.setValue("jetson/heartbeat_ms", self.heartbeat_period_ms.text())
        self.settings.sync()
        if checked:
            self.start_heartbeat()
        else:
            self.stop_heartbeat()

    def on_safe_distance_toggled(self, checked: bool) -> None:
        self.settings.setValue("jetson/safe_distance_enable", checked)
        self.settings.setValue("jetson/safe_distance_mm", self.safe_distance_mm.text())
        self.settings.setValue("jetson/safe_distance_valid", self.safe_distance_valid.isChecked())
        self.settings.sync()
        if checked:
            self.start_safe_distance()
        else:
            self.stop_safe_distance()

    def start_heartbeat(self) -> None:
        if self.current_jetson_protocol() != JETSON_PROTOCOL_NEW:
            self._set_status("HEARTBEAT", "OFF")
            self.emit_log("JETSON", "INFO", "OLD protocol has no heartbeat")
            return
        if not self.jetson.is_open():
            self._set_status("HEARTBEAT", "OFF")
            self.emit_log("JETSON", "WARN", "Jetson serial is not open; heartbeat not started")
            return
        try:
            period_ms = max(50, int(self.heartbeat_period_ms.text()))
        except ValueError:
            period_ms = 200
            self.heartbeat_period_ms.setText(str(period_ms))
        self.settings.setValue("jetson/heartbeat_ms", str(period_ms))
        self.settings.sync()

        if self.heartbeat_thread is not None and self.heartbeat_thread.is_alive() and not self.heartbeat_stop.is_set():
            self._set_status("HEARTBEAT", "ON")
            return

        self.heartbeat_stop.set()
        stop_event = threading.Event()
        self.heartbeat_stop = stop_event
        try:
            self._send_heartbeat_frame()
        except Exception as exc:
            stop_event.set()
            self._set_status("HEARTBEAT", "OFF")
            self.emit_log("JETSON", "ERROR", f"首帧心跳发送失败：{exc}")
            return
        self.heartbeat_thread = threading.Thread(target=self._heartbeat_loop, args=(period_ms, stop_event), daemon=True)
        self.heartbeat_thread.start()
        self._set_status("HEARTBEAT", "ON")
        self.emit_log("JETSON", "INFO", f"heartbeat started: {period_ms} ms")

    def stop_heartbeat(self) -> None:
        self.heartbeat_stop.set()
        self._set_status("HEARTBEAT", "OFF")
        self.emit_log("JETSON", "INFO", "heartbeat stopped")

    def _send_heartbeat_frame(self) -> None:
        tick_ms = int(time.monotonic() * 1000) & 0xFFFFFFFF
        self.jetson.write(
            build_unified_heartbeat_frame(self._next_unified_seq(), tick_ms),
            "HEARTBEAT",
        )

    def _heartbeat_loop(self, period_ms: int, stop_event: threading.Event) -> None:
        while not stop_event.is_set():
            if not self.jetson.is_open() or self.current_jetson_protocol() != JETSON_PROTOCOL_NEW:
                stop_event.set()
                break
            try:
                self._send_heartbeat_frame()
            except Exception as exc:
                self.emit_log("JETSON", "ERROR", str(exc))
                stop_event.set()
                break
            time.sleep(period_ms / 1000.0)
        if self.heartbeat_stop is stop_event:
            self._set_status("HEARTBEAT", "OFF")

    def send_safe_distance_once(self) -> None:
        try:
            sent = self.send_safe_distance_values(int(self.safe_distance_mm.text()), self.safe_distance_valid.isChecked())
            if sent:
                self.emit_log("JETSON", "INFO", f"safe distance sent: {self.safe_distance_mm.text()} mm")
        except Exception as exc:
            self.emit_log("JETSON", "ERROR", str(exc))

    def send_safe_distance_values(self, distance_mm: int, valid: bool) -> bool:
        if self.current_jetson_protocol() != JETSON_PROTOCOL_NEW:
            self.emit_log("JETSON", "WARN", "OLD protocol does not support safe distance")
            return False
        if not self.jetson.is_open():
            self.emit_log("JETSON", "ERROR", "Jetson serial is not open; safe distance not sent")
            return False
        frame = build_unified_safe_distance_frame(distance_mm, valid, self._next_unified_seq())
        self.jetson.write(frame, f"SAFE_DISTANCE distance={distance_mm} valid={1 if valid else 0}")
        return True

    def start_safe_distance(self) -> None:
        if self.current_jetson_protocol() != JETSON_PROTOCOL_NEW:
            self._set_status("SAFE_DIST", "-")
            self.emit_log("JETSON", "WARN", "OLD protocol does not support safe distance")
            return
        if not self.jetson.is_open():
            self.emit_log("JETSON", "WARN", "Jetson serial is not open; safe distance not started")
            return
        try:
            distance_mm = int(self.safe_distance_mm.text())
            period_ms = max(50, int(self.period_ms.text()))
        except ValueError as exc:
            self.emit_log("JETSON", "ERROR", f"bad safe distance setting: {exc}")
            return
        valid = self.safe_distance_valid.isChecked()
        self.settings.setValue("jetson/safe_distance_mm", str(distance_mm))
        self.settings.setValue("jetson/safe_distance_valid", valid)
        self.settings.sync()

        if self.safe_distance_thread is not None and self.safe_distance_thread.is_alive() and not self.safe_distance_stop.is_set():
            return

        self.safe_distance_stop.set()
        stop_event = threading.Event()
        self.safe_distance_stop = stop_event
        self.safe_distance_thread = threading.Thread(
            target=self._safe_distance_loop,
            args=(distance_mm, valid, period_ms, stop_event),
            daemon=True,
        )
        self.safe_distance_thread.start()
        self.emit_log("JETSON", "INFO", f"safe distance started: {distance_mm} mm, {period_ms} ms")

    def stop_safe_distance(self) -> None:
        self.safe_distance_stop.set()
        self.emit_log("JETSON", "INFO", "safe distance stopped")

    def _safe_distance_loop(self, distance_mm: int, valid: bool, period_ms: int, stop_event: threading.Event) -> None:
        while not stop_event.is_set():
            if not self.jetson.is_open() or self.current_jetson_protocol() != JETSON_PROTOCOL_NEW:
                stop_event.set()
                break
            try:
                self.send_safe_distance_values(distance_mm, valid)
            except Exception as exc:
                self.emit_log("JETSON", "ERROR", str(exc))
                stop_event.set()
                break
            time.sleep(period_ms / 1000.0)

    def run_safe_distance_sequence(self) -> None:
        if self.current_jetson_protocol() != JETSON_PROTOCOL_NEW:
            self.emit_log("JETSON", "WARN", "旧协议不支持安全距离序列，请切换到新协议")
            return
        if not self.jetson.is_open():
            self.emit_log("JETSON", "ERROR", "Jetson模拟串口未打开，安全距离序列未发送")
            return
        try:
            sequence = parse_safe_distance_sequence(self.safe_distance_sequence.text())
            period_ms = max(50, int(self.period_ms.text()))
        except Exception as exc:
            self.emit_log("JETSON", "ERROR", f"安全距离序列格式错误: {exc}")
            return
        if not sequence:
            self.emit_log("JETSON", "WARN", "安全距离序列为空")
            return

        self.settings.setValue("jetson/safe_distance_sequence", self.safe_distance_sequence.text())
        self.settings.sync()
        self.safe_distance_stop.set()
        stop_event = threading.Event()
        self.safe_distance_stop = stop_event
        valid = self.safe_distance_valid.isChecked()
        self.safe_distance_thread = threading.Thread(
            target=self._safe_distance_sequence_loop,
            args=(sequence, valid, period_ms, stop_event),
            daemon=True,
        )
        self.safe_distance_thread.start()
        self.emit_log("JETSON", "INFO", f"安全距离序列开始: {self.safe_distance_sequence.text()} period={period_ms} ms")

    def _safe_distance_sequence_loop(self, sequence: list[tuple[int, int]], valid: bool, period_ms: int, stop_event: threading.Event) -> None:
        for distance_mm, count in sequence:
            if stop_event.is_set():
                return
            label_count = "持续" if count == 0 else str(count)
            self.emit_log("JETSON", "INFO", f"安全距离段: {distance_mm} mm x {label_count}")
            if count == 0:
                while not stop_event.is_set():
                    self.send_safe_distance_values(distance_mm, valid)
                    time.sleep(period_ms / 1000.0)
                return
            for _ in range(count):
                if stop_event.is_set():
                    return
                self.send_safe_distance_values(distance_mm, valid)
                time.sleep(period_ms / 1000.0)
        if self.safe_distance_stop is stop_event:
            self.emit_log("JETSON", "INFO", "安全距离序列完成")

    def refresh_ports(self) -> None:
        ports = [item.device for item in list_ports.comports()]
        for combo, preferred in ((self.arm_port, "COM7"), (self.jetson_port, "COM14")):
            current = combo.currentText()
            combo.clear()
            combo.addItems(ports)
            idx = combo.findText(current) if current else -1
            if idx < 0:
                idx = combo.findText(preferred)
            if idx >= 0:
                combo.setCurrentIndex(idx)
        if ports:
            self.emit_log("APP", "INFO", "已刷新串口: " + ", ".join(ports))
        else:
            self.emit_log("APP", "WARN", "未发现串口，请检查USB连接后再点“刷新串口”。")

    def open_arm(self) -> None:
        self._open_channel(self.arm, self.arm_port.currentText(), "ARM_PORT")

    def open_jetson(self) -> None:
        self._open_channel(self.jetson, self.jetson_port.currentText(), "JETSON_PORT")

    def _open_channel(self, channel: SerialChannel, port: str, key: str) -> None:
        if not port:
            self.emit_log(channel.name, "ERROR", "未选择串口，请先点“刷新串口”。")
            QMessageBox.warning(self, "串口未选择", "未选择串口，请先点“刷新串口”。")
            return
        try:
            channel.open(port, int(self.baud_edit.text()))
            self._set_status(key, f"已打开 {port}")
            if channel is self.arm:
                self.arm_rx_buffer.clear()
                self._clear_arm_runtime_status()
            elif channel is self.jetson:
                self._clear_target_runtime_status(clear_target=True)
                if self.current_jetson_protocol() == JETSON_PROTOCOL_NEW and self.heartbeat_enable.isChecked():
                    self.start_heartbeat()
                if self.current_jetson_protocol() == JETSON_PROTOCOL_NEW and self.safe_distance_enable.isChecked():
                    self.start_safe_distance()
        except Exception as exc:
            self.emit_log(channel.name, "ERROR", str(exc))
            QMessageBox.warning(self, "串口打开失败", str(exc))

    def close_channel(self, channel: SerialChannel) -> None:
        if channel is self.jetson:
            if self.formal_workflow_active:
                self.formal_workflow_stop.set()
                self._stop_formal_vision()
                self._send_formal_abort_best_effort()
            self.stop_periodic_vision()
            self.stop_heartbeat()
            self.stop_safe_distance()
        elif channel is self.arm:
            if self.view_arc_thread is not None and self.view_arc_thread.is_alive():
                self.stop_view_arc_demo()
            self.arm_rx_buffer.clear()
        channel.close()
        self._set_status("ARM_PORT" if channel is self.arm else "JETSON_PORT", "已关闭")
        if channel is self.arm:
            self._clear_arm_runtime_status()
        elif channel is self.jetson:
            self._clear_target_runtime_status(clear_target=True)
        self.emit_log(channel.name, "INFO", "已关闭串口")

    def send_arm_command(self, command: str) -> None:
        if not isinstance(command, str):
            command = str(command)
        command = command.strip()
        if not command:
            self.emit_log("ARM", "WARN", "空命令已忽略。")
            return
        if self.formal_workflow_active and command.lower() != "laser_off":
            self.emit_log("ARM", "SAFETY", f"比赛全流程运行中，已拒绝手动命令: {command}")
            return
        self._remember_arm_command(command)
        if not self.arm.is_open():
            self.emit_log("ARM", "ERROR", f"ARM串口未打开，命令未发送: {command}")
            return
        try:
            self.arm.write(build_arm_command(command), f"发送命令 {command}")
        except Exception as exc:
            self.emit_log("ARM", "ERROR", str(exc))

    def send_auto(self) -> None:
        self.send_arm_command(f"auto {self.auto_x.text()} {self.auto_y.text()} {self.auto_z.text()}")

    def send_joint(self, command: str) -> None:
        self.send_arm_command(f"{command} {self.joint_id.text()} {self.joint_angle.text()}")

    def send_circle(self) -> None:
        self.send_arm_command(f"circle {self.circle_time.text()} {self.circle_radius.text()}")

    def _clear_arm_events(self, names: tuple[str, ...]) -> None:
        for name in names:
            self.arm_events[name].clear()

    def _wait_arm_event(
        self,
        success_name: str,
        failure_names: tuple[str, ...],
        timeout_s: float,
    ) -> str:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self.view_arc_stop.is_set():
                return "stopped"
            if self.arm_events[success_name].is_set():
                return "ok"
            for name in failure_names:
                if self.arm_events[name].is_set():
                    return name.lower()
            time.sleep(0.05)
        return "timeout"

    def _run_view_arc_stage(
        self,
        command: str,
        success_name: str,
        failure_names: tuple[str, ...],
        timeout_s: float,
        prompt: str,
    ) -> bool:
        names = (success_name,) + failure_names
        self._clear_arm_events(names)
        self.emit_log("VIEW_ARC", "STAGE", prompt)
        self.send_arm_command(command)
        result = self._wait_arm_event(success_name, failure_names, timeout_s)
        if result == "ok":
            return True
        self.emit_log(
            "VIEW_ARC",
            "ERROR",
            f"{prompt}未完成，结果={result}，已终止后续动作。",
        )
        return False

    def _view_arc_demo_loop(self, duration_ms: int) -> bool:
        if not self._run_view_arc_stage(
            "soft_reset",
            "SOFT_RESET_PASS",
            ("SOFT_RESET_FAIL", "MOTION_ABORTED"),
            50.0,
            "正在回 HOME",
        ):
            return False
        if not self._run_view_arc_stage(
            "auto 0 -130 -15",
            "AUTO_FINISHED",
            ("AUTO_FAILED", "MOTION_ABORTED"),
            30.0,
            "正在前往三视图公共位",
        ):
            return False
        if not self._run_view_arc_stage(
            f"view_arc {duration_ms}",
            "VIEW_ARC_FINISHED",
            ("VIEW_ARC_FAILED", "MOTION_ABORTED"),
            duration_ms / 1000.0 + 15.0,
            "正在平滑经过左、正、右视图",
        ):
            return False
        self.emit_log(
            "VIEW_ARC",
            "INFO",
            "三视图弧线完成，机械臂保持右视图；下一次 AUTO 前请先执行 soft_reset。",
        )
        return True

    def _view_arc_demo_thread_main(self, duration_ms: int) -> None:
        completed = False
        try:
            completed = self._view_arc_demo_loop(duration_ms)
        finally:
            if not completed and not self.view_arc_stop.is_set():
                self.send_arm_command("motion_abort")
                self.send_arm_command("laser_off")
                self.emit_log("VIEW_ARC", "SAFETY", "弧线流程异常，已锁存停止并关闭激光。")
            self.view_arc_stop.set()
            self._release_automatic_demo("view_arc")

    def start_view_arc_demo(self) -> None:
        if not self.arm.is_open():
            self.emit_log("VIEW_ARC", "ERROR", "ARM串口未打开，无法启动三视图弧线。")
            return
        try:
            duration_ms = int(self.view_arc_time.text())
        except ValueError:
            self.emit_log("VIEW_ARC", "ERROR", "弧线时间必须是 4000～8000 ms 的整数。")
            return
        if not 4000 <= duration_ms <= 8000:
            self.emit_log("VIEW_ARC", "ERROR", "弧线时间必须在 4000～8000 ms。")
            return
        if self.formal_workflow_active:
            self.emit_log("VIEW_ARC", "WARN", "比赛全流程运行中，不能启动独立弧线演示。")
            return
        if not self._acquire_automatic_demo("view_arc"):
            self.emit_log("VIEW_ARC", "WARN", "另一个自动演示正在运行，不能启动三视图弧线。")
            return

        self.view_arc_stop.clear()
        self._clear_arm_events(tuple(self.arm_events))
        self.view_arc_thread = threading.Thread(
            target=self._view_arc_demo_thread_main,
            args=(duration_ms,),
            daemon=True,
        )
        self.view_arc_thread.start()
        self.emit_log("VIEW_ARC", "INFO", f"一键三视图弧线已启动，轨迹时间 {duration_ms} ms。")

    def stop_view_arc_demo(self) -> None:
        self.view_arc_stop.set()
        self.send_arm_command("motion_abort")
        self.send_arm_command("laser_off")
        self.emit_log("VIEW_ARC", "SAFETY", "已请求停止弧线并关闭激光；恢复运动前必须 soft_reset。")

    def send_history_command(self) -> None:
        self.send_arm_command(self.arm_history.currentText())

    def _remember_arm_command(self, command: str) -> None:
        history = [self.arm_history.itemText(idx) for idx in range(self.arm_history.count())]
        history = [item for item in history if item != command]
        history.insert(0, command)
        history = history[:ARM_HISTORY_LIMIT]
        self.arm_history.blockSignals(True)
        self.arm_history.clear()
        self.arm_history.addItems(history)
        self.arm_history.setCurrentIndex(0)
        if self.arm_history.lineEdit() is not None:
            self.arm_history.setEditText(command)
        self.arm_history.blockSignals(False)

    def send_target_ctrl(self, enable: bool) -> bool:
        if not self.jetson.is_open():
            self.emit_log("JETSON", "ERROR", "Jetson模拟串口未打开，控制帧未发送。")
            return False
        try:
            if self.current_jetson_protocol() == JETSON_PROTOCOL_NEW:
                if enable and threading.current_thread() is threading.main_thread():
                    if self.heartbeat_enable.isChecked():
                        self.start_heartbeat()
                    if self.safe_distance_enable.isChecked():
                        self.start_safe_distance()
                frame = build_unified_target_control_frame(enable, self._next_unified_seq())
                label = "TARGET_CTRL_START_NEW" if enable else "TARGET_CTRL_STOP_NEW"
            else:
                frame = build_target_control_frame(enable)
                label = "TARGET_CTRL_START" if enable else "TARGET_CTRL_STOP"
            self.jetson.write(frame, label)
        except Exception as exc:
            self.emit_log("JETSON", "ERROR", str(exc))
            return False
        return True

    def send_capture_ctrl(self, action: int, point_id: int) -> bool:
        if self.current_jetson_protocol() != JETSON_PROTOCOL_NEW:
            self.emit_log("JETSON", "ERROR", "CAPTURE_CTRL requires NEW A5 5A + CRC16 protocol.")
            return False
        if not self.jetson.is_open():
            self.emit_log("JETSON", "ERROR", "Jetson serial is not open; capture control not sent.")
            return False
        try:
            if threading.current_thread() is threading.main_thread():
                if self.heartbeat_enable.isChecked():
                    self.start_heartbeat()
                self.send_safe_distance_values(int(self.safe_distance_mm.text()), self.safe_distance_valid.isChecked())
                if self.safe_distance_enable.isChecked():
                    self.start_safe_distance()
            frame = build_unified_capture_control_frame(action, point_id, self._next_unified_seq())
            label = f"CAPTURE_CTRL a={action} p={point_id}"
            self.jetson.write(frame, label)
        except Exception as exc:
            self.emit_log("JETSON", "ERROR", str(exc))
            return False
        return True

    def stop_target_ctrl_with_reset(self) -> None:
        self.stop_heartbeat()
        self.stop_safe_distance()
        if not self.send_target_ctrl(False):
            self.send_arm_command("soft_reset")

    def send_vision_once(self) -> None:
        try:
            self.send_vision_values(int(self.dcx.text()), int(self.dcy.text()))
        except Exception as exc:
            self.emit_log("JETSON", "ERROR", str(exc))

    def send_vision_values(self, dcx: int, dcy: int) -> None:
        if not self.jetson.is_open():
            self.emit_log("JETSON", "ERROR", f"Jetson模拟串口未打开，视觉误差未发送: dcx={dcx} dcy={dcy}")
            return
        if self.current_jetson_protocol() == JETSON_PROTOCOL_NEW:
            frame = build_unified_vision_error_frame(dcx, dcy, self._next_unified_seq())
            label = f"VISION_NEW dcx={dcx} dcy={dcy}"
        else:
            frame = build_vision_error_frame(dcx, dcy)
            label = f"VISION dcx={dcx} dcy={dcy}"
        self.jetson.write(frame, label)

    def start_periodic_vision(self) -> None:
        self.vision_stop.clear()
        dcx = int(self.dcx.text())
        dcy = int(self.dcy.text())
        period = max(20, int(self.period_ms.text()))
        threading.Thread(target=self._vision_loop, args=(dcx, dcy, period), daemon=True).start()
        self.emit_log("JETSON", "INFO", "periodic vision started")

    def stop_periodic_vision(self) -> None:
        self.vision_stop.set()
        self.emit_log("JETSON", "INFO", "已停止周期视觉发送")

    def _vision_loop(self, dcx: int, dcy: int, period_ms: int) -> None:
        while not self.vision_stop.is_set():
            try:
                self.send_vision_values(dcx, dcy)
            except Exception as exc:
                self.emit_log("JETSON", "ERROR", str(exc))
                self.vision_stop.set()
                return
            time.sleep(period_ms / 1000.0)

    def run_vision_sequence(self) -> None:
        try:
            sequence = parse_vision_sequence(self.sequence.text())
            period = max(20, int(self.period_ms.text()))
        except Exception as exc:
            self.emit_log("JETSON", "ERROR", str(exc))
            return
        self.vision_stop.clear()
        threading.Thread(target=self._sequence_loop, args=(sequence, period), daemon=True).start()
        self.emit_log("JETSON", "INFO", f"vision sequence started: {self.sequence.text()}")

    def save_demo_settings(self) -> None:
        self.settings.setValue("demo/sequence", self.demo_sequence.text())
        self.settings.setValue("demo/period_ms", self.demo_period_ms.text())
        self.settings.setValue("demo/send_enable", self.demo_send_enable.isChecked())
        self.settings.setValue("demo/wait_ready", self.demo_wait_ready.isChecked())
        self.settings.setValue("demo/prompt_p000", self.demo_prompt_p000.isChecked())
        self.settings.setValue("demo/auto_stop", self.demo_auto_stop.isChecked())
        self.settings.setValue("demo/use_current_prestart", self.demo_use_current_prestart.isChecked())
        self.settings.sync()
        self.emit_log("DEMO", "INFO", "演示配置已保存")

    def load_capture_recipe(self, view: str) -> None:
        text = self._settings_text(self._capture_settings_key(view), DEFAULT_CAPTURE_RECIPES.get(view, ""))
        self.capture_recipe.setPlainText(text)
        self.capture_step_index[view] = 0

    def save_capture_recipe(self) -> None:
        view = self.capture_view.currentText()
        self.settings.setValue(self._capture_settings_key(view), self.capture_recipe.toPlainText())
        self.settings.sync()
        self.capture_step_index[view] = 0
        self.emit_log("APP", "INFO", f"{view} 拍摄配方已保存")

    def reset_capture_recipe(self) -> None:
        view = self.capture_view.currentText()
        self.capture_recipe.setPlainText(DEFAULT_CAPTURE_RECIPES.get(view, ""))
        self.capture_step_index[view] = 0
        self.emit_log("APP", "INFO", f"{view} 拍摄配方已重置为默认")

    def _capture_commands(self) -> list[str]:
        commands: list[str] = []
        for line in self.capture_recipe.toPlainText().splitlines():
            command = line.strip()
            if command and not command.startswith("#"):
                commands.append(command)
        return commands

    def reset_capture_steps(self) -> None:
        view = self.capture_view.currentText()
        self.capture_step_index[view] = 0
        self.emit_log("APP", "INFO", f"{view} 拍摄配方已从头开始")

    def send_next_capture_command(self) -> None:
        view = self.capture_view.currentText()
        commands = self._capture_commands()
        index = self.capture_step_index.get(view, 0)
        if index >= len(commands):
            self.emit_log("APP", "INFO", f"{view} 拍摄配方已执行完")
            return
        command = commands[index]
        self.capture_step_index[view] = index + 1
        self.emit_log("APP", "INFO", f"{view} 第 {index + 1}/{len(commands)} 条: {command}")
        self.send_arm_command(command)

    def start_target_demo(self) -> None:
        try:
            sequence = parse_vision_sequence(self.demo_sequence.text())
            period = max(20, int(self.demo_period_ms.text()))
        except Exception as exc:
            self.emit_log("DEMO", "ERROR", str(exc))
            return
        if not self.jetson.is_open():
            self.emit_log("DEMO", "ERROR", "Jetson模拟串口未打开，无法开始一键定靶演示。")
            return
        if self.current_jetson_protocol() != JETSON_PROTOCOL_NEW:
            self.emit_log("DEMO", "ERROR", "OLD protocol does not support safe distance; switch to NEW A5 5A + CRC16 for one-key demo")
            return
        if not self._acquire_automatic_demo("target"):
            with self.automatic_demo_lock:
                owner = self.automatic_demo_owner
            if owner == "formal":
                self.emit_log("DEMO", "WARN", "比赛全流程正在运行，不能同时启动一键定靶演示")
            else:
                self.emit_log("DEMO", "WARN", "另一个自动演示正在运行")
            return
        self.save_demo_settings()
        if self.current_jetson_protocol() == JETSON_PROTOCOL_NEW:
            if self.heartbeat_enable.isChecked():
                self.start_heartbeat()
            else:
                self.heartbeat_enable.setChecked(True)
            if self.safe_distance_enable.isChecked():
                self.start_safe_distance()
            else:
                self.safe_distance_enable.setChecked(True)
        self._clear_ra6_events()
        self.demo_stop.clear()
        self.vision_stop.set()
        options = (
            self.demo_send_enable.isChecked(),
            self.demo_wait_ready.isChecked(),
            self.demo_prompt_p000.isChecked(),
            self.demo_auto_stop.isChecked(),
            self.demo_use_current_prestart.isChecked(),
        )
        threading.Thread(target=self._target_demo_thread_main, args=(sequence, period, options), daemon=True).start()
        self.emit_log("DEMO", "INFO", "一键定靶演示开始")

    def stop_target_demo(self) -> None:
        with self.automatic_demo_lock:
            owner = self.automatic_demo_owner
        if owner == "formal":
            self.emit_log("DEMO", "WARN", "比赛全流程正在运行，请使用“终止并保持”按钮")
            return
        self.demo_stop.set()
        self.stop_periodic_vision()
        self.stop_heartbeat()
        self.stop_safe_distance()
        stop_sent = self.send_target_ctrl(False)
        self.send_arm_command("laser_off")
        if not stop_sent:
            self.send_arm_command("soft_reset")
        self.emit_log("DEMO", "INFO", "演示已停止，已关闭视觉发送、激光输出，并请求回 HOME")

    def _target_demo_thread_main(
        self,
        sequence: list[tuple[int, int, int]],
        period_ms: int,
        options: tuple[bool, bool, bool, bool, bool],
    ) -> None:
        try:
            self._target_demo_loop(sequence, period_ms, options)
        finally:
            self._release_automatic_demo("target")

    def _target_demo_loop(self, sequence: list[tuple[int, int, int]], period_ms: int, options: tuple[bool, bool, bool, bool, bool]) -> None:
        send_enable, wait_ready, prompt_p000, auto_stop, use_current_prestart = options
        if send_enable:
            if use_current_prestart:
                if not self.send_capture_ctrl(0x04, 0):
                    self.emit_log("DEMO", "ERROR", "当前位置定靶起点标记失败，演示停止。")
                    return
                if not self._wait_ra6_event("TARGET_PRESTART_CURRENT", 3.0):
                    self.stop_periodic_vision()
                    self.stop_heartbeat()
                    self.stop_safe_distance()
                    self.emit_log("DEMO", "ERROR", "未收到当前位置定靶起点确认，演示停止。")
                    return
            self.send_target_ctrl(True)
        if wait_ready and not self._wait_ra6_event("READY", 12.0):
            self.stop_periodic_vision()
            self.stop_heartbeat()
            self.stop_safe_distance()
            self.send_target_ctrl(False)
            self.emit_log("DEMO", "ERROR", "未收到 READY，演示停止。请先确认 soft_reset 和状态机启动。")
            return

        self.vision_stop.clear()
        threading.Thread(target=self._sequence_loop, args=(sequence, period_ms), daemon=True).start()
        self.emit_log("DEMO", "INFO", "开始发送视觉序列")

        if prompt_p000 and self._wait_ra6_event("ALIGN_DONE", 30.0):
            self.emit_log("DEMO", "ACTION", "已对准：请按住 P000 KEY 允许激光输出。")
        if prompt_p000 and self._wait_ra6_event("OUTPUT_ON", 30.0):
            self.emit_log("DEMO", "ACTION", "激光已输出：需要关闭时松开 P000 KEY。")
        if auto_stop and self._wait_ra6_event("OUTPUT_OFF", 60.0):
            self.stop_periodic_vision()
            self.stop_target_ctrl_with_reset()
            self.emit_log("DEMO", "INFO", "检测到 OUTPUT_OFF，演示流程结束。")

    def _sequence_loop(self, sequence: list[tuple[int, int, int]], period_ms: int) -> None:
        for dcx, dcy, count in sequence:
            if self.vision_stop.is_set():
                return
            if count == 0:
                while not self.vision_stop.is_set():
                    self.send_vision_values(dcx, dcy)
                    time.sleep(period_ms / 1000.0)
                return
            for _ in range(count):
                if self.vision_stop.is_set():
                    return
                self.send_vision_values(dcx, dcy)
                time.sleep(period_ms / 1000.0)
        self.vision_stop.set()

    def safety_stop(self) -> None:
        if self.formal_workflow_active:
            self.stop_formal_workflow_demo()
            self.send_arm_command("laser_off")
            self.emit_log("APP", "SAFETY", "比赛全流程已请求 ABORT，并发送 laser_off")
            return
        self.stop_periodic_vision()
        self.stop_heartbeat()
        self.stop_safe_distance()
        self.view_arc_stop.set()
        self.send_arm_command("motion_abort")
        self.send_arm_command("target_disable")
        self.send_arm_command("laser_off")
        self.emit_log("APP", "SAFETY", "已锁存停止运动、停止视觉发送，并关闭定靶和激光")

    def on_serial_data(self, source: str, data: bytes) -> None:
        if source == "ARM":
            self.arm_rx_buffer.extend(data)
            normalized = bytes(self.arm_rx_buffer).replace(b"\r\n", b"\n").replace(b"\r", b"\n")
            parts = normalized.split(b"\n")
            self.arm_rx_buffer = bytearray(parts.pop())
            for raw_line in parts:
                line = raw_line.decode("utf-8", errors="replace").strip()
                if line:
                    self._update_arm_status(line)
                if line and should_display_arm_line(line):
                    self.emit_log("ARM", "RX", line)
        else:
            self.emit_log("JETSON", "RX", bytes_to_hex(data))
            self.jetson_rx_buffer.extend(data)
            frames, remaining = parse_ra6_status_stream(bytes(self.jetson_rx_buffer))
            self.jetson_rx_buffer = bytearray(remaining[-64:])
            for status in frames:
                if status.is_formal:
                    self.formal_status_mailbox.publish(status)
                    self.emit_log(
                        "JETSON",
                        "STATUS",
                        (
                            f"{status.name} SEQ=0x{status.seq:02X} "
                            f"VALUE=0x{status.value:02X} ERROR=0x{status.error:02X}: "
                            f"{bytes_to_hex(status.raw)}"
                        ),
                    )
                else:
                    self.emit_log("JETSON", "STATUS", f"{status.name}: {bytes_to_hex(status.raw)}")
                self._update_ra6_status(status.name)

    def _update_arm_status(self, line: str) -> None:
        lower = line.lower()
        if "soft reset final verify pass" in lower:
            self.arm_events["SOFT_RESET_PASS"].set()
            self._set_status("POSE_VALID", "YES")
        elif "soft reset final verify fail" in lower or "soft reset failed" in lower:
            self.arm_events["SOFT_RESET_FAIL"].set()
            self._set_status("POSE_VALID", "NO")
        elif "pose invalid" in lower or "final fail" in lower:
            self._set_status("POSE_VALID", "NO")
        if "[auto] finished" in lower:
            self.arm_events["AUTO_FINISHED"].set()
        if (
            "reject auto" in lower
            or "[auto] failed" in lower
            or "robot_auto_move_interpolation robot kinematics inverse failed" in lower
            or "scurve path generation failed" in lower
        ):
            self.arm_events["AUTO_FAILED"].set()
        if "[view_arc] finished" in lower:
            self.arm_events["VIEW_ARC_FINISHED"].set()
        if "[view_arc] failed" in lower or "reject view_arc" in lower:
            self.arm_events["VIEW_ARC_FAILED"].set()
        if "motion_abort latched" in lower or "pid run aborted by safety request" in lower:
            self.arm_events["MOTION_ABORTED"].set()
        if "state=confirm" in lower:
            self._set_status("CONFIRMING", "YES", "yellow")
        elif "state=output" in lower or "state=done" in lower or "state=recover" in lower:
            self._set_status("CONFIRMING", "NO")

    def _update_ra6_status(self, name: str) -> None:
        if name in self.ra6_events:
            self.ra6_events[name].set()
        if name == "TARGET_CTRL_ON":
            self._clear_target_runtime_status(clear_target=False)
            self._set_status("TARGET_CTRL", "ON")
        elif name == "TARGET_CTRL_OFF":
            self._set_status("TARGET_CTRL", "OFF")
            self._set_status("READY", "-")
            self._set_status("ALIGN_DONE", "-")
            self._set_status("SAFE_DIST", "-")
            self._set_status("CONFIRMING", "-")
            self._set_status("OUTPUT", "OFF")
            self._set_status("ERROR", "-")
        elif name == "READY":
            self._set_status("READY", "YES")
        elif name == "ALIGN_DONE":
            self._set_status("ALIGN_DONE", "YES")
        elif name == "OUTPUT_ON":
            self._set_status("OUTPUT", "ON")
        elif name == "OUTPUT_OFF":
            self._set_status("OUTPUT", "OFF")
        elif name == "SAFE_DISTANCE_OK":
            if self.ra6_events.get("SAFE_DISTANCE_TOO_CLOSE") is not None:
                self.ra6_events["SAFE_DISTANCE_TOO_CLOSE"].clear()
            self._set_status("SAFE_DIST", "OK")
            self._set_status("ERROR", "-")
        elif name == "SAFE_DISTANCE_TOO_CLOSE":
            self._set_status("SAFE_DIST", "TOO_CLOSE", "red")
            self._set_status("ERROR", "YES")
        elif name.startswith("CAPTURE_POINT_"):
            self._set_status("READY", name.replace("CAPTURE_POINT_", "CAP_"), "green")
            self._set_status("ERROR", "-")
        elif name == "CAPTURE_DONE_HOME":
            self._set_status("POSE_VALID", "YES")
            self._set_status("READY", "HOME", "green")
            self._set_status("ERROR", "-")
        elif name.startswith("TARGET_PRESTART_"):
            self._set_status("READY", name.replace("TARGET_PRESTART_", "PRE_"), "green")
            self._set_status("ERROR", "-")
        elif name in {
            "INVALID_PARAM",
            "BUSY",
            "HEARTBEAT_TIMEOUT",
            "SAFETY_ERROR",
            "VISION_LOST",
            "SOFT_RESET_FAILED",
            "INVALID_STATE",
            "SEQ_CONFLICT",
            "TARGET_GATE_DENIED",
            "MOTION_ABORTED",
        }:
            self._set_status("ERROR", "YES")
        elif name == "ERROR":
            self._set_status("ERROR", "YES")

    def _set_status(self, key: str, value: str, color: str | None = None) -> None:
        if threading.current_thread() is not threading.main_thread():
            self.status_update_queue.put((key, value, color))
            return
        self.status_labels[key].setText(f"{key}: {value}")
        self._set_status_dot(key, color or self._status_color(key, value))

    def _status_color(self, key: str, value: str) -> str:
        if key == "ERROR" and value == "YES":
            return "red"
        if key == "PROTO":
            return "green" if value == "NEW" else "gray"
        if key == "HEARTBEAT":
            return "green" if value == "ON" else "gray"
        if key == "SAFE_DIST":
            if value == "OK":
                return "green"
            if value == "TOO_CLOSE":
                return "red"
        if value in {"YES", "ON"} or value.startswith("已打开"):
            return "green"
        return "gray"

    def _set_status_dot(self, key: str, color: str) -> None:
        colors = {
            "green": "#22c55e",
            "gray": "#9ca3af",
            "red": "#ef4444",
            "yellow": "#f59e0b",
        }
        self.status_dots[key].setStyleSheet(
            f"background-color: {colors[color]}; border-radius: 7px; border: 1px solid #6b7280;"
        )

    def _clear_arm_runtime_status(self) -> None:
        self._set_status("POSE_VALID", "-")

    def _clear_target_runtime_status(self, clear_target: bool) -> None:
        if clear_target:
            self._set_status("TARGET_CTRL", "-")
        for key in ("READY", "ALIGN_DONE", "SAFE_DIST", "CONFIRMING", "OUTPUT", "ERROR"):
            self._set_status(key, "-")

    def _clear_ra6_events(self) -> None:
        for event in self.ra6_events.values():
            event.clear()

    def _wait_ra6_event(self, name: str, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        event = self.ra6_events[name]
        while time.monotonic() < deadline and not self.demo_stop.is_set():
            if self.ra6_events["ERROR"].is_set():
                return False
            if self.ra6_events.get("SAFE_DISTANCE_TOO_CLOSE") is not None and self.ra6_events["SAFE_DISTANCE_TOO_CLOSE"].is_set():
                return False
            if event.wait(0.1):
                return True
        return False

    def clear_visible_log(self) -> None:
        self.log_text.clear()

    def emit_log(self, source: str, kind: str, message: str) -> None:
        line = f"[{dt.datetime.now().strftime('%H:%M:%S.%f')[:-3]}] [{source}-{kind}] {message}"
        self.logger.write(line)
        self.log_queue.put(line)

    def _flush_logs(self) -> None:
        while True:
            try:
                key, value, color = self.status_update_queue.get_nowait()
            except queue.Empty:
                break
            self._set_status(key, value, color)
        while True:
            try:
                stage = self.workflow_stage_queue.get_nowait()
            except queue.Empty:
                break
            self.workflow_stage.setText(stage)
        while True:
            try:
                line = self.log_queue.get_nowait()
            except queue.Empty:
                return
            self.log_text.append(line)

    def closeEvent(self, event) -> None:  # noqa: N802
        if self.formal_workflow_active:
            self.formal_workflow_stop.set()
            self._stop_formal_vision()
            self._send_formal_abort_best_effort()
        self.demo_stop.set()
        if self.view_arc_thread is not None and self.view_arc_thread.is_alive():
            self.stop_view_arc_demo()
        self.stop_periodic_vision()
        self.stop_heartbeat()
        self.stop_safe_distance()
        self.arm.close()
        self.jetson.close()
        event.accept()


def qt_gui_self_test() -> None:
    app = QApplication.instance() or QApplication([])
    window = QtUpperConsole(Path("logs"))
    assert window.windowTitle() == APP_TITLE
    assert "ARM_PORT" in window.status_labels
    assert parse_safe_distance_sequence(window.safe_distance_sequence.text())
    for button in window.findChildren(QPushButton):
        if button.text() == "soft_reset":
            button.click()
            break
    else:
        raise AssertionError("soft_reset button not found")
    window._flush_logs()
    log_text = window.log_text.toPlainText()
    assert "soft_reset" in log_text
    assert "False" not in log_text
    window.close()
    app.processEvents()


def main() -> int:
    parser = argparse.ArgumentParser(description=APP_TITLE)
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--gui-self-test", action="store_true")
    parser.add_argument("--log-dir", type=Path, default=Path.cwd() / "logs")
    args = parser.parse_args()
    if args.self_test:
        protocol_self_test()
        print("upper console self-test passed")
        return 0
    if args.gui_self_test:
        protocol_self_test()
        qt_gui_self_test()
        print("upper console gui self-test passed")
        return 0
    app = QApplication([])
    window = QtUpperConsole(args.log_dir)
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
