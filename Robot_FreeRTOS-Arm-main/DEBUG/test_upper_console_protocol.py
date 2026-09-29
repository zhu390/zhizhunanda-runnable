from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from UpperComputer.ra6m5_protocol import (  # noqa: E402
    EVENT_COMMAND_ACK,
    JETSON_MSG_STATUS,
    JETSON_MSG_WORKFLOW_CTRL,
    WORKFLOW_START,
    build_arm_command,
    build_target_control_frame,
    build_unified_frame,
    build_unified_workflow_control_frame,
    build_vision_error_frame,
    parse_ra6_status_frames,
    parse_vision_sequence,
)


def test_build_vision_error_frame_uses_little_endian_and_checksum() -> None:
    frame = build_vision_error_frame(-7, -50)
    assert frame == bytes.fromhex("FF 05 03 F9 FF CE FF CD FE")


def test_build_target_control_frame() -> None:
    assert build_target_control_frame(True) == bytes.fromhex("AA 01 01 BB")
    assert build_target_control_frame(False) == bytes.fromhex("AA 01 00 BB")


def test_parse_ra6_status_frames_skips_noise() -> None:
    frames = parse_ra6_status_frames(bytes.fromhex("00 CC 04 01 DD 99 CC 03 00 DD"))
    assert [(item.func, item.value, item.name) for item in frames] == [
        (0x04, 0x01, "TARGET_CTRL_ON"),
        (0x03, 0x00, "OUTPUT_OFF"),
    ]


def test_build_arm_command_appends_crlf() -> None:
    assert build_arm_command("soft_reset") == b"soft_reset\r\n"
    assert build_arm_command(" auto 0 -50 0 ") == b"auto 0 -50 0\r\n"


def test_parse_vision_sequence() -> None:
    assert parse_vision_sequence("15,-10:2;8,-5:1;0,0:0") == [
        (15, -10, 2),
        (8, -5, 1),
        (0, 0, 0),
    ]


def test_build_workflow_control_frame() -> None:
    assert build_unified_workflow_control_frame(WORKFLOW_START, 0x22) == build_unified_frame(
        JETSON_MSG_WORKFLOW_CTRL,
        0x22,
        b"\x01",
    )


def test_formal_status_keeps_seq_event_value_and_error() -> None:
    raw = build_unified_frame(JETSON_MSG_STATUS, 0x31, bytes([EVENT_COMMAND_ACK, 0x01, 0x00]))
    status = parse_ra6_status_frames(raw)[0]
    assert (status.seq, status.event, status.value, status.error) == (0x31, EVENT_COMMAND_ACK, 0x01, 0x00)
    assert status.name == "COMMAND_ACK"


def test_formal_error_keeps_async_seq_and_error_code() -> None:
    raw = build_unified_frame(0xFE, 0x00, b"\x08")
    status = parse_ra6_status_frames(raw)[0]
    assert status.seq == 0x00
    assert status.error == 0x08
    assert status.name == "SAFETY_ERROR"


def test_formal_workflow_fault_and_retreat_states_have_names() -> None:
    fault = parse_ra6_status_frames(
        build_unified_frame(JETSON_MSG_STATUS, 0x40, bytes([0x20, 0x07, 0x06]))
    )[0]
    retreat = parse_ra6_status_frames(
        build_unified_frame(JETSON_MSG_STATUS, 0x41, bytes([0x20, 0x08, 0x00]))
    )[0]
    assert fault.name == "WORKFLOW_FAULT_HOLD"
    assert retreat.name == "WORKFLOW_RETREAT_STEP_READY"


if __name__ == "__main__":
    test_build_vision_error_frame_uses_little_endian_and_checksum()
    test_build_target_control_frame()
    test_parse_ra6_status_frames_skips_noise()
    test_build_arm_command_appends_crlf()
    test_parse_vision_sequence()
    test_build_workflow_control_frame()
    test_formal_status_keeps_seq_event_value_and_error()
    test_formal_error_keeps_async_seq_and_error_code()
    test_formal_workflow_fault_and_retreat_states_have_names()
    print("upper console protocol checks passed")
