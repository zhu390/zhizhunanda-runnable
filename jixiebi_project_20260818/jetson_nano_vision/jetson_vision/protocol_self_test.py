"""Self-test for the RA6M5 unified vision serial protocol."""

from .vision_protocol import (
    MSG_ERROR,
    MSG_STATUS,
    Ra6StatusParser,
    build_unified_frame,
    pack_ra6_status,
    parse_unified_frame,
    pack_unified_capture_ctrl,
    pack_unified_heartbeat,
    pack_unified_safe_distance,
    pack_unified_target_ctrl,
    pack_unified_vision_error,
    pack_unified_workflow_ctrl,
    unified_frame_to_status,
)


def hex_bytes(data: bytes) -> str:
    return " ".join("{:02X}".format(byte) for byte in data)


def require_equal(name: str, actual: bytes, expected_hex: str) -> None:
    expected = bytes.fromhex(expected_hex)
    if actual != expected:
        raise AssertionError(
            "{} mismatch\nactual:   {}\nexpected: {}".format(
                name,
                hex_bytes(actual),
                hex_bytes(expected),
            )
        )


def main() -> int:
    cases = [
        (
            "HEARTBEAT",
            pack_unified_heartbeat(1, 1234),
            "A5 5A 01 01 01 04 D2 04 00 00 19 AF",
        ),
        (
            "WORKFLOW START",
            pack_unified_workflow_ctrl(0x10, 0x01),
            "A5 5A 01 06 10 01 01 D8 DD",
        ),
        (
            "TARGET_CTRL start",
            pack_unified_target_ctrl(2, True),
            "A5 5A 01 02 02 01 01 79 E8",
        ),
        (
            "TARGET_CTRL stop",
            pack_unified_target_ctrl(3, False),
            "A5 5A 01 02 03 01 00 E9 E8",
        ),
        (
            "VISION_ERROR",
            pack_unified_vision_error(4, -7, -50, True),
            "A5 5A 01 03 04 05 F9 FF CE FF 01 4F 2F",
        ),
        (
            "SAFE_DISTANCE",
            pack_unified_safe_distance(5, 120, True),
            "A5 5A 01 05 05 03 78 00 01 86 08",
        ),
        (
            "CAPTURE_CTRL move",
            pack_unified_capture_ctrl(6, 0x01, 1),
            "A5 5A 01 04 06 02 01 01 91 12",
        ),
        (
            "CAPTURE_CTRL home",
            pack_unified_capture_ctrl(7, 0x02, 0),
            "A5 5A 01 04 07 02 02 00 51 DE",
        ),
        (
            "CAPTURE_CTRL select",
            pack_unified_capture_ctrl(8, 0x03, 3),
            "A5 5A 01 04 08 02 03 03 13 5B",
        ),
    ]

    for name, actual, expected in cases:
        require_equal(name, actual, expected)
        parsed = parse_unified_frame(actual)
        print("{} OK: {}".format(name, hex_bytes(actual)))
        assert parsed.seq == actual[4]

    status = build_unified_frame(MSG_STATUS, 9, bytes([0x01, 0x01, 0x00]))
    require_equal("STATUS TARGET_READY", status, "A5 5A 01 81 09 03 01 01 00 99 C4")
    parsed_status = unified_frame_to_status(parse_unified_frame(status))
    assert parsed_status is not None
    assert parsed_status.label == "TARGET_READY"

    error = build_unified_frame(MSG_ERROR, 10, bytes([0x07]))
    require_equal("ERROR HEARTBEAT_TIMEOUT", error, "A5 5A 01 FE 0A 01 07 48 78")
    parsed_error = unified_frame_to_status(parse_unified_frame(error))
    assert parsed_error is not None
    assert parsed_error.label == "ERROR_HEARTBEAT_TIMEOUT"

    parser = Ra6StatusParser(allow_legacy=True)
    mixed = b"\x00\x99" + pack_ra6_status(0x01, 0x01) + b"\x12" + status + error
    statuses = parser.feed(mixed)
    labels = [item.label for item in statuses]
    assert labels == ["READY", "TARGET_READY", "ERROR_HEARTBEAT_TIMEOUT"], labels
    print("mixed stream parser OK:", labels)

    strict_parser = Ra6StatusParser()
    split = build_unified_frame(MSG_STATUS, 0x10, bytes([0x21, 0x01, 0x00]))
    assert strict_parser.feed(pack_ra6_status(0x01, 0x01) + split[:4]) == []
    split_statuses = strict_parser.feed(split[4:])
    assert [item.label for item in split_statuses] == ["COMMAND_ACK_ACCEPTED"]
    print("strict split-frame parser OK")

    resync_parser = Ra6StatusParser()
    false_header = bytes.fromhex("A5 5A 02 99 00 20")
    resynced = resync_parser.feed(false_header + status)
    assert [item.label for item in resynced] == ["TARGET_READY"]
    print("bad-version resynchronization OK")

    invalid_status = build_unified_frame(MSG_STATUS, 11, bytes([0x21, 0x02, 0x00]))
    assert Ra6StatusParser().feed(invalid_status) == []
    print("invalid business value rejected OK")

    print("all protocol self-tests passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
