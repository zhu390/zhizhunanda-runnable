from pathlib import Path
import sys


DEBUG = Path(__file__).resolve().parent
sys.path.insert(0, str(DEBUG))

from formal_workflow_demo import (  # noqa: E402
    FrameParser,
    MSG_STATUS,
    MSG_WORKFLOW_CTRL,
    build_frame,
    crc16_modbus,
)


def test_crc_vectors_match_protocol_document() -> None:
    assert crc16_modbus(bytes.fromhex("01 06 10 01 01")) == 0xDDD8
    assert build_frame(MSG_WORKFLOW_CTRL, 0x10, b"\x01") == bytes.fromhex(
        "A5 5A 01 06 10 01 01 D8 DD"
    )


def test_parser_handles_noise_split_and_concatenated_frames() -> None:
    parser = FrameParser()
    status = build_frame(MSG_STATUS, 0x10, bytes.fromhex("21 01 00"))
    workflow = build_frame(MSG_STATUS, 0x10, bytes.fromhex("20 01 00"))

    assert parser.feed(b"\x00\xA5") == []
    frames = parser.feed(status[1:] + workflow)
    assert [(frame.msg_type, frame.seq, frame.payload) for frame in frames] == [
        (MSG_STATUS, 0x10, bytes.fromhex("21 01 00")),
        (MSG_STATUS, 0x10, bytes.fromhex("20 01 00")),
    ]


def test_parser_drops_bad_crc_without_losing_next_frame() -> None:
    parser = FrameParser()
    bad = bytearray(build_frame(MSG_STATUS, 0x20, bytes.fromhex("04 01 00")))
    bad[-1] ^= 0x01
    good = build_frame(MSG_STATUS, 0x21, bytes.fromhex("01 01 00"))

    frames = parser.feed(bytes(bad) + good)
    assert len(frames) == 1
    assert frames[0].seq == 0x21
    assert frames[0].payload == bytes.fromhex("01 01 00")


if __name__ == "__main__":
    test_crc_vectors_match_protocol_document()
    test_parser_handles_noise_split_and_concatenated_frames()
    test_parser_drops_bad_crc_without_losing_next_frame()
    print("formal workflow demo protocol checks passed")
