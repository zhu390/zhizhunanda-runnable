import unittest

from jetson_nano_vision.jetson_vision.vision_protocol import (
    ERROR_SAFE_DISTANCE_TIMEOUT,
    MSG_STATUS,
    MSG_WORKFLOW_CTRL,
    STATUS_SAFE_DISTANCE,
    WORKFLOW_ACTION_START_MEASURE_CONTINUOUS,
    build_unified_frame,
    describe_unified_error_code,
    pack_unified_workflow_ctrl,
    parse_unified_frame,
    unified_frame_to_status,
)


class ContinuousSafetyProtocolTest(unittest.TestCase):
    def test_continuous_start_action_round_trips(self):
        frame = pack_unified_workflow_ctrl(
            23,
            WORKFLOW_ACTION_START_MEASURE_CONTINUOUS,
        )
        parsed = parse_unified_frame(frame)
        self.assertEqual(parsed.msg_type, MSG_WORKFLOW_CTRL)
        self.assertEqual(parsed.seq, 23)
        self.assertEqual(
            parsed.payload,
            bytes([WORKFLOW_ACTION_START_MEASURE_CONTINUOUS]),
        )

    def test_safe_distance_timeout_status_is_parsed_and_named(self):
        frame = build_unified_frame(
            MSG_STATUS,
            71,
            bytes([STATUS_SAFE_DISTANCE, 0, ERROR_SAFE_DISTANCE_TIMEOUT]),
        )
        parsed = unified_frame_to_status(parse_unified_frame(frame))
        self.assertIsNotNone(parsed)
        self.assertEqual(parsed.error_code, ERROR_SAFE_DISTANCE_TIMEOUT)
        self.assertEqual(
            describe_unified_error_code(ERROR_SAFE_DISTANCE_TIMEOUT),
            "SAFE_DISTANCE_TIMEOUT",
        )


if __name__ == "__main__":
    unittest.main()
