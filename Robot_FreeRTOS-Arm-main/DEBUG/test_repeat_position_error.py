from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
DEBUG = ROOT / "DEBUG"
sys.path.insert(0, str(DEBUG))

from repeat_position_error_test import (  # noqa: E402
    circular_diff_deg,
    parse_ave_error_line,
    parse_last_result_angles,
    parse_read_all_angles,
    summarize_samples,
)


SAMPLE_LOG = """
[245] <22.98 -122.53 -15.09> result: 97.69 118.74 -52.38 -7.78 81.21 0.00
[266] <23.15 -123.45 -15.20> result: 97.71 118.92 -52.08 -7.80 81.07 0.00
[joint 1] ave_error:0.09
[joint 2] ave_error:1.88
[joint 3] ave_error:2.03
[joint 4] ave_error:0.04
[joint 5] ave_error:0.64
robot pid run finished!!
read_all ret=0 elapsed=3 ms missing=0x00
J1 angle=97.64
J2 angle=118.71
J3 angle=307.27
J4 angle=352.22
J5 angle=81.5
"""


def test_parse_last_result_angles_uses_final_trajectory_result() -> None:
    assert parse_last_result_angles(SAMPLE_LOG.splitlines()) == [97.71, 118.92, -52.08, -7.80, 81.07]


def test_parse_read_all_angles_collects_j1_to_j5() -> None:
    assert parse_read_all_angles(SAMPLE_LOG.splitlines()) == [97.64, 118.71, 307.27, 352.22, 81.5]


def test_circular_diff_handles_wraparound_angles() -> None:
    assert abs(circular_diff_deg(307.27, -52.08) - (-0.65)) < 0.01
    assert abs(circular_diff_deg(352.22, -7.80) - 0.02) < 0.01


def test_parse_ave_error_line() -> None:
    values = {}
    for line in SAMPLE_LOG.splitlines():
        parsed = parse_ave_error_line(line)
        if parsed:
            joint_index, value = parsed
            values[joint_index] = value
    assert values == {1: 0.09, 2: 1.88, 3: 2.03, 4: 0.04, 5: 0.64}


def test_summarize_samples_reports_repetition_stddev() -> None:
    samples = [
        [97.64, 118.71, 307.27, 352.22, 81.50],
        [97.66, 118.75, 307.20, 352.18, 81.45],
        [97.62, 118.73, 307.31, 352.20, 81.55],
    ]
    summary = summarize_samples(samples)
    assert summary[0].mean_angle > 97.63
    assert summary[0].stddev_deg > 0.0
    assert summary[2].range_deg > 0.0


if __name__ == "__main__":
    test_parse_last_result_angles_uses_final_trajectory_result()
    test_parse_read_all_angles_collects_j1_to_j5()
    test_circular_diff_handles_wraparound_angles()
    test_parse_ave_error_line()
    test_summarize_samples_reports_repetition_stddev()
    print("repeat position parser checks passed")
