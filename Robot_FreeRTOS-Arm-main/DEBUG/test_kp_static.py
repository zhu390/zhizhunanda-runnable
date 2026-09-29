from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[1]
ROBOT_C = (ROOT / "APP" / "robot.c").read_text(encoding="utf-8")
DOC = (ROOT / "控制文档.md").read_text(encoding="utf-8")

EXPECTED_KP = "{0.65f, 3.00f, 3.00f, 2.00f, 2.30f, 10.0f}"
EXPECTED_DOC_KP = "{0.65, 3.00, 3.00, 2.00, 2.30, 10.0}"


def test_robot_joint_kp_matches_tuned_values() -> None:
    match = re.search(r"ROBOT_JOINT_KP\[ROBOT_MAX_JOINT_NUM\]\s*=\s*(\{[^;]+\})", ROBOT_C)
    assert match, "ROBOT_JOINT_KP definition not found"
    assert match.group(1) == EXPECTED_KP


def test_control_doc_records_current_kp_values() -> None:
    assert EXPECTED_DOC_KP in DOC


if __name__ == "__main__":
    test_robot_joint_kp_matches_tuned_values()
    test_control_doc_records_current_kp_values()
    print("Kp static checks passed")
