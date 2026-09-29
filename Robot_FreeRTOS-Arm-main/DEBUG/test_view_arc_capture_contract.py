from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[1]
ROBOT = (ROOT / "APP" / "robot.c").read_text(encoding="utf-8")
CAPTURE = (ROOT / "APP" / "robot_capture.c").read_text(encoding="utf-8")


def macro_value(name: str) -> float:
    match = re.search(rf"#define\s+{name}\s+\((-?\d+(?:\.\d+)?)f\)", ROBOT)
    assert match is not None, name
    return float(match.group(1))


def capture_joint_value(array_name: str, joint_id: int) -> float:
    start = CAPTURE.index(f"static const capture_step_t {array_name}[]")
    end = CAPTURE.index("};", start)
    body = CAPTURE[start:end]
    match = re.search(
        rf"CAPTURE_STEP_ABS,\s+{joint_id}u,\s+(-?\d+(?:\.\d+)?)f",
        body,
    )
    assert match is not None, f"{array_name} joint {joint_id}"
    return float(match.group(1))


def test_view_arc_poses_match_formal_capture_calibration() -> None:
    cases = (
        ("LEFT", "s_left_steps", 0, "J0"),
        ("LEFT", "s_left_steps", 3, "J3"),
        ("LEFT", "s_left_steps", 4, "J4"),
        ("FRONT", "s_front_steps", 0, "J0"),
        ("FRONT", "s_front_steps", 4, "J4"),
        ("RIGHT", "s_right_steps", 0, "J0"),
        ("RIGHT", "s_right_steps", 4, "J4"),
    )
    for pose, array_name, joint_id, label in cases:
        assert macro_value(f"ROBOT_VIEW_ARC_{pose}_{label}_DEG") == capture_joint_value(
            array_name, joint_id
        )

    # Capture uses wrapped 0/35 degrees; the arc deliberately unfolds them to 360/395.
    assert macro_value("ROBOT_VIEW_ARC_FRONT_J3_DEG") == (
        capture_joint_value("s_front_steps", 3) + 360.0
    )
    assert macro_value("ROBOT_VIEW_ARC_RIGHT_J3_DEG") == (
        capture_joint_value("s_right_steps", 3) + 360.0
    )


if __name__ == "__main__":
    test_view_arc_poses_match_formal_capture_calibration()
    print("view arc capture contract checks passed")
