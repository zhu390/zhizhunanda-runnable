from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
ROBOT_H = (ROOT / "APP" / "robot.h").read_text(encoding="utf-8")
ROBOT_C = (ROOT / "APP" / "robot.c").read_text(encoding="utf-8")
TARGET_H = (ROOT / "APP" / "robot_target.h").read_text(encoding="utf-8")
TARGET_C = (ROOT / "APP" / "robot_target.c").read_text(encoding="utf-8")


def require(text: str, needle: str, label: str) -> None:
    assert needle in text, f"missing {label}: {needle}"


def test_target_first_stage_params() -> None:
    require(TARGET_H, "TARGET_ALIGN_TOL_PX_COARSE", "coarse tolerance")
    require(TARGET_H, "TARGET_MAX_STEP_MM_FINE", "fine step")
    require(TARGET_H, "TARGET_CONFIRM_STABLE_COUNT", "confirm stable count")


def test_auto_busy_is_cartesian_only() -> None:
    require(ROBOT_H, "ROBOT_STATUS_AUTO_BUSY", "auto busy status")
    require(ROBOT_H, "bool robot_is_auto_busy(void);", "auto busy API")
    require(ROBOT_C, "ROBOT_STATUS_SET(g_robot.status, ROBOT_STATUS_AUTO_BUSY)", "auto busy set")
    require(ROBOT_C, "ROBOT_STATUS_CLEAR(g_robot.status, ROBOT_STATUS_AUTO_BUSY)", "auto busy clear")
    require(TARGET_C, "robot_is_auto_busy()", "target align busy gate")
    assert "align_move_pending" not in TARGET_C
    assert "TARGET_ALIGN_MOVE_TIMEOUT" not in TARGET_H + TARGET_C


def test_target_confirm_requires_new_frames() -> None:
    require(TARGET_C, "confirm_stable_count", "confirm counter")
    require(TARGET_C, "TARGET_CONFIRM_STABLE_COUNT", "confirm threshold use")
    require(TARGET_C, "new_vision &&", "new frame-gated confirm")


def test_alignment_counts_reset_together() -> None:
    require(TARGET_C, "static void reset_alignment_counts(void)", "shared counter reset helper")
    assert TARGET_C.count("s_target.stable_count = 0u;") == 1
    assert TARGET_C.count("s_target.confirm_stable_count = 0u;") == 1
    require(TARGET_C, "reset_alignment_counts();", "paired counter reset calls")


if __name__ == "__main__":
    tests = [
        test_target_first_stage_params,
        test_auto_busy_is_cartesian_only,
        test_target_confirm_requires_new_frames,
        test_alignment_counts_reset_together,
    ]
    for test in tests:
        test()
    print("4.12 static checks passed")
