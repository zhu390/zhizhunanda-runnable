from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read(relative: str) -> str:
    return (ROOT / relative).read_text(encoding="utf-8")


def function_body(source: str, start_marker: str, end_marker: str) -> str:
    start = source.index(start_marker)
    end = source.index(end_marker, start + len(start_marker))
    return source[start:end]


def test_view_arc_event_and_uart_commands_are_wired() -> None:
    robot_h = read("APP/robot.h")
    robot_c = read("APP/robot.c")
    cmd_c = read("Middle/robot_cmd.c")

    assert "ROBOT_VIEW_ARC_EVENT" in robot_h
    assert "int robot_send_view_arc_event(float duration_ms);" in robot_h
    assert "case ROBOT_VIEW_ARC_EVENT:" in robot_c
    assert '{"view_arc",' in cmd_c
    assert '{"motion_abort",' in cmd_c


def test_view_arc_uses_calibrated_views_and_continuous_j3_angles() -> None:
    robot_c = read("APP/robot.c")
    required_constants = [
        "ROBOT_VIEW_ARC_LEFT_J0_DEG     (65.0f)",
        "ROBOT_VIEW_ARC_FRONT_J0_DEG    (90.0f)",
        "ROBOT_VIEW_ARC_RIGHT_J0_DEG    (115.0f)",
        "ROBOT_VIEW_ARC_LEFT_J3_DEG     (330.0f)",
        "ROBOT_VIEW_ARC_FRONT_J3_DEG    (360.0f)",
        "ROBOT_VIEW_ARC_RIGHT_J3_DEG    (395.0f)",
        "ROBOT_VIEW_ARC_LEFT_J4_DEG     (85.0f)",
        "ROBOT_VIEW_ARC_FRONT_J4_DEG    (80.0f)",
        "ROBOT_VIEW_ARC_RIGHT_J4_DEG    (80.0f)",
    ]
    for constant in required_constants:
        assert constant in robot_c

    assert "ROBOT_VIEW_ARC_DURATION_MAX_MS (8000U)" in robot_c
    assert "ROBOT_VIEW_ARC_ENTRY_DURATION_MS (2000U)" in robot_c
    assert "robot_view_arc_bezier" in robot_c
    assert "robot_pid_run(NULL, path_size, s_result_buf)" in robot_c


def test_view_arc_marks_cartesian_pose_invalid_and_keeps_capture_isolated() -> None:
    robot_c = read("APP/robot.c")
    capture_c = read("APP/robot_capture.c")
    body = function_body(
        robot_c,
        "static int robot_view_arc_move(uint32_t duration_ms)",
        "static void robot_joints_sync_to",
    )
    assert "ROBOT_STATUS_CLEAR(g_robot.status, ROBOT_STATUS_POSE_VALID);" in body
    assert "ROBOT_STATUS_SET(g_robot.status, ROBOT_STATUS_POSE_DEGRADED);" in body
    assert "robot_auto_busy_clear();" in body
    assert "view_arc" not in capture_c.lower()


if __name__ == "__main__":
    test_view_arc_event_and_uart_commands_are_wired()
    test_view_arc_uses_calibrated_views_and_continuous_j3_angles()
    test_view_arc_marks_cartesian_pose_invalid_and_keeps_capture_isolated()
    print("view arc static checks passed")
