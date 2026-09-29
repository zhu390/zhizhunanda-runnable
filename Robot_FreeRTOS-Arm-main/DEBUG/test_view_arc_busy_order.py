from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SOURCE = (ROOT / "APP" / "robot.c").read_text(encoding="utf-8")


def body_between(start_marker: str, end_marker: str) -> str:
    start = SOURCE.index(start_marker)
    end = SOURCE.index(end_marker, start + len(start_marker))
    return SOURCE[start:end]


def test_auto_success_marker_is_after_busy_clear() -> None:
    body = body_between(
        "static void robot_auto_move_interpolation(struct robot_event *event)",
        "static float robot_angle_normalize",
    )
    final_confirm = body.index("robot_auto_final_confirm")
    busy_clear = body.index("robot_auto_busy_clear();", final_confirm)
    success_marker = body.index('LOG("[AUTO] finished', final_confirm)
    assert success_marker > busy_clear


def test_view_arc_success_marker_is_after_busy_clear() -> None:
    body = body_between(
        "static int robot_view_arc_move(uint32_t duration_ms)",
        "static void robot_joints_sync_to",
    )
    busy_clear = body.rindex("robot_auto_busy_clear();")
    success_marker = body.index('LOG("[VIEW_ARC] finished')
    assert success_marker > busy_clear


if __name__ == "__main__":
    test_auto_success_marker_is_after_busy_clear()
    test_view_arc_success_marker_is_after_busy_clear()
    print("view arc busy ordering checks passed")
