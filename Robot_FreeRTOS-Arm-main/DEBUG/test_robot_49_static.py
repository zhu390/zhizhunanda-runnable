from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[1]
ROBOT_C = ROOT / "APP" / "robot.c"
DOC = ROOT / "控制文档.md"


def _read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def _function_body(source: str, name: str) -> str:
    match = re.search(rf"static\s+\w+\s+{name}\s*\([^)]*\)\s*\{{", source)
    assert match, f"{name} not found"
    start = match.end()
    depth = 1
    idx = start
    while idx < len(source) and depth:
        if source[idx] == "{":
            depth += 1
        elif source[idx] == "}":
            depth -= 1
        idx += 1
    assert depth == 0, f"{name} body not closed"
    return source[start:idx - 1]


def test_single_joint_commands_wait_for_position_confirm() -> None:
    source = _read(ROBOT_C)
    for event_name in ("ROBOT_JOINT_REL_ROTATE", "ROBOT_JOINT_ABS_ROTATE"):
        body = source.split(f"case {event_name}:", 1)[1].split("break;", 1)[0]
        assert "robot_joint_wait_target" in body
        assert "robot_joint_stop" in body


def test_control_doc_records_fixed_settle_window() -> None:
    doc = _read(DOC)
    assert "ROBOT_PID_SETTLE_PERIODS`（10，通用终点稳定）均保留不变" not in doc
    assert "`ROBOT_PID_SETTLE_PERIODS=10`" in doc
    assert "`100ms`" in doc


def test_time_func_uses_final_position_confirm() -> None:
    source = _read(ROBOT_C)
    body = _function_body(source, "robot_time_func_move")
    assert "robot_auto_final_confirm" in body
    assert "ROBOT_STATUS_POSE_DEGRADED" in body


if __name__ == "__main__":
    test_single_joint_commands_wait_for_position_confirm()
    test_control_doc_records_fixed_settle_window()
    test_time_func_uses_final_position_confirm()
    print("robot 4.9 static checks passed")
