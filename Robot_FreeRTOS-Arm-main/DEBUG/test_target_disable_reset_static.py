from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
ROBOT_TARGET_C = (ROOT / "APP" / "robot_target.c").read_text(encoding="utf-8")
ROBOT_WORKFLOW_C = (ROOT / "APP" / "robot_workflow.c").read_text(encoding="utf-8")


def _function_body(name: str) -> str:
    start = ROBOT_TARGET_C.index(f"void {name}(void)")
    next_func = ROBOT_TARGET_C.index("\nvoid ", start + 1)
    return ROBOT_TARGET_C[start:next_func]


def test_target_disable_holds_without_reset() -> None:
    body = _function_body("robot_target_disable_request")
    assert "robot_target_stop_hold();" in body
    assert "robot_send_reset_event" not in body
    assert "JETSON_WORKFLOW_ACTION_FINISH_RETURN_HOME" in ROBOT_WORKFLOW_C
    assert "workflow_begin_home(obs->now_ms, FLOW_RETURN_HOME)" in ROBOT_WORKFLOW_C


if __name__ == "__main__":
    test_target_disable_holds_without_reset()
    print("target disable hold static check passed")
