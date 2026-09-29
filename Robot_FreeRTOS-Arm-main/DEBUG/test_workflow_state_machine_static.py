from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read(relative: str) -> str:
    path = ROOT / relative
    assert path.exists(), f"missing required file: {relative}"
    return path.read_text(encoding="utf-8")


def require(text: str, needle: str, label: str) -> None:
    assert needle in text, f"missing {label}: {needle}"


def test_motion_abort_is_latched_and_generation_guarded() -> None:
    robot_h = read("APP/robot.h")
    robot_c = read("APP/robot.c")

    require(robot_h, "uint32_t generation;", "robot event generation")
    require(robot_h, "ROBOT_RESET_RESULT_ABORTED", "reset aborted result")
    require(robot_h, "bool robot_motion_abort_latched(void);", "abort latch query API")
    require(robot_c, "g_robot_motion_generation++", "abort generation increment")
    require(robot_c, "robot_event_is_stale", "stale queued event guard")
    require(robot_c, "soft reset aborted by safety request", "soft reset abort path")
    require(robot_c, "hard reset joint %u timeout", "hard reset timeout")

    pid_abort = robot_c.index("robot pid run aborted by safety request")
    pid_window = robot_c[pid_abort : pid_abort + 220]
    assert "robot_motion_abort_clear" not in pid_window, "PID must not clear the safety latch"


def test_formal_protocol_has_workflow_seq_and_explicit_status() -> None:
    vision_h = read("Middle/jetson_vision.h")
    vision_c = read("Middle/jetson_vision.c")

    require(vision_h, "JETSON_MSG_WORKFLOW_CTRL", "workflow message type")
    require(vision_h, "JETSON_WORKFLOW_ACTION_START_MEASURE", "workflow start action")
    require(vision_h, "JETSON_WORKFLOW_ACTION_FINISH_RETURN_HOME", "workflow finish action")
    require(vision_h, "JETSON_WORKFLOW_ACTION_ABORT_HOLD", "workflow abort action")
    require(vision_h, "jetson_get_workflow_control", "workflow getter")
    require(vision_h, "jetson_get_vision_error_ex", "vision valid/seq getter")
    require(vision_h, "jetson_send_status(", "explicit status API")
    require(vision_h, "jetson_send_error(", "explicit error API")
    require(vision_c, "s_heartbeat_seen", "heartbeat-seen latch")
    require(vision_c, "mark_heartbeat_alive", "heartbeat-only refresh helper")


def test_top_level_workflow_owns_competition_flow() -> None:
    workflow_h = read("APP/robot_workflow.h")
    workflow_c = read("APP/robot_workflow.c")
    project = read("Robot_FreeRTOS.uvprojx")

    for state in (
        "FLOW_IDLE",
        "FLOW_HOMING",
        "FLOW_MEASURE_POSITION",
        "FLOW_WAIT_SAFE_DISTANCE",
        "FLOW_SAFE_READY",
        "FLOW_CAPTURE",
        "FLOW_CAPTURE_HOME",
        "FLOW_SELECTED_VIEW",
        "FLOW_TARGET_ACTIVE",
        "FLOW_TARGET_HOLD",
        "FLOW_RETURN_HOME",
        "FLOW_RETREAT_WAIT_RESTART",
        "FLOW_FAULT_HOLD",
    ):
        require(workflow_h, state, f"workflow state {state}")

    require(workflow_h, "ROBOT_WORKFLOW_MEASURE_Y", "public measurement point")
    require(workflow_h, "ROBOT_WORKFLOW_RETREAT_STEP_MM", "retreat step")
    require(workflow_h, "ROBOT_WORKFLOW_RETREAT_MAX_STEPS", "retreat limit")
    require(workflow_h, "ROBOT_WORKFLOW_RETREAT_WAIT_MS", "retreat distance timeout")
    require(workflow_c, "distance_safe_latched", "distance safety latch")
    require(workflow_c, "command_cache", "control command deduplication")
    require(workflow_c, "BSP_Laser_On();", "single workflow laser-on outlet")
    require(project, "robot_workflow.c", "workflow C file in Keil project")
    require(project, "robot_workflow.h", "workflow header in Keil project")

    laser_callers = []
    for folder in ("APP", "Middle", "src"):
        for path in (ROOT / folder).rglob("*.c"):
            if "BSP_Laser_On();" in path.read_text(encoding="utf-8"):
                laser_callers.append(path.relative_to(ROOT).as_posix())
    assert laser_callers == ["APP/robot_workflow.c"], \
        f"laser-on must have one workflow caller, got: {laser_callers}"


def test_capture_and_target_are_substate_executors() -> None:
    capture_h = read("APP/robot_capture.h")
    capture_c = read("APP/robot_capture.c")
    target_h = read("APP/robot_target.h")
    target_c = read("APP/robot_target.c")

    require(capture_h, "robot_capture_result_consume", "capture result API")
    require(capture_h, "robot_capture_cancel", "capture cancel API")
    assert "jetson_send_status" not in capture_c, "capture must not own protocol status"
    assert "BSP_Laser_On" not in capture_c, "capture must never turn on the laser"

    require(target_h, "TARGET_OUTPUT_MAX_MS", "10 second output ceiling")
    require(target_h, "robot_target_event_consume", "target event API")
    require(target_h, "robot_target_output_requested", "target output request API")
    assert "jetson_send_status" not in target_c, "target must not own protocol status"
    assert "BSP_Laser_On" not in target_c, "target must request, not apply, laser output"
    assert "robot_send_reset_event" not in target_c, "target STOP_HOLD must not reset"


def test_vision_thread_has_one_business_dispatcher() -> None:
    service = read("src/vision_service_thread_entry.c")

    require(service, '#include "robot_workflow.h"', "workflow include")
    require(service, "robot_workflow_init();", "workflow initialization")
    require(service, "robot_workflow_step(&obs);", "workflow service call")
    assert "robot_capture_step" not in service, "vision thread must not dispatch capture directly"
    assert "robot_target_step" not in service, "vision thread must not dispatch target directly"


def test_competition_safety_parameters_are_final() -> None:
    workflow_h = read("APP/robot_workflow.h")
    target_h = read("APP/robot_target.h")

    require(workflow_h, "ROBOT_WORKFLOW_SAFE_DISTANCE_MM       (150u)",
            "150 mm safety threshold")
    require(workflow_h, "ROBOT_WORKFLOW_RETREAT_STEP_MM        (20.0f)",
            "20 mm retreat step")
    require(workflow_h, "ROBOT_WORKFLOW_RETREAT_MAX_STEPS      (5u)",
            "five-step retreat limit")
    require(target_h, "TARGET_CONFIRM_STABLE_COUNT  (2u)",
            "two-frame secondary alignment confirmation")


def test_jetson_rx_uses_gapless_ring_buffer() -> None:
    vision_h = read("Middle/jetson_vision.h")
    vision_c = read("Middle/jetson_vision.c")
    uart_c = read("USER_BSP/bsp_uart.c")

    require(vision_h, "jetson_notify_rx_char_from_isr", "RX ISR byte API")
    require(vision_c, "JETSON_RX_RING_SIZE", "Jetson RX ring")
    require(vision_c, "parser_resync", "parser resynchronization helper")
    require(uart_c, "jetson_notify_rx_char_from_isr((uint8_t)p_args->data)",
            "RX_CHAR forwarding")
    assert "R_SCI_UART_Read" not in vision_c, "Jetson RX must not rearm a finite DTC read"
    assert "JETSON_TRANSFER_INSTANCE" not in vision_c, "Jetson RX must not poll DTC position"


def test_safe_distance_and_command_seq_guards_exist() -> None:
    workflow_c = read("APP/robot_workflow.c")

    require(workflow_c, "WORKFLOW_COMMAND_CACHE_TTL_MS", "command cache TTL")
    require(workflow_c, "safe_sample_seqs", "all safe-sample SEQs")
    require(workflow_c, "workflow_safe_seq_seen", "safe SEQ duplicate check")
    require(workflow_c, "obs->distance_seq == 0u", "zero SAFE SEQ rejection")


if __name__ == "__main__":
    test_motion_abort_is_latched_and_generation_guarded()
    test_formal_protocol_has_workflow_seq_and_explicit_status()
    test_top_level_workflow_owns_competition_flow()
    test_capture_and_target_are_substate_executors()
    test_vision_thread_has_one_business_dispatcher()
    test_competition_safety_parameters_are_final()
    test_jetson_rx_uses_gapless_ring_buffer()
    test_safe_distance_and_command_seq_guards_exist()
    print("workflow state-machine static checks passed")
