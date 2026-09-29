"""Static C structure guards for the visual-servo control contract.

These checks reject obvious source-structure regressions. They do not replace
Keil compilation or runtime verification on the RA6M5 and mechanical arm.
"""

import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
ROBOT_H = (ROOT / "APP" / "robot.h").read_text(encoding="utf-8")
ROBOT_C = (ROOT / "APP" / "robot.c").read_text(encoding="utf-8")
TARGET_H = (ROOT / "APP" / "robot_target.h").read_text(encoding="utf-8")
TARGET_C = (ROOT / "APP" / "robot_target.c").read_text(encoding="utf-8")
WORKFLOW_H = (ROOT / "APP" / "robot_workflow.h").read_text(encoding="utf-8")
WORKFLOW_C = (ROOT / "APP" / "robot_workflow.c").read_text(encoding="utf-8")
KINEMATICS_H = (ROOT / "Middle" / "robot_kinematics.h").read_text(encoding="utf-8")
KINEMATICS_C = (ROOT / "Middle" / "robot_kinematics.c").read_text(encoding="utf-8")
JETSON_H = (ROOT / "Middle" / "jetson_vision.h").read_text(encoding="utf-8")


def require(text: str, needle: str, label: str) -> None:
    assert needle in text, f"missing {label}: {needle}"


def _blank_except_newlines(text: str) -> str:
    return "".join(char if char in "\r\n" else " " for char in text)


def _strip_c_lexical_noise(text: str) -> str:
    """Blank comments and literals while preserving length and line layout."""
    output = []
    state = "code"
    escaped = False
    index = 0

    while index < len(text):
        char = text[index]
        next_char = text[index + 1] if (index + 1) < len(text) else ""

        if state == "code":
            if (char == "/") and (next_char == "/"):
                output.extend((" ", " "))
                state = "line_comment"
                index += 2
                continue
            if (char == "/") and (next_char == "*"):
                output.extend((" ", " "))
                state = "block_comment"
                index += 2
                continue
            if char == '"':
                output.append(" ")
                state = "string"
                escaped = False
                index += 1
                continue
            if char == "'":
                output.append(" ")
                state = "char"
                escaped = False
                index += 1
                continue
            output.append(char)
            index += 1
            continue

        if state == "line_comment":
            if char in "\r\n":
                output.append(char)
                state = "code"
            else:
                output.append(" ")
            index += 1
            continue

        if state == "block_comment":
            if (char == "*") and (next_char == "/"):
                output.extend((" ", " "))
                state = "code"
                index += 2
                continue
            output.append(char if char in "\r\n" else " ")
            index += 1
            continue

        output.append(char if char in "\r\n" else " ")
        if escaped:
            escaped = False
        elif char == "\\":
            escaped = True
        elif ((state == "string") and (char == '"')) or (
            (state == "char") and (char == "'")
        ):
            state = "code"
        index += 1

    return "".join(output)


def _is_literal_zero(expression: str) -> bool:
    expression = re.sub(r"/\*.*?\*/", "", expression)
    expression = expression.split("//", 1)[0].strip()
    return re.fullmatch(r"(?:0[uUlL]*|\(\s*0[uUlL]*\s*\))", expression) is not None


def _exclude_if0_blocks(text: str) -> str:
    """Blank literal #if 0 branches, retaining an optional #else/#elif branch."""
    source_lines = text.splitlines(keepends=True)
    directive_lines = _strip_c_lexical_noise(text).splitlines(keepends=True)
    output = []
    stack = []
    active = True

    for line, directive_line in zip(source_lines, directive_lines):
        directive = re.match(
            r"^\s*#\s*(if|ifdef|ifndef|elif|else|endif)\b(.*)$",
            directive_line,
        )
        if directive is None:
            output.append(line if active else _blank_except_newlines(line))
            continue

        keyword = directive.group(1)
        expression = directive.group(2)
        if keyword in ("if", "ifdef", "ifndef"):
            controlled = (keyword == "if") and _is_literal_zero(expression)
            frame = {
                "parent_active": active,
                "controlled": controlled,
                "branch_active": not controlled,
                "branch_taken": False,
            }
            stack.append(frame)
            active = frame["parent_active"] and frame["branch_active"]
        elif (keyword == "elif") and stack:
            frame = stack[-1]
            if frame["controlled"]:
                branch_active = (
                    not frame["branch_taken"] and not _is_literal_zero(expression)
                )
                frame["branch_active"] = branch_active
                frame["branch_taken"] = frame["branch_taken"] or branch_active
            else:
                frame["branch_active"] = True
            active = frame["parent_active"] and frame["branch_active"]
        elif (keyword == "else") and stack:
            frame = stack[-1]
            if frame["controlled"]:
                frame["branch_active"] = not frame["branch_taken"]
                frame["branch_taken"] = True
            else:
                frame["branch_active"] = True
            active = frame["parent_active"] and frame["branch_active"]
        elif (keyword == "endif") and stack:
            frame = stack.pop()
            active = frame["parent_active"]

        output.append(_blank_except_newlines(line))

    return "".join(output)


def effective_c(text: str) -> str:
    return _strip_c_lexical_noise(_exclude_if0_blocks(text))


def _is_word_token(token: str) -> bool:
    return re.fullmatch(
        r"(?:[A-Za-z_]\w*|0[xX][0-9A-Fa-f]+[uUlL]*|\d+(?:\.\d+)?[A-Za-z]*)",
        token,
    ) is not None


def flexible_code_pattern(snippet: str) -> str:
    tokens = re.findall(
        r"[A-Za-z_]\w*|0[xX][0-9A-Fa-f]+[uUlL]*|\d+(?:\.\d+)?[A-Za-z]*|[^\s]",
        snippet,
    )
    assert tokens, f"empty C snippet: {snippet!r}"
    parts = []
    for index, token in enumerate(tokens):
        if index > 0:
            separator = r"\s+" if (
                _is_word_token(tokens[index - 1]) and _is_word_token(token)
            ) else r"\s*"
            parts.append(separator)
        parts.append(re.escape(token))

    prefix = r"(?<![A-Za-z0-9_])" if _is_word_token(tokens[0]) else ""
    suffix = r"(?![A-Za-z0-9_])" if _is_word_token(tokens[-1]) else ""
    return prefix + "".join(parts) + suffix


def require_code(text: str, snippet: str, label: str) -> None:
    assert re.search(flexible_code_pattern(snippet), effective_c(text)) is not None, (
        f"missing {label}: {snippet}"
    )


def call_pattern(function_name: str, arguments: tuple[str, ...]) -> str:
    argument_pattern = r"\s*,\s*".join(
        flexible_code_pattern(argument) for argument in arguments
    )
    return (
        rf"(?<![A-Za-z0-9_]){re.escape(function_name)}(?![A-Za-z0-9_])"
        rf"\s*\(\s*{argument_pattern}\s*\)"
    )


def call_matches(text: str, function_name: str,
                 arguments: tuple[str, ...]) -> list[re.Match[str]]:
    return list(re.finditer(call_pattern(function_name, arguments), effective_c(text)))


def require_call(text: str, function_name: str,
                 arguments: tuple[str, ...], label: str) -> None:
    assert call_matches(text, function_name, arguments), (
        f"missing {label}: {function_name}({', '.join(arguments)})"
    )


def has_call_with_first_argument(text: str, function_name: str,
                                 first_argument: str) -> bool:
    pattern = (
        rf"(?<![A-Za-z0-9_]){re.escape(function_name)}(?![A-Za-z0-9_])"
        rf"\s*\(\s*{flexible_code_pattern(first_argument)}\s*,"
    )
    return re.search(pattern, effective_c(text)) is not None


def _last_code_match(text: str, snippet: str) -> re.Match[str]:
    matches = list(re.finditer(flexible_code_pattern(snippet), text))
    assert matches, f"missing C structure: {snippet}"
    return matches[-1]


def case_body(text: str, case_label: str, next_case_label: str) -> str:
    code = effective_c(text)
    start_match = _last_code_match(code, case_label)
    end_match = re.search(flexible_code_pattern(next_case_label),
                          code[start_match.end():])
    assert end_match is not None, f"missing next case: {next_case_label}"
    end = start_match.end() + end_match.start()
    return code[start_match.start():end]


def function_body(text: str, signature: str) -> str:
    """Return one effective C function using noise-safe brace matching."""
    code = effective_c(text)
    signature_match = _last_code_match(code, signature)
    start = signature_match.start()
    opening_brace = code.find("{", signature_match.end())
    assert opening_brace >= 0, f"missing function opening brace: {signature}"
    depth = 0

    for index in range(opening_brace, len(code)):
        if code[index] == "{":
            depth += 1
        elif code[index] == "}":
            depth -= 1
            if depth == 0:
                return code[start:index + 1]

    raise AssertionError(f"unterminated function body: {signature}")


def compact_c(text: str) -> str:
    return re.sub(r"\s+", "", effective_c(text))


def require_macro(text: str, name: str, expected: str, label: str) -> None:
    code = effective_c(text)
    match = re.search(rf"^\s*#define\s+{re.escape(name)}\s+([^\r\n]+)",
                      code, re.MULTILINE)
    assert match is not None, f"missing {label}: {name}"
    value = match.group(1).strip()
    normalized = value.replace("(", "").replace(")", "").strip().lower()
    assert normalized == expected.lower(), (
        f"wrong {label}: {name}={value}, expected {expected}"
    )


def test_robot_visual_servo_api_exists() -> None:
    require_code(ROBOT_H, "ROBOT_VISUAL_SERVO_EVENT", "visual servo event")
    require_code(ROBOT_H, "ROBOT_STATUS_VISUAL_SERVO_ACTIVE", "visual servo status")
    require_code(ROBOT_H, "int robot_visual_servo_start(void);", "visual servo start API")
    require_code(ROBOT_H, "void robot_visual_servo_stop(void);", "visual servo stop API")
    require_code(ROBOT_H,
                 "void robot_visual_servo_set_velocity(float vx, float vy, float vz);",
                 "visual servo velocity API")
    require_code(ROBOT_H, "bool robot_is_visual_servo_active(void);",
                 "visual servo active API")
    require_code(ROBOT_H, "void robot_motion_abort(void);", "motion abort API")
    require_code(ROBOT_H, "robot_auto_result_t", "auto result type")
    require_code(ROBOT_H, "robot_auto_result_t robot_auto_result_consume(void);",
                 "auto result consume API")


def test_robot_visual_servo_runtime_exists() -> None:
    require_code(ROBOT_C, "static int robot_visual_servo_run(void)",
                 "visual servo runner")
    require_code(ROBOT_C, "case ROBOT_VISUAL_SERVO_EVENT:",
                 "visual servo event dispatch")
    require_call(ROBOT_C, "ROBOT_STATUS_SET",
                 ("g_robot.status", "ROBOT_STATUS_VISUAL_SERVO_ACTIVE"),
                 "visual servo active set")
    require_call(ROBOT_C, "ROBOT_STATUS_CLEAR",
                 ("g_robot.status", "ROBOT_STATUS_VISUAL_SERVO_ACTIVE"),
                 "visual servo active clear")
    require_call(ROBOT_C, "ROBOT_STATUS_SET",
                 ("g_robot.status", "ROBOT_STATUS_AUTO_BUSY"),
                 "auto busy guard")
    servo = function_body(ROBOT_C, "static int robot_visual_servo_run(void)")
    require_call(servo, "robot_pid_one_period",
                 ("target_angle", "feedforward", "NULL",
                  "ROBOT_ARM_JOINT_NUM", "true"),
                 "strict-feedback PID use")
    require_call(ROBOT_C, "robot_motion_abort_is_requested", (), "motion abort check")
    require_call(ROBOT_C, "robot_auto_result_set", ("ROBOT_AUTO_RESULT_RUNNING",),
                 "auto result running set")
    require_call(ROBOT_C, "robot_auto_result_set", ("ROBOT_AUTO_RESULT_OK",),
                 "auto result ok set")
    require_call(ROBOT_C, "robot_auto_result_set", ("ROBOT_AUTO_RESULT_FAILED",),
                 "auto result failed set")
    require_call(ROBOT_C, "robot_auto_result_set", ("ROBOT_AUTO_RESULT_ABORTED",),
                 "auto result aborted set")


def test_target_visual_servo_params_and_calls_exist() -> None:
    require_code(TARGET_H, "TARGET_USE_VISUAL_SERVO", "visual servo compile switch")
    require_code(TARGET_H, "TARGET_VS_KX_MM_S_PER_PX", "visual servo X gain")
    require_code(TARGET_H, "TARGET_VS_KZ_MM_S_PER_PX", "visual servo Z gain")
    require_code(TARGET_H, "TARGET_VS_MAX_SPEED_MM_S", "visual servo coarse speed")
    require_code(TARGET_H, "TARGET_VS_FINE_MAX_SPEED_MM_S", "visual servo fine speed")
    require_code(TARGET_H, "TARGET_VS_CMD_TIMEOUT_MS", "visual servo command timeout")
    require_call(TARGET_C, "robot_visual_servo_start", (), "target starts visual servo")
    require_code(TARGET_C, "robot_visual_servo_set_velocity(",
                 "target updates visual servo velocity")
    require_call(TARGET_C, "robot_visual_servo_stop", (), "target stops visual servo")
    require_code(WORKFLOW_H, "ROBOT_WORKFLOW_SAFE_DISTANCE_MM",
                 "workflow safe distance threshold")
    require_code(WORKFLOW_H, "ROBOT_WORKFLOW_RETREAT_STEP_MM",
                 "workflow retreat step")
    require_code(WORKFLOW_C, "workflow_handle_safe_distance",
                 "workflow safe-distance owner")
    assert "SAFE_DISTANCE" not in effective_c(TARGET_C), (
        "target substate must not own distance safety"
    )


def test_selected_view_is_target_start_pose() -> None:
    require_code(TARGET_H, "robot_target_start_at_current_pose",
                 "start-at-current-pose API")
    require_call(TARGET_C, "enter_state",
                 ("ROBOT_TARGET_STATE_WAIT_DETECT", "now_ms"),
                 "direct wait-detect start")
    target_code = effective_c(TARGET_C)
    assert "TARGET_PRE_POSITION" not in target_code, (
        "target must not repeat selected-view pre-position"
    )
    assert "robot_send_reset_event" not in target_code, "target substate must not reset"


def test_wait_detect_emits_ready_event_once() -> None:
    wait_detect = case_body(TARGET_C, "case ROBOT_TARGET_STATE_WAIT_DETECT:",
                            "case ROBOT_TARGET_STATE_ALIGN:")
    require_call(wait_detect, "target_set_event", ("ROBOT_TARGET_EVENT_READY",),
                 "READY event")
    require_code(wait_detect, "s_target.ready_sent = true", "READY one-shot latch")
    assert "jetson_send_status" not in effective_c(TARGET_C), (
        "workflow must own protocol replies"
    )


def test_static_guard_ignores_disabled_and_lexical_noise() -> None:
    sample = r'''
static void guarded_function(void)
{
#if 0
    robot_visual_servo_sync_actual_pose(T_cmd);
#endif
    // robot_visual_servo_sync_actual_pose(T_cmd);
    /* robot_visual_servo_sync_actual_pose(T_cmd); */
    const char *log_text = "robot_visual_servo_sync_actual_pose(T_cmd) }";
    char closing_brace = '}';
    strict_call(
        target_angle,
        true
    );
}
'''
    body = function_body(sample, "static void guarded_function(void)")
    assert not call_matches(
        body, "robot_visual_servo_sync_actual_pose", ("T_cmd",)
    ), "disabled, commented, or literal calls must not satisfy a guard"
    require_call(body, "strict_call", ("target_angle", "true"),
                 "active multiline call after literal braces")


def test_visual_servo_uses_measured_fk_pose() -> None:
    servo = function_body(ROBOT_C, "static int robot_visual_servo_run(void)")
    require_code(servo, "T_cmd", "persistent visual-servo command matrix")
    require_call(servo, "robot_visual_servo_sync_actual_pose", ("T_cmd",),
                 "measured-pose startup synchronization")
    assert not has_call_with_first_argument(
        servo, "robot_kinematics_cal_T", "T_0_6_reset"
    ), (
        "visual servo must not rebuild every target from the reset matrix"
    )

    sync_actual_pose = function_body(
        ROBOT_C, "static bool robot_visual_servo_sync_actual_pose("
    )
    require_call(sync_actual_pose, "robot_kinematics_forward",
                 ("measured", "T_cmd"),
                 "startup synchronization forward kinematics")
    require_code(KINEMATICS_H, "int robot_kinematics_forward(",
                 "forward kinematics API")
    require_code(KINEMATICS_C, "int robot_kinematics_forward(",
                 "forward kinematics implementation")
    function_body(KINEMATICS_C, "int robot_kinematics_forward(")


def test_visual_servo_fault_contract_and_thresholds() -> None:
    require_code(ROBOT_H, "ROBOT_VISUAL_SERVO_FAULT_NONE", "no-fault enum")
    require_code(ROBOT_H, "ROBOT_VISUAL_SERVO_FAULT_FEEDBACK",
                 "feedback fault enum")
    require_code(ROBOT_H, "ROBOT_VISUAL_SERVO_FAULT_FK", "FK fault enum")
    require_code(ROBOT_H, "ROBOT_VISUAL_SERVO_FAULT_IK", "IK fault enum")
    require_code(ROBOT_H,
                 "robot_visual_servo_fault_t robot_visual_servo_fault_get(void);",
                 "fault getter API")
    require_code(ROBOT_H, "void robot_visual_servo_fault_clear(void);",
                 "fault clear API")

    require_macro(ROBOT_C, "ROBOT_VISUAL_SERVO_FEEDBACK_RETRY_COUNT", "3U",
                  "startup feedback retry count")
    require_macro(ROBOT_C, "ROBOT_VISUAL_SERVO_FEEDBACK_FAIL_LIMIT", "3U",
                  "runtime feedback failure limit")
    require_macro(ROBOT_C, "ROBOT_VISUAL_SERVO_IK_FAIL_LIMIT", "3U",
                  "runtime IK failure limit")
    require_macro(ROBOT_C, "ROBOT_VISUAL_SERVO_FK_IK_TOL_DEG", "3.0f",
                  "startup FK/IK tolerance")

    fault_setter = function_body(ROBOT_C, "static void robot_visual_servo_fault_set(")
    require_code(fault_setter,
                 "g_visual_servo.fault == ROBOT_VISUAL_SERVO_FAULT_NONE",
                 "first-fault latch guard")


def test_visual_servo_requires_fresh_feedback_for_pid() -> None:
    require_code(ROBOT_C, "static bool robot_pid_one_period(",
                 "boolean PID period result")
    pid_period = function_body(ROBOT_C, "static bool robot_pid_one_period(")
    require_code(pid_period, "bool require_fresh_feedback", "strict feedback parameter")
    require_code(pid_period, "robot_update_all_angles(", "fresh joint feedback read")
    require_code(pid_period, "require_fresh_feedback && !fresh_feedback",
                 "strict feedback rejection")
    require_call(pid_period, "robot_joint_velocity_zero_all",
                 ("(uint8_t) joint_num",), "zero velocity on stale feedback")

    servo = function_body(ROBOT_C, "static int robot_visual_servo_run(void)")
    require_call(servo, "robot_pid_one_period",
                 ("target_angle", "feedforward", "NULL",
                  "ROBOT_ARM_JOINT_NUM", "true"),
                 "visual servo strict-feedback PID call")


def test_target_observes_fault_and_clears_only_for_new_round() -> None:
    start_round = function_body(TARGET_C, "bool robot_target_start_at_current_pose(void)")
    target_step = function_body(TARGET_C, "void robot_target_step(const target_obs_t *obs)")

    all_clear_calls = call_matches(TARGET_C, "robot_visual_servo_fault_clear", ())
    start_clear_calls = call_matches(start_round, "robot_visual_servo_fault_clear", ())
    assert len(all_clear_calls) == 1, (
        "effective target code must contain exactly one visual-servo fault clear call"
    )
    assert len(start_clear_calls) == 1, (
        "the unique visual-servo fault clear must be in the new-round start function"
    )

    clear_fault = start_clear_calls[0].start()
    enable_match = re.search(
        flexible_code_pattern("ROBOT_TARGET_ENABLED = true"), start_round
    )
    assert enable_match is not None, "new-round start must enable the target state"
    failed_returns = [
        match.start()
        for match in re.finditer(r"\breturn\s+false\s*;", start_round)
    ]
    assert failed_returns, "new-round start must retain explicit failed precondition returns"
    assert all(return_pos < clear_fault for return_pos in failed_returns), (
        "visual-servo fault must clear after every failed startup precondition return"
    )
    assert clear_fault < enable_match.start(), (
        "visual-servo fault must clear before enabling the new target round"
    )

    for signature in (
        "void robot_target_init(void)",
        "bool robot_target_enable_request(void)",
        "void robot_target_stop_hold(void)",
        "void robot_target_disable_request(void)",
        "void robot_target_step(const target_obs_t *obs)",
    ):
        lifecycle_body = function_body(TARGET_C, signature)
        assert not call_matches(
            lifecycle_body, "robot_visual_servo_fault_clear", ()
        ), (
            f"visual-servo fault must not clear in {signature}"
        )

    fault_get_calls = call_matches(target_step, "robot_visual_servo_fault_get", ())
    assert fault_get_calls, "target step must observe the visual-servo fault"
    require_code(target_step, "ROBOT_TARGET_EVENT_FAULT", "target fault event")
    require_code(target_step, "ROBOT_TARGET_STATE_FAULT", "target fault state")
    new_vision_branch = re.search(flexible_code_pattern("if (new_vision)"),
                                  target_step)
    assert new_vision_branch is not None, "target step must retain new-vision handling"
    assert fault_get_calls[0].start() < new_vision_branch.start(), (
        "latched visual-servo fault must be handled before new vision can restart motion"
    )


def test_visual_servo_keeps_protocol_and_workflow_mapping_unchanged() -> None:
    require_macro(JETSON_H, "JETSON_UNIFIED_SOF0", "0xA5u", "unified protocol SOF0")
    require_macro(JETSON_H, "JETSON_UNIFIED_SOF1", "0x5Au", "unified protocol SOF1")
    require_macro(JETSON_H, "JETSON_MSG_STATUS", "0x81u", "unified status message")
    require_macro(JETSON_H, "JETSON_MSG_ERROR", "0xFEu", "unified error message")
    require_macro(JETSON_H, "JETSON_ERROR_MOTION_FAILED", "0x06u",
                  "generic motion-failed error")

    fault_case = case_body(WORKFLOW_C, "case ROBOT_TARGET_EVENT_FAULT:",
                           "case ROBOT_TARGET_EVENT_NONE:")
    require_call(fault_case, "workflow_fault",
                 ("JETSON_ERROR_MOTION_FAILED", "now_ms"),
                 "existing generic motion-failed mapping")
    assert "robot_kinematics_forward" not in effective_c(WORKFLOW_C), (
        "workflow must not own kinematics"
    )


def run_tests() -> None:
    tests = (
        test_static_guard_ignores_disabled_and_lexical_noise,
        test_robot_visual_servo_api_exists,
        test_robot_visual_servo_runtime_exists,
        test_target_visual_servo_params_and_calls_exist,
        test_selected_view_is_target_start_pose,
        test_wait_detect_emits_ready_event_once,
        test_visual_servo_uses_measured_fk_pose,
        test_visual_servo_fault_contract_and_thresholds,
        test_visual_servo_requires_fresh_feedback_for_pid,
        test_target_observes_fault_and_clears_only_for_new_round,
        test_visual_servo_keeps_protocol_and_workflow_mapping_unchanged,
    )
    failures = []

    for test in tests:
        try:
            test()
        except (AssertionError, ValueError) as exc:
            failures.append(f"{test.__name__}: {exc}")

    if failures:
        print("visual servo static checks failed:")
        for failure in failures:
            print(f"- {failure}")
        raise SystemExit(1)

    print("visual servo static checks passed")


if __name__ == "__main__":
    run_tests()
