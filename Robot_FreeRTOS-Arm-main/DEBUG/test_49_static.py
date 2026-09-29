from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
ROBOT_C = (ROOT / "APP" / "robot.c").read_text(encoding="utf-8")
DOC = (ROOT / "控制文档.md").read_text(encoding="utf-8")


def require(text: str, needle: str, label: str) -> None:
    assert needle in text, f"missing {label}: {needle}"


def test_joint_position_wait_helper_exists() -> None:
    require(ROBOT_C, "static bool robot_joint_wait_target", "joint wait helper")
    require(ROBOT_C, "robot_joint_compare_error", "shared error compare")
    require(ROBOT_C, "ROBOT_JOINT_POS_CONFIRM_STABLE_HITS", "stable hit threshold")


def test_joint_position_commands_wait_after_send() -> None:
    rel_case = ROBOT_C.split("case ROBOT_JOINT_REL_ROTATE:", 1)[1].split("break;", 1)[0]
    abs_case = ROBOT_C.split("case ROBOT_JOINT_ABS_ROTATE:", 1)[1].split("break;", 1)[0]
    require(rel_case, "robot_joint_wait_target", "relative joint command wait")
    require(abs_case, "robot_joint_wait_target", "absolute joint command wait")


def test_auto_updates_pose_only_after_final_confirm() -> None:
    auto_func = ROBOT_C.split("static void robot_auto_move_interpolation", 1)[1].split("static float robot_angle_normalize", 1)[0]
    require(auto_func, "robot_auto_final_confirm", "AUTO final confirm")
    assert "g_robot.cur_pos.x = target_pos->x;" in auto_func
    assert auto_func.index("robot_auto_final_confirm") < auto_func.index("g_robot.cur_pos.x = target_pos->x;")
    require(auto_func, "ROBOT_STATUS_CLEAR(g_robot.status, ROBOT_STATUS_POSE_VALID)", "pose invalid on AUTO miss")


def test_time_func_updates_pose_only_after_final_confirm() -> None:
    time_func = ROBOT_C.split("static void robot_time_func_move", 1)[1].split("static void robot_auto_busy_set", 1)[0]
    require(time_func, "robot_auto_final_confirm", "time_func final confirm")
    assert "g_robot.cur_pos.x = path[path_size -1].x;" in time_func
    assert time_func.index("robot_auto_final_confirm") < time_func.index("g_robot.cur_pos.x = path[path_size -1].x;")


def test_document_marks_49_done() -> None:
    status_line = next(line for line in DOC.splitlines() if line.startswith("| 4.9"))
    require(status_line, "到位确认", "4.9 arrival confirmation status")
    require(status_line, "已完成", "4.9 completed status")
    require(DOC, "robot_joint_wait_target", "document joint wait helper")
    require(DOC, "robot_auto_final_confirm", "document auto final confirm")


def test_settle_tail_remains_fixed_short_window() -> None:
    """当前可用版本保留固定 10 周期末端稳定段，不引入自适应稳定逻辑。"""
    pid_run = ROBOT_C.split("static int robot_pid_run(", 1)[1].split("static void robot_joints_sync_to", 1)[0]
    settle_tail = pid_run.split("前馈清零", 1)[1].split("if (sample_count > 0)", 1)[0]

    require(settle_tail, "for (int k = 0; k < ROBOT_PID_SETTLE_PERIODS; k++)",
            "fixed settle loop")
    require(settle_tail, "robot_pid_one_period(target_angle, feedforward, total_error, ROBOT_ARM_JOINT_NUM);",
            "settle loop keeps final target")
    assert "ROBOT_JOINT_POS_CONFIRM_TOL_DEG" not in settle_tail, \
        "settle tail must not reintroduce adaptive confirm logic"


def test_settle_periods_is_documented_as_timeout_upper_bound() -> None:
    require(DOC, "ROBOT_PID_SETTLE_PERIODS", "settle periods constant documented")
    require(DOC, "100ms", "settle periods documented as fixed 100ms window")


def test_per_joint_feedforward_gain_applied() -> None:
    """AUTO 速度环用 ROBOT_JOINT_FF_GAIN[j] 缩放前馈，修比例型跟踪偏差(非单点Kp)。"""
    require(ROBOT_C, "ROBOT_JOINT_FF_GAIN[ROBOT_MAX_JOINT_NUM]", "per-joint FF gain array")
    require(ROBOT_C, "ROBOT_JOINT_FF_GAIN[j] * feedforward[j] + ROBOT_JOINT_KP[j] * error",
            "FF gain applied in velocity law")


def test_final_confirm_dumps_per_joint_error() -> None:
    """确认失败时逐关节打印 cur/target/err，供下一轮前馈调参。"""
    confirm = ROBOT_C.split("static bool robot_auto_final_confirm", 1)[1].split("static void robot_joint_soft_reset", 1)[0]
    require(confirm, "cur=%.2f target=%.2f err=%.2f", "per-joint cur/target/err dump")


def test_pid_run_failure_also_degrades_pose() -> None:
    """robot_pid_run() 中途异常返回(ret!=0)时也要清 POSE_VALID/置 POSE_DEGRADED，
    否则半路中止会让软件位姿保留上一次成功的旧状态。"""
    for anchor, end in (
        ("static void robot_auto_move_interpolation", "static float robot_angle_normalize"),
        ("static void robot_time_func_move", "static void robot_auto_busy_set"),
    ):
        func = ROBOT_C.split(anchor, 1)[1].split(end, 1)[0]
        call_index = func.index("robot_pid_run(")
        if_index = func.index("if (ret == 0)", call_index)
        else_head = func.split("PID aborted before the final target", 1)[1][:500]
        require(else_head, "ROBOT_STATUS_CLEAR(g_robot.status, ROBOT_STATUS_POSE_VALID)",
                f"{anchor}: else branch clears POSE_VALID")
        require(else_head, "ROBOT_STATUS_SET(g_robot.status, ROBOT_STATUS_POSE_DEGRADED)",
                f"{anchor}: else branch sets POSE_DEGRADED")
        assert if_index < func.index("PID aborted before the final target"), \
            "PID failure branch must follow the ret==0 check"


if __name__ == "__main__":
    tests = [
        test_joint_position_wait_helper_exists,
        test_joint_position_commands_wait_after_send,
        test_auto_updates_pose_only_after_final_confirm,
        test_time_func_updates_pose_only_after_final_confirm,
        test_document_marks_49_done,
        test_settle_tail_remains_fixed_short_window,
        test_settle_periods_is_documented_as_timeout_upper_bound,
        test_per_joint_feedforward_gain_applied,
        test_final_confirm_dumps_per_joint_error,
        test_pid_run_failure_also_degrades_pose,
    ]
    for test in tests:
        test()
    print("4.9 static checks passed")
