from pathlib import Path
import sys
import tempfile
import threading

from PySide6.QtCore import QSettings
from PySide6.QtWidgets import QApplication, QPushButton, QTabWidget

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from UpperComputer.ra6m5_upper_console import (  # noqa: E402
    EVENT_COMMAND_ACK,
    JETSON_MSG_STATUS,
    JETSON_MSG_WORKFLOW_CTRL,
    JETSON_PROTOCOL_NEW,
    Ra6Status,
    QtUpperConsole,
    build_unified_frame,
    should_log_serial_tx,
)


def _new_window() -> QtUpperConsole:
    QApplication.instance() or QApplication([])
    temp_dir = tempfile.TemporaryDirectory()
    window = QtUpperConsole(Path(temp_dir.name))
    window._test_temp_dir = temp_dir
    return window


def _clear_settings() -> None:
    settings = QSettings("RA6M5Robot", "UpperConsole")
    settings.clear()
    settings.sync()


def _button_texts(window: QtUpperConsole) -> list[str]:
    return [button.text() for button in window.findChildren(QPushButton)]


def _button_by_text(window: QtUpperConsole, text: str) -> QPushButton:
    for button in window.findChildren(QPushButton):
        if button.text() == text:
            return button
    raise AssertionError(f"button not found: {text}")


def _tab_texts(window: QtUpperConsole) -> list[str]:
    tabs = window.findChild(QTabWidget)
    assert tabs is not None
    return [tabs.tabText(idx) for idx in range(tabs.count())]


def test_log_clear_and_refresh_controls_are_visible() -> None:
    window = _new_window()
    try:
        texts = _button_texts(window)
        assert "清空日志" in texts
        assert "刷新串口" in texts
    finally:
        window.close()


def test_arm_command_without_open_port_has_chinese_log_feedback() -> None:
    window = _new_window()
    try:
        window.send_arm_command("soft_reset")
        window._flush_logs()
        text = window.log_text.toPlainText()
        assert "ARM串口未打开" in text
        assert "soft_reset" in text
    finally:
        window.close()


def test_soft_reset_button_sends_soft_reset_command() -> None:
    window = _new_window()
    try:
        _button_by_text(window, "soft_reset").click()
        window._flush_logs()
        text = window.log_text.toPlainText()
        assert "ARM串口未打开" in text
        assert "soft_reset" in text
        assert "False" not in text
    finally:
        window.close()


def test_clear_log_clears_visible_log_area() -> None:
    window = _new_window()
    try:
        window.emit_log("APP", "INFO", "hello")
        window._flush_logs()
        assert "hello" in window.log_text.toPlainText()
        window.clear_visible_log()
        assert window.log_text.toPlainText() == ""
    finally:
        window.close()


def test_arm_command_history_keeps_latest_15_and_moves_duplicates_to_front() -> None:
    window = _new_window()
    try:
        for idx in range(17):
            window.send_arm_command(f"cmd_{idx}")
        history = [window.arm_history.itemText(idx) for idx in range(window.arm_history.count())]
        assert len(history) == 15
        assert history[0] == "cmd_16"
        assert history[-1] == "cmd_2"

        window.send_arm_command("cmd_10")
        history = [window.arm_history.itemText(idx) for idx in range(window.arm_history.count())]
        assert history[0] == "cmd_10"
        assert history.count("cmd_10") == 1
        assert len(history) == 15
    finally:
        window.close()


def test_target_demo_controls_are_visible_with_safe_defaults() -> None:
    _clear_settings()
    window = _new_window()
    try:
        texts = _button_texts(window)
        assert "一键定靶演示" in texts
        assert "停止演示" in texts
        assert "保存演示配置" in texts
        assert window.demo_sequence.text() == "30,-20:5;15,-10:5;8,-5:5;0,0:0"
        assert window.demo_period_ms.text() == "200"
        assert window.demo_send_enable.isChecked()
        assert window.demo_wait_ready.isChecked()
        assert window.demo_auto_stop.isChecked()
    finally:
        window.close()
        _clear_settings()


def test_target_demo_settings_persist() -> None:
    _clear_settings()
    window = _new_window()
    try:
        window.demo_sequence.setText("15,-10:2;0,0:0")
        window.demo_period_ms.setText("150")
        window.demo_wait_ready.setChecked(False)
        window.save_demo_settings()
    finally:
        window.close()

    window = _new_window()
    try:
        assert window.demo_sequence.text() == "15,-10:2;0,0:0"
        assert window.demo_period_ms.text() == "150"
        assert not window.demo_wait_ready.isChecked()
    finally:
        window.close()
        _clear_settings()


def test_formal_workflow_controls_are_visible_with_safe_defaults() -> None:
    _clear_settings()
    window = _new_window()
    try:
        texts = _button_texts(window)
        assert "一键定靶演示" in texts
        assert "开始一键比赛全流程" in texts
        assert "终止并保持" in texts
        assert window.workflow_view.currentData() == 2
        assert window.workflow_safe_distance.text() == "160"
        assert window.workflow_vision_sequence.text() == "20,-15:2;8,-6:2;0,0:0"
        assert window.workflow_stage.text() == "未运行"
    finally:
        window.close()
        _clear_settings()


def test_formal_workflow_settings_persist() -> None:
    _clear_settings()
    window = _new_window()
    try:
        window.workflow_view.setCurrentIndex(2)
        window.workflow_safe_distance.setText("135")
        window.workflow_vision_sequence.setText("10,-8:1;0,0:0")
        window.save_formal_workflow_settings()
    finally:
        window.close()

    window = _new_window()
    try:
        assert window.workflow_view.currentData() == 3
        assert window.workflow_safe_distance.text() == "135"
        assert window.workflow_vision_sequence.text() == "10,-8:1;0,0:0"
    finally:
        window.close()
        _clear_settings()


def test_formal_workflow_start_without_jetson_port_has_chinese_feedback() -> None:
    window = _new_window()
    try:
        window.start_formal_workflow_demo()
        window._flush_logs()
        text = window.log_text.toPlainText()
        assert "Jetson模拟串口未打开" in text
        assert "比赛全流程" in text
    finally:
        window.close()


def test_formal_control_waits_for_same_seq_ack() -> None:
    window = _new_window()
    try:
        protocol_index = window.protocol_mode.findData(JETSON_PROTOCOL_NEW)
        window.protocol_mode.setCurrentIndex(protocol_index)
        sent: list[tuple[bytes, str]] = []
        window.jetson.is_open = lambda: True  # type: ignore[method-assign]
        window.jetson.write = lambda data, label: sent.append((bytes(data), str(label)))  # type: ignore[method-assign]
        window.unified_seq = 0
        window.formal_workflow_stop.clear()
        window.formal_status_mailbox.publish(
            Ra6Status(
                raw=b"",
                func=EVENT_COMMAND_ACK,
                value=1,
                name="COMMAND_ACK",
                seq=1,
                event=EVENT_COMMAND_ACK,
                error=0,
                msg_type=JETSON_MSG_STATUS,
                is_formal=True,
            )
        )

        seq = window._send_formal_control(JETSON_MSG_WORKFLOW_CTRL, b"\x01", "测试启动", 0.1)

        assert seq == 1
        assert sent[0][0][3] == JETSON_MSG_WORKFLOW_CTRL
        assert sent[0][0][4] == 1
        assert sent[0][0][6] == 1
    finally:
        window.close()


def test_formal_workflow_stop_requests_abort_without_soft_reset() -> None:
    window = _new_window()
    try:
        actions: list[str] = []
        window.formal_workflow_active = True
        window._send_formal_abort_best_effort = lambda: actions.append("abort")  # type: ignore[method-assign]
        window.send_arm_command = lambda command: actions.append(str(command))  # type: ignore[method-assign]

        window.stop_formal_workflow_demo()

        assert window.formal_workflow_stop.is_set()
        assert actions == ["abort"]
    finally:
        window.close()


def test_jetson_rx_publishes_formal_status_with_seq() -> None:
    window = _new_window()
    try:
        window.formal_workflow_stop.clear()
        frame = build_unified_frame(
            JETSON_MSG_STATUS,
            0x44,
            bytes([EVENT_COMMAND_ACK, 0x01, 0x00]),
        )
        window.on_serial_data("JETSON", frame)
        status = window.formal_status_mailbox.wait(
            EVENT_COMMAND_ACK,
            1,
            0x44,
            0.1,
            window.formal_workflow_stop,
        )
        assert status.seq == 0x44
    finally:
        window.close()


def test_workflow_stage_queue_updates_chinese_prompt() -> None:
    window = _new_window()
    try:
        window._set_workflow_stage("视觉对准完成，请按住 P000 开启激光")
        window._flush_logs()
        assert window.workflow_stage.text() == "视觉对准完成，请按住 P000 开启激光"
    finally:
        window.close()


def test_old_target_demo_cannot_start_during_formal_workflow() -> None:
    window = _new_window()
    try:
        protocol_index = window.protocol_mode.findData(JETSON_PROTOCOL_NEW)
        window.protocol_mode.setCurrentIndex(protocol_index)
        window.jetson.is_open = lambda: True  # type: ignore[method-assign]
        window.jetson.write = lambda _data, _label: None  # type: ignore[method-assign]
        window.demo_sequence.setText("0,0:1")
        window.demo_send_enable.setChecked(False)
        window.demo_wait_ready.setChecked(False)
        window.demo_prompt_p000.setChecked(False)
        window.demo_auto_stop.setChecked(False)
        window.automatic_demo_owner = "formal"

        window.start_target_demo()
        window._flush_logs()

        assert "比赛全流程正在运行" in window.log_text.toPlainText()
    finally:
        window.close()


def test_closing_jetson_port_aborts_active_formal_workflow_first() -> None:
    window = _new_window()
    try:
        actions: list[str] = []
        window.formal_workflow_active = True
        window.formal_workflow_stop.clear()
        window._send_formal_abort_best_effort = lambda: actions.append("abort")  # type: ignore[method-assign]
        window.jetson.close = lambda: actions.append("close")  # type: ignore[method-assign]

        window.close_channel(window.jetson)

        assert window.formal_workflow_stop.is_set()
        assert actions[:2] == ["abort", "close"]
    finally:
        window.close()


def test_periodic_formal_vision_frames_do_not_flood_log() -> None:
    assert not should_log_serial_tx("FORMAL_VISION dcx=0 dcy=0 SEQ=0x22")


def test_window_close_requests_abort_for_active_formal_workflow() -> None:
    window = _new_window()
    try:
        actions: list[str] = []
        window.formal_workflow_active = True
        window.formal_workflow_stop.clear()
        window._send_formal_abort_best_effort = lambda: actions.append("abort")  # type: ignore[method-assign]
        window.arm.close = lambda: actions.append("arm_close")  # type: ignore[method-assign]
        window.jetson.close = lambda: actions.append("jetson_close")  # type: ignore[method-assign]

        class FakeCloseEvent:
            def accept(self) -> None:
                actions.append("accept")

        window.closeEvent(FakeCloseEvent())

        assert window.formal_workflow_stop.is_set()
        assert actions[0] == "abort"
        assert actions[-1] == "accept"
        window.formal_workflow_active = False
    finally:
        window.close()


def test_formal_workflow_forces_200ms_heartbeat() -> None:
    window = _new_window()
    try:
        calls: list[str] = []
        window.heartbeat_period_ms.setText("900")
        window.heartbeat_enable.blockSignals(True)
        window.heartbeat_enable.setChecked(False)
        window.heartbeat_enable.blockSignals(False)
        window.start_heartbeat = lambda: calls.append("start")  # type: ignore[method-assign]

        window._start_formal_heartbeat()

        assert window.heartbeat_period_ms.text() == "200"
        assert window.heartbeat_enable.isChecked()
        assert calls == ["start"]
    finally:
        window.close()


def test_start_heartbeat_writes_first_frame_before_return() -> None:
    window = _new_window()
    try:
        new_index = window.protocol_mode.findData(JETSON_PROTOCOL_NEW)
        window.protocol_mode.setCurrentIndex(new_index)
        sent: list[tuple[bytes, str]] = []
        window.jetson.is_open = lambda: True  # type: ignore[method-assign]
        window.jetson.write = lambda data, label: sent.append((bytes(data), str(label)))  # type: ignore[method-assign]
        window._heartbeat_loop = lambda _period, _stop: None  # type: ignore[method-assign]

        window.start_heartbeat()

        assert sent
        assert sent[0][0][:2] == bytes.fromhex("A5 5A")
        assert sent[0][1] == "HEARTBEAT"
    finally:
        window.close()


def test_initial_unified_sequence_seed_is_nonzero() -> None:
    window = _new_window()
    try:
        assert 1 <= window.unified_seq <= 0xFF
    finally:
        window.close()


def test_safety_stop_uses_formal_abort_during_competition_workflow() -> None:
    window = _new_window()
    try:
        actions: list[str] = []
        window.formal_workflow_active = True
        window.stop_formal_workflow_demo = lambda: actions.append("formal_abort")  # type: ignore[method-assign]
        window.send_arm_command = lambda command: actions.append(str(command))  # type: ignore[method-assign]

        window.safety_stop()

        assert actions == ["formal_abort", "laser_off"]
        window.formal_workflow_active = False
    finally:
        window.close()


def test_switching_away_from_formal_protocol_aborts_active_workflow() -> None:
    window = _new_window()
    try:
        new_index = window.protocol_mode.findData(JETSON_PROTOCOL_NEW)
        old_index = 0 if new_index != 0 else 1
        window.protocol_mode.setCurrentIndex(new_index)
        actions: list[str] = []
        window.formal_workflow_active = True
        window.formal_workflow_stop.clear()
        window._send_formal_abort_best_effort = lambda: actions.append("abort")  # type: ignore[method-assign]

        window.protocol_mode.setCurrentIndex(old_index)

        assert window.formal_workflow_stop.is_set()
        assert actions == ["abort"]
        window.formal_workflow_active = False
    finally:
        window.close()


def test_unified_sequence_wrap_skips_reserved_zero() -> None:
    window = _new_window()
    try:
        window.unified_seq = 0xFE
        assert window._next_unified_seq() == 0xFF
        assert window._next_unified_seq() == 0x01
    finally:
        window.close()


def test_manual_motion_is_blocked_during_formal_workflow_but_laser_off_is_allowed() -> None:
    window = _new_window()
    try:
        sent: list[bytes] = []
        window.formal_workflow_active = True
        window.arm.is_open = lambda: True  # type: ignore[method-assign]
        window.arm.write = lambda data, _label: sent.append(bytes(data))  # type: ignore[method-assign]

        window.send_arm_command("auto 15 0 0")
        window.send_arm_command("soft_reset")
        window.send_arm_command("laser_off")
        window._flush_logs()

        assert sent == [b"laser_off\r\n"]
        assert "比赛全流程运行中，已拒绝手动命令" in window.log_text.toPlainText()
        window.formal_workflow_active = False
    finally:
        window.close()


def test_formal_workflow_rejects_simulated_distance_below_firmware_threshold() -> None:
    window = _new_window()
    try:
        new_index = window.protocol_mode.findData(JETSON_PROTOCOL_NEW)
        window.protocol_mode.setCurrentIndex(new_index)
        window.jetson.is_open = lambda: True  # type: ignore[method-assign]
        window.workflow_safe_distance.setText("90")
        window._acquire_automatic_demo = lambda _owner: False  # type: ignore[method-assign]

        window.start_formal_workflow_demo()
        window._flush_logs()

        assert "安全距离不得低于固件阈值 150 mm" in window.log_text.toPlainText()
    finally:
        window.close()


def test_status_lights_follow_ra6_status() -> None:
    window = _new_window()
    try:
        window._update_ra6_status("TARGET_CTRL_ON")
        assert "background-color: #22c55e" in window.status_dots["TARGET_CTRL"].styleSheet()
        window._update_ra6_status("OUTPUT_OFF")
        assert "background-color: #9ca3af" in window.status_dots["OUTPUT"].styleSheet()
        window._update_ra6_status("ERROR")
        assert "background-color: #ef4444" in window.status_dots["ERROR"].styleSheet()
        window._set_status("ERROR", "-")
        window._update_ra6_status("VISION_LOST")
        assert "background-color: #ef4444" in window.status_dots["ERROR"].styleSheet()
    finally:
        window.close()


def test_background_status_updates_wait_for_gui_flush() -> None:
    window = _new_window()
    try:
        arm_thread = threading.Thread(
            target=window._update_arm_status,
            args=("soft reset final verify PASS",),
        )
        ra6_thread = threading.Thread(target=window._update_ra6_status, args=("READY",))
        arm_thread.start()
        ra6_thread.start()
        arm_thread.join()
        ra6_thread.join()

        assert window.status_labels["POSE_VALID"].text() == "POSE_VALID: -"
        assert window.status_labels["READY"].text() == "READY: -"

        window._flush_logs()

        assert window.status_labels["POSE_VALID"].text() == "POSE_VALID: YES"
        assert window.status_labels["READY"].text() == "READY: YES"
    finally:
        window.close()


def test_closing_ports_clears_unknown_runtime_status_lights() -> None:
    window = _new_window()
    try:
        window._set_status("POSE_VALID", "YES")
        window.close_channel(window.arm)
        assert window.status_labels["POSE_VALID"].text() == "POSE_VALID: -"
        assert "background-color: #9ca3af" in window.status_dots["POSE_VALID"].styleSheet()

        for name in ("TARGET_CTRL_ON", "READY", "ALIGN_DONE", "OUTPUT_ON", "ERROR"):
            window._update_ra6_status(name)
        window.close_channel(window.jetson)
        for key in ("TARGET_CTRL", "READY", "ALIGN_DONE", "CONFIRMING", "OUTPUT", "ERROR"):
            assert window.status_labels[key].text() == f"{key}: -"
            assert "background-color: #9ca3af" in window.status_dots[key].styleSheet()
    finally:
        window.close()


def test_target_restart_and_stop_clear_stale_target_lights() -> None:
    window = _new_window()
    try:
        for name in ("TARGET_CTRL_ON", "READY", "ALIGN_DONE", "OUTPUT_ON", "ERROR"):
            window._update_ra6_status(name)

        window._update_ra6_status("TARGET_CTRL_ON")
        for key in ("READY", "ALIGN_DONE", "CONFIRMING", "OUTPUT", "ERROR"):
            assert window.status_labels[key].text() == f"{key}: -"
            assert "background-color: #9ca3af" in window.status_dots[key].styleSheet()

        window._update_ra6_status("READY")
        window._update_ra6_status("TARGET_CTRL_OFF")
        assert window.status_labels["TARGET_CTRL"].text() == "TARGET_CTRL: OFF"
        assert window.status_labels["OUTPUT"].text() == "OUTPUT: OFF"
        for key in ("READY", "ALIGN_DONE", "CONFIRMING", "ERROR"):
            assert window.status_labels[key].text() == f"{key}: -"
            assert "background-color: #9ca3af" in window.status_dots[key].styleSheet()
    finally:
        window.close()


def test_target_stop_buttons_request_firmware_reset_or_arm_fallback() -> None:
    window = _new_window()
    try:
        sent: list[str] = []
        window.send_arm_command = lambda command: sent.append(str(command))  # type: ignore[method-assign]
        window.send_target_ctrl = lambda enable: sent.append(f"target_ctrl:{int(enable)}") or True  # type: ignore[method-assign]

        _button_by_text(window, "关闭状态机 AA 01 00 BB").click()
        assert sent == ["target_ctrl:0"]

        sent.clear()
        _button_by_text(window, "停止演示").click()
        assert sent == ["target_ctrl:0", "laser_off"]

        window.send_target_ctrl = lambda enable: sent.append(f"target_ctrl:{int(enable)}") or False  # type: ignore[method-assign]
        sent.clear()
        _button_by_text(window, "关闭状态机 AA 01 00 BB").click()
        assert sent == ["target_ctrl:0", "soft_reset"]
    finally:
        window.close()


def test_capture_recipe_tab_has_default_three_view_recipes() -> None:
    _clear_settings()
    window = _new_window()
    try:
        assert "拍摄点位" in _tab_texts(window)
        assert window.capture_view.currentText() == "左视图"
        assert "auto 45 -130 -15" in window.capture_recipe.toPlainText()
        assert "abs_rotate 0 85" in window.capture_recipe.toPlainText()

        window.capture_view.setCurrentText("正视图")
        assert "auto 0 -130 -15" in window.capture_recipe.toPlainText()
        assert "abs_rotate 0 90" in window.capture_recipe.toPlainText()

        window.capture_view.setCurrentText("右视图")
        assert "auto -45 -130 -15" in window.capture_recipe.toPlainText()
        assert "abs_rotate 0 105" in window.capture_recipe.toPlainText()
    finally:
        window.close()
        _clear_settings()


def test_capture_recipe_settings_persist_for_selected_view() -> None:
    _clear_settings()
    window = _new_window()
    try:
        window.capture_view.setCurrentText("正视图")
        window.capture_recipe.setPlainText("soft_reset\nauto 1 2 3\nabs_rotate 0 91")
        window.save_capture_recipe()
    finally:
        window.close()

    window = _new_window()
    try:
        window.capture_view.setCurrentText("正视图")
        assert "auto 1 2 3" in window.capture_recipe.toPlainText()
        assert "abs_rotate 0 91" in window.capture_recipe.toPlainText()
    finally:
        window.close()
        _clear_settings()


def test_capture_recipe_next_skips_comments_and_can_reset_to_start() -> None:
    window = _new_window()
    try:
        sent: list[str] = []
        window.send_arm_command = lambda command: sent.append(str(command))  # type: ignore[method-assign]
        window.capture_recipe.setPlainText("# comment\n\nsoft_reset\n# middle\nauto 1 2 3")

        window.send_next_capture_command()
        window.send_next_capture_command()
        assert sent == ["soft_reset", "auto 1 2 3"]

        window.reset_capture_steps()
        window.send_next_capture_command()
        assert sent[-1] == "soft_reset"
    finally:
        window.close()


if __name__ == "__main__":
    test_log_clear_and_refresh_controls_are_visible()
    test_arm_command_without_open_port_has_chinese_log_feedback()
    test_soft_reset_button_sends_soft_reset_command()
    test_clear_log_clears_visible_log_area()
    test_arm_command_history_keeps_latest_15_and_moves_duplicates_to_front()
    test_target_demo_controls_are_visible_with_safe_defaults()
    test_target_demo_settings_persist()
    test_formal_workflow_controls_are_visible_with_safe_defaults()
    test_formal_workflow_settings_persist()
    test_formal_workflow_start_without_jetson_port_has_chinese_feedback()
    test_formal_control_waits_for_same_seq_ack()
    test_formal_workflow_stop_requests_abort_without_soft_reset()
    test_jetson_rx_publishes_formal_status_with_seq()
    test_workflow_stage_queue_updates_chinese_prompt()
    test_old_target_demo_cannot_start_during_formal_workflow()
    test_closing_jetson_port_aborts_active_formal_workflow_first()
    test_periodic_formal_vision_frames_do_not_flood_log()
    test_window_close_requests_abort_for_active_formal_workflow()
    test_formal_workflow_forces_200ms_heartbeat()
    test_start_heartbeat_writes_first_frame_before_return()
    test_initial_unified_sequence_seed_is_nonzero()
    test_safety_stop_uses_formal_abort_during_competition_workflow()
    test_switching_away_from_formal_protocol_aborts_active_workflow()
    test_unified_sequence_wrap_skips_reserved_zero()
    test_manual_motion_is_blocked_during_formal_workflow_but_laser_off_is_allowed()
    test_formal_workflow_rejects_simulated_distance_below_firmware_threshold()
    test_status_lights_follow_ra6_status()
    test_background_status_updates_wait_for_gui_flush()
    test_closing_ports_clears_unknown_runtime_status_lights()
    test_target_restart_and_stop_clear_stale_target_lights()
    test_target_stop_buttons_request_firmware_reset_or_arm_fallback()
    test_capture_recipe_tab_has_default_three_view_recipes()
    test_capture_recipe_settings_persist_for_selected_view()
    test_capture_recipe_next_skips_comments_and_can_reset_to_start()
    print("upper console gui feature checks passed")
