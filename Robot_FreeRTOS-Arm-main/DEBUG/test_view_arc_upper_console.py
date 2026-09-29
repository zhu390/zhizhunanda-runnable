from pathlib import Path
import sys
import tempfile

from PySide6.QtWidgets import QApplication, QPushButton


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from UpperComputer.ra6m5_upper_console import QtUpperConsole  # noqa: E402


def new_window() -> QtUpperConsole:
    QApplication.instance() or QApplication([])
    temp_dir = tempfile.TemporaryDirectory()
    window = QtUpperConsole(Path(temp_dir.name))
    window._test_temp_dir = temp_dir
    return window


def button_texts(window: QtUpperConsole) -> list[str]:
    return [button.text() for button in window.findChildren(QPushButton)]


def test_view_arc_demo_controls_are_visible_with_safe_default() -> None:
    window = new_window()
    try:
        texts = button_texts(window)
        assert "一键三视图弧线" in texts
        assert "停止弧线" in texts
        assert window.view_arc_time.text() == "8000"
    finally:
        window.close()


def test_view_arc_demo_requires_open_arm_port_with_chinese_feedback() -> None:
    window = new_window()
    try:
        window.start_view_arc_demo()
        window._flush_logs()
        text = window.log_text.toPlainText()
        assert "ARM串口未打开" in text
        assert "三视图弧线" in text
    finally:
        window.close()


def test_view_arc_demo_sends_reset_common_pose_and_single_arc_command() -> None:
    window = new_window()
    try:
        sent: list[str] = []
        window.send_arm_command = lambda command: sent.append(str(command))  # type: ignore[method-assign]
        window._wait_arm_event = lambda *_args, **_kwargs: "ok"  # type: ignore[method-assign]
        window.view_arc_stop.clear()

        assert window._view_arc_demo_loop(8000)
        assert sent == ["soft_reset", "auto 0 -130 -15", "view_arc 8000"]
    finally:
        window.close()


def test_split_arm_log_line_still_sets_view_arc_finished_event() -> None:
    window = new_window()
    try:
        window.on_serial_data("ARM", b"[VIEW_AR")
        assert not window.arm_events["VIEW_ARC_FINISHED"].is_set()

        window.on_serial_data("ARM", b"C] finished\r\n")
        assert window.arm_events["VIEW_ARC_FINISHED"].is_set()
    finally:
        window.close()


def test_stop_view_arc_demo_sends_latched_motion_abort() -> None:
    window = new_window()
    try:
        sent: list[str] = []
        window.send_arm_command = lambda command: sent.append(str(command))  # type: ignore[method-assign]

        window.stop_view_arc_demo()

        assert window.view_arc_stop.is_set()
        assert sent == ["motion_abort", "laser_off"]
    finally:
        window.close()


if __name__ == "__main__":
    test_view_arc_demo_controls_are_visible_with_safe_default()
    test_view_arc_demo_requires_open_arm_port_with_chinese_feedback()
    test_view_arc_demo_sends_reset_common_pose_and_single_arc_command()
    test_split_arm_log_line_still_sets_view_arc_finished_event()
    test_stop_view_arc_demo_sends_latched_motion_abort()
    print("view arc upper console checks passed")
