from pathlib import Path
import sys
import tempfile

from PySide6.QtWidgets import QApplication


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from UpperComputer.ra6m5_upper_console import QtUpperConsole  # noqa: E402


def new_window() -> QtUpperConsole:
    QApplication.instance() or QApplication([])
    temp_dir = tempfile.TemporaryDirectory()
    window = QtUpperConsole(Path(temp_dir.name))
    window._test_temp_dir = temp_dir
    return window


def test_auto_waits_for_final_confirmation_marker_not_pid_loop_end() -> None:
    window = new_window()
    try:
        window.arm_events["AUTO_FINISHED"].clear()
        window._update_arm_status("robot pid run finished!!")
        assert not window.arm_events["AUTO_FINISHED"].is_set()

        window._update_arm_status("[AUTO] finished target=<0.0 -130.0 -15.0>")
        assert window.arm_events["AUTO_FINISHED"].is_set()
    finally:
        window.close()


def test_firmware_emits_auto_marker_only_after_final_confirmation() -> None:
    source = (ROOT / "APP" / "robot.c").read_text(encoding="utf-8")
    success_marker = source.index('LOG("[AUTO] finished')
    final_confirm = source.index("robot_auto_final_confirm", source.index("static void robot_auto_move_interpolation"))
    assert success_marker > final_confirm


def test_general_safety_stop_aborts_motion_and_forces_laser_off() -> None:
    window = new_window()
    try:
        sent: list[str] = []
        window.send_arm_command = lambda command: sent.append(str(command))  # type: ignore[method-assign]

        window.safety_stop()

        assert sent[0] == "motion_abort"
        assert "target_disable" in sent
        assert sent[-1] == "laser_off"
    finally:
        window.close()


if __name__ == "__main__":
    test_auto_waits_for_final_confirmation_marker_not_pid_loop_end()
    test_firmware_emits_auto_marker_only_after_final_confirmation()
    test_general_safety_stop_aborts_motion_and_forces_laser_off()
    print("view arc review guard checks passed")
