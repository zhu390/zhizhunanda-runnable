from pathlib import Path
import sys
import tempfile

from PySide6.QtWidgets import QApplication


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from UpperComputer.ra6m5_upper_console import QtUpperConsole  # noqa: E402


class RunningThread:
    def is_alive(self) -> bool:
        return True


def test_closing_arm_port_stops_active_arc_before_close_and_clears_rx() -> None:
    QApplication.instance() or QApplication([])
    temp_dir = tempfile.TemporaryDirectory()
    window = QtUpperConsole(Path(temp_dir.name))
    actions: list[str] = []
    try:
        window.view_arc_thread = RunningThread()  # type: ignore[assignment]
        window.arm_rx_buffer.extend(b"partial")
        window.stop_view_arc_demo = lambda: actions.append("stop")  # type: ignore[method-assign]
        window.arm.close = lambda: actions.append("close")  # type: ignore[method-assign]

        window.close_channel(window.arm)

        assert actions[:2] == ["stop", "close"]
        assert window.arm_rx_buffer == bytearray()
    finally:
        window.view_arc_thread = None
        window.close()
        temp_dir.cleanup()


if __name__ == "__main__":
    test_closing_arm_port_stops_active_arc_before_close_and_clears_rx()
    print("view arc port close checks passed")
