from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from UpperComputer.ra6m5_upper_console import qt_gui_self_test  # noqa: E402


def test_current_qt_entry_starts_without_serial_hardware() -> None:
    qt_gui_self_test()


if __name__ == "__main__":
    test_current_qt_entry_starts_without_serial_hardware()
    print("upper console compatibility smoke checks passed")
