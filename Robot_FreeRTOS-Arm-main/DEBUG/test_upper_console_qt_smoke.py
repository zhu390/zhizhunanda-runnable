from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from UpperComputer.ra6m5_upper_console import qt_gui_self_test  # noqa: E402


def test_qt_gui_self_test_creates_main_window() -> None:
    qt_gui_self_test()


if __name__ == "__main__":
    test_qt_gui_self_test_creates_main_window()
    print("upper console qt smoke checks passed")
