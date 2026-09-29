from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SOURCE = (ROOT / "UpperComputer" / "ra6m5_upper_console.py").read_text(encoding="utf-8")


def test_capture_buttons_are_chinese() -> None:
    required = [
        "左视图拍摄",
        "正视图拍摄",
        "右视图拍摄",
        "三视图完成回HOME",
        "选择左视图定靶点",
        "选择正视图定靶点",
        "选择右视图定靶点",
    ]
    for label in required:
        assert label in SOURCE


def test_capture_buttons_do_not_use_old_english_labels() -> None:
    old_labels = [
        "Capture L",
        "Capture F",
        "Capture R",
        "Capture Done HOME",
        "Select L prestart",
        "Select F prestart",
        "Select R prestart",
    ]
    for label in old_labels:
        assert label not in SOURCE


if __name__ == "__main__":
    test_capture_buttons_are_chinese()
    test_capture_buttons_do_not_use_old_english_labels()
    print("upper console Chinese label checks passed")
