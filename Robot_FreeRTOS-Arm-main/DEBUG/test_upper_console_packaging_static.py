from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
ENTRY = (ROOT / "UpperComputer" / "ra6m5_upper_console.py").read_text(encoding="utf-8")


def test_exe_entry_has_no_sibling_protocol_import() -> None:
    assert "ra6m5_protocol" not in ENTRY


if __name__ == "__main__":
    test_exe_entry_has_no_sibling_protocol_import()
    print("upper console packaging checks passed")
