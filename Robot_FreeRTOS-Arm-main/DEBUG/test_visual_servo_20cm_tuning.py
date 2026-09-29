from __future__ import annotations

import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
TARGET_H = ROOT / "APP" / "robot_target.h"


def macro_value(source: str, name: str) -> float:
    match = re.search(
        rf"^#define\s+{re.escape(name)}\s+\((-?[0-9]+(?:\.[0-9]+)?)[fu]?\)",
        source,
        flags=re.MULTILINE,
    )
    if match is None:
        raise AssertionError(f"missing numeric macro: {name}")
    return float(match.group(1))


def main() -> None:
    source = TARGET_H.read_text(encoding="utf-8")
    kx = macro_value(source, "TARGET_VS_KX_MM_S_PER_PX")
    kz = macro_value(source, "TARGET_VS_KZ_MM_S_PER_PX")
    coarse_limit = macro_value(source, "TARGET_VS_MAX_SPEED_MM_S")
    fine_limit = macro_value(source, "TARGET_VS_FINE_MAX_SPEED_MM_S")
    timeout_ms = macro_value(source, "TARGET_VS_CMD_TIMEOUT_MS")
    tolerance_px = macro_value(source, "TARGET_ALIGN_TOL_PX")

    # mono_calib_params.npz: fx=589.83, fy=591.45. At 200 mm,
    # one pixel is about 0.338 mm. K=0.17 therefore gives a moderate
    # closed-loop rate of about 0.5/s without changing the verified signs.
    mm_per_px_x = 200.0 / 589.8318638
    mm_per_px_z = 200.0 / 591.44912007
    assert 0.45 <= kx / mm_per_px_x <= 0.55
    assert 0.45 <= abs(kz) / mm_per_px_z <= 0.55
    assert kx > 0.0 and kz < 0.0

    assert coarse_limit == 6.0
    assert fine_limit == 2.0
    assert timeout_ms == 450.0
    assert tolerance_px == 12.0

    # From the 30 px fine-zone boundary to 12 px, the physical travel is
    # about 6.1 mm. At 2 mm/s it should finish in roughly 3.1 seconds.
    fine_travel_mm = (30.0 - tolerance_px) * max(mm_per_px_x, mm_per_px_z)
    assert fine_travel_mm / fine_limit < 3.2
    print("20 cm visual-servo tuning checks passed")


if __name__ == "__main__":
    main()
