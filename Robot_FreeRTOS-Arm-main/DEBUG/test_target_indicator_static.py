from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read(relative: str) -> str:
    return (ROOT / relative).read_text(encoding="utf-8")


def test_target_leds_follow_confirmed_and_output_states() -> None:
    led_h = read("USER_BSP/bsp_led.h")
    workflow_c = read("APP/robot_workflow.c")

    assert "BSP_TargetReadyLed_Set(bool on);" in led_h
    assert "BSP_TargetOutputLed_Set(bool on);" in led_h
    assert '#include "bsp_led.h"' in workflow_c
    assert "target_aligned" in workflow_c
    assert "BSP_TargetReadyLed_Set(ready);" in workflow_c
    assert "BSP_TargetOutputLed_Set(output);" in workflow_c
    assert "case ROBOT_TARGET_EVENT_ALIGN_DONE:" in workflow_c
    assert "s_workflow.target_aligned = true;" in workflow_c
    assert workflow_c.count("s_workflow.target_aligned = false;") >= 3


if __name__ == "__main__":
    test_target_leds_follow_confirmed_and_output_states()
    print("target indicator static checks passed")
