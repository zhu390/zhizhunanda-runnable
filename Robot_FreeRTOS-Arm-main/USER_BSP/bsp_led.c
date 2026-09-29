#include "bsp_led.h"

void LED_Init(void)
{
    (void) R_IOPORT_Open(&g_ioport_ctrl, g_ioport.p_cfg);
}

void BSP_TargetReadyLed_Set(bool on)
{
    if (on)
    {
        RED_LED_ON;
    }
    else
    {
        RED_LED_OFF;
    }
}

void BSP_TargetOutputLed_Set(bool on)
{
    if (on)
    {
        BLUE_LED_ON;
    }
    else
    {
        BLUE_LED_OFF;
    }
}
