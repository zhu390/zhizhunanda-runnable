#ifndef __BSP_LED_H_
#define __BSP_LED_H_

#include <stdbool.h>
#include "hal_data.h"

void LED_Init(void);
void BSP_TargetReadyLed_Set(bool on);
void BSP_TargetOutputLed_Set(bool on);

#define USER_LED_ON          R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_04_PIN_00, BSP_IO_LEVEL_LOW)
#define RED_LED_ON           R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_00_PIN_03, BSP_IO_LEVEL_LOW)
#define BLUE_LED_ON          R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_05_PIN_03, BSP_IO_LEVEL_LOW)

#define USER_LED_OFF         R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_04_PIN_00, BSP_IO_LEVEL_HIGH)
#define RED_LED_OFF          R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_00_PIN_03, BSP_IO_LEVEL_HIGH)
#define BLUE_LED_OFF         R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_05_PIN_03, BSP_IO_LEVEL_HIGH)

#define USER_LED_TODGGLE     R_PORT4->PODR ^= 1 << (BSP_IO_PORT_04_PIN_00 & 0xFF)
#define RED_LED_TODGGLE      R_PORT1->PODR ^= 1 << (BSP_IO_PORT_01_PIN_14 & 0xFF)
#define BLUE_LED_TODGGLE     R_PORT1->PODR ^= 1 << (BSP_IO_PORT_01_PIN_15 & 0xFF)

#endif
