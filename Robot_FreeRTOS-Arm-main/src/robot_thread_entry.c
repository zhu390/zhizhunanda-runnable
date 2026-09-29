#include "robot_thread.h"
#include "bsp_led.h"
#include "bsp_laser.h"
#include "bsp_can.h"
#include "bsp_uart.h"
#include "bsp_usb_cdc.h"
#include "robot.h"

#include "task.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* robot_thread entry function */

/* pvParameters contains TaskHandle_t */
void robot_thread_entry(void * pvParameters)
{
    FSP_PARAMETER_NOT_USED(pvParameters);

    LED_Init();
    BSP_Laser_Init();
    USER_LED_ON;
    BSP_UART_Init();
    LOG("BSP_UART_Init Success!!!\r\n");
    if (BSP_USB_CDC_Init()) {
        LOG("BSP_USB_CDC_Init Success!!!\r\n");
    } else {
        LOG("BSP_USB_CDC_Init failed, continue SCI9 only.\r\n");
    }
    BSP_CAN_Init();
    LOG("BSP_CAN_Init Success!!!\r\n");
    USER_LED_OFF;

    robot_init();

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char * pcTaskName)
{
    (void) xTask;
    LOG("STACK OVERFLOW: %s\r\n", (pcTaskName != NULL) ? pcTaskName : "unknown");
    taskDISABLE_INTERRUPTS();
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
