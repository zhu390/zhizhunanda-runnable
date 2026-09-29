#include "robot_chassis_app.h"

#include <stdbool.h>
#include <stddef.h>

#include "bsp_uart.h"

#include "FreeRTOS.h"
#include "task.h"

#define CHASSIS_TASK_STACK_WORDS        (1024u)
#define CHASSIS_TASK_PRIORITY           (2u)
#define CHASSIS_CMD_TIMEOUT_MS          (1000u)

static TaskHandle_t s_chassis_task_handle = NULL;
static volatile TickType_t s_last_cmd_tick = 0;
static volatile bool s_chassis_cmd_active = false;

static void chassis_task(void *pvParameters);

void robot_chassis_app_init(void)
{
    if (s_chassis_task_handle != NULL)
    {
        return;
    }

    chassis_init();

    taskENTER_CRITICAL();
    s_last_cmd_tick = xTaskGetTickCount();
    s_chassis_cmd_active = false;
    taskEXIT_CRITICAL();

    BaseType_t xr = xTaskCreate(chassis_task, "chassis_task", CHASSIS_TASK_STACK_WORDS, NULL, CHASSIS_TASK_PRIORITY, &s_chassis_task_handle);
    if (xr != pdPASS)
    {
        s_chassis_task_handle = NULL;
        LOG("robot_chassis_app: create chassis_task failed\r\n");
        return;
    }

    robot_chassis_print_help();
}

bool robot_chassis_cmd_drive(chassis_mode_t mode, uint16_t speed_rpm)
{
    if ((mode != CHASSIS_MODE_FORWARD) &&
        (mode != CHASSIS_MODE_BACKWARD) &&
        (mode != CHASSIS_MODE_TURN_LEFT) &&
        (mode != CHASSIS_MODE_TURN_RIGHT))
    {
        return false;
    }

    chassis_set_mode(mode, speed_rpm);

    taskENTER_CRITICAL();
    s_last_cmd_tick = xTaskGetTickCount();
    s_chassis_cmd_active = true;
    taskEXIT_CRITICAL();

    return true;
}

void robot_chassis_cmd_stop(void)
{
    chassis_stop();

    taskENTER_CRITICAL();
    s_last_cmd_tick = xTaskGetTickCount();
    s_chassis_cmd_active = false;
    taskEXIT_CRITICAL();
}

void robot_chassis_print_help(void)
{
    LOG("==== chassis cmd ====\r\n");
    LOG("ch_f <rpm>\r\n");
    LOG("ch_b <rpm>\r\n");
    LOG("ch_l <rpm>\r\n");
    LOG("ch_r <rpm>\r\n");
    LOG("ch_s\r\n");
}

static void chassis_task(void *pvParameters)
{
    (void)pvParameters;

    while (1)
    {
        bool timeout_stop = false;
        TickType_t now_tick = xTaskGetTickCount();

        taskENTER_CRITICAL();
        TickType_t last_tick = s_last_cmd_tick;
        bool active = s_chassis_cmd_active;
        taskEXIT_CRITICAL();

        if (active && ((now_tick - last_tick) >= pdMS_TO_TICKS(CHASSIS_CMD_TIMEOUT_MS)))
        {
            timeout_stop = true;
        }

        if (timeout_stop)
        {
            robot_chassis_cmd_stop();
            LOG("chassis timeout stop (%u ms)\r\n", (unsigned)CHASSIS_CMD_TIMEOUT_MS);
        }

        chassis_periodic_10ms();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

