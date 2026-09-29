#include "chassis.h"

#include <stdbool.h>

#include "Emm_V5.h"
#include "bsp_can.h"
#include "bsp_uart.h"
#include "FreeRTOS.h"
#include "task.h"

#define CHASSIS_LF_ADDR                 (7u)
#define CHASSIS_LR_ADDR                 (8u)
#define CHASSIS_RF_ADDR                 (9u)
#define CHASSIS_RR_ADDR                 (10u)

#define CHASSIS_DIR_CW                  (0u)
#define CHASSIS_DIR_CCW                 (1u)

/* 轮位缩写说明：
 * LF = Left Front  (左前轮)
 * LR = Left Rear   (左后轮)
 * RF = Right Front (右前轮)
 * RR = Right Rear  (右后轮)
 * _FORWARD_DIR 表示该轮“前进”时对应驱动器方向位（CW/CCW）
 */
#ifndef CHASSIS_LF_FORWARD_DIR
#define CHASSIS_LF_FORWARD_DIR          CHASSIS_DIR_CW
#endif
#ifndef CHASSIS_LR_FORWARD_DIR
#define CHASSIS_LR_FORWARD_DIR          CHASSIS_DIR_CW
#endif
#ifndef CHASSIS_RF_FORWARD_DIR
#define CHASSIS_RF_FORWARD_DIR          CHASSIS_DIR_CCW
#endif
#ifndef CHASSIS_RR_FORWARD_DIR
#define CHASSIS_RR_FORWARD_DIR          CHASSIS_DIR_CCW
#endif

#define CHASSIS_SPEED_MIN_RPM           (20u)
#define CHASSIS_SPEED_MAX_RPM           (4000u)
#define CHASSIS_ACCEL                   (0u)
#define CHASSIS_STOP_DECEL              (0u)
#define CHASSIS_SYNC_BROADCAST_ADDR     (0u)



typedef struct
{
    chassis_mode_t target_mode;
    uint16_t target_speed;
    chassis_mode_t applied_mode;
    uint16_t applied_speed;
} chassis_ctx_t;

static chassis_ctx_t s_chassis = {
    .target_mode = CHASSIS_MODE_STOP,
    .target_speed = 200u,
    .applied_mode = CHASSIS_MODE_STOP,
    .applied_speed = 0u,
};

static uint16_t clamp_speed(uint16_t speed)
{
    if (speed == 0u)
    {
        return 0u;
    }
    if (speed < CHASSIS_SPEED_MIN_RPM)
    {
        return CHASSIS_SPEED_MIN_RPM;
    }
    if (speed > CHASSIS_SPEED_MAX_RPM)
    {
        return CHASSIS_SPEED_MAX_RPM;
    }
    return speed;
}

static void wheel_set_speed(uint8_t addr, int16_t signed_rpm, uint8_t forward_dir)
{
    if (signed_rpm == 0)
    {
        Emm_V5_Vel_Control(addr, forward_dir, 0u, CHASSIS_STOP_DECEL, true);
        return;
    }

    uint16_t rpm = (uint16_t)((signed_rpm > 0) ? signed_rpm : -signed_rpm);
    uint8_t dir = (signed_rpm > 0) ? forward_dir : (uint8_t)!forward_dir;

    Emm_V5_Vel_Control(addr, dir, rpm, CHASSIS_ACCEL, true);
}



static void chassis_apply(chassis_mode_t mode, uint16_t speed)
{
    int16_t left = 0;
    int16_t right = 0;

    switch (mode)
    {
        case CHASSIS_MODE_FORWARD:
            left = (int16_t)speed;
            right = (int16_t)speed;
            break;

        case CHASSIS_MODE_BACKWARD:
            left = -(int16_t)speed;
            right = -(int16_t)speed;
            break;

        case CHASSIS_MODE_TURN_LEFT:
            /* 差速左转：同向前进，右侧内轮慢，左侧外轮快 */
            left = (int16_t)speed;
            right = (int16_t)(speed / 2u);
            break;

        case CHASSIS_MODE_TURN_RIGHT:
            /* 差速右转：同向前进，左侧内轮慢，右侧外轮快 */
            left = (int16_t)(speed / 2u);
            right = (int16_t)speed;
            break;

        case CHASSIS_MODE_STOP:
        default:
            left = 0;
            right = 0;
            break;
    }

    if (!BSP_CAN_Lock(20u))
    {
        LOG("chassis_apply: can lock timeout\r\n");
        return;
    }

    wheel_set_speed(CHASSIS_LF_ADDR, left, CHASSIS_LF_FORWARD_DIR);
    wheel_set_speed(CHASSIS_LR_ADDR, left, CHASSIS_LR_FORWARD_DIR);
    wheel_set_speed(CHASSIS_RF_ADDR, right, CHASSIS_RF_FORWARD_DIR);
    wheel_set_speed(CHASSIS_RR_ADDR, right, CHASSIS_RR_FORWARD_DIR);

    Emm_V5_Synchronous_motion(CHASSIS_SYNC_BROADCAST_ADDR);

    BSP_CAN_Unlock();


    s_chassis.applied_mode = mode;
    s_chassis.applied_speed = speed;
}

void chassis_init(void)
{
    if (!BSP_CAN_Lock(50u))
    {
        LOG("chassis_init: can lock timeout\r\n");
        return;
    }

    Emm_V5_En_Control(CHASSIS_LF_ADDR, true, false);
    Emm_V5_En_Control(CHASSIS_LR_ADDR, true, false);
    Emm_V5_En_Control(CHASSIS_RF_ADDR, true, false);
    Emm_V5_En_Control(CHASSIS_RR_ADDR, true, false);

    BSP_CAN_Unlock();

    s_chassis.target_mode = CHASSIS_MODE_STOP;
    s_chassis.target_speed = 200u;
    s_chassis.applied_mode = CHASSIS_MODE_STOP;
    s_chassis.applied_speed = 0u;


    chassis_periodic_10ms();
    LOG("chassis_init done\r\n");
}

void chassis_set_mode(chassis_mode_t mode, uint16_t speed_rpm)
{
    uint16_t clamped = clamp_speed(speed_rpm);
    if (clamped != speed_rpm)
    {
        LOG("chassis speed clamp: req=%u -> %u\r\n", (unsigned)speed_rpm, (unsigned)clamped);
    }

    s_chassis.target_mode = mode;
    s_chassis.target_speed = clamped;
}


void chassis_stop(void)
{
    chassis_set_mode(CHASSIS_MODE_STOP, 0u);
}

void chassis_periodic_10ms(void)
{
    if ((s_chassis.target_mode != s_chassis.applied_mode) ||
        (s_chassis.target_speed != s_chassis.applied_speed))
    {
        chassis_apply(s_chassis.target_mode, s_chassis.target_speed);
    }
}

chassis_mode_t chassis_get_mode(void)
{
    return s_chassis.target_mode;
}

uint16_t chassis_get_speed_rpm(void)
{
    return s_chassis.target_speed;
}
