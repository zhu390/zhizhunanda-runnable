#ifndef ROBOT_TARGET_H_
#define ROBOT_TARGET_H_

#include <stdint.h>
#include <stdbool.h>
#include "robot.h"

#ifdef __cplusplus
extern "C" {
#endif

//预开始位置
#ifndef TARGET_PRE_X
#define TARGET_PRE_X                 (0.0f)      /* 定靶预定位 X 方向偏移，单位 mm */
#endif
#ifndef TARGET_PRE_Y
#define TARGET_PRE_Y                 (-130.0f)    /* 定靶预定位 Y 方向偏移，单位 mm，负值表示向前靠近目标 */
#endif
#ifndef TARGET_PRE_Z
#define TARGET_PRE_Z                 (-15.0f)      /* 定靶预定位 Z 方向偏移，单位 mm */
#endif



//视觉伺服开关
#ifndef TARGET_USE_VISUAL_SERVO
#define TARGET_USE_VISUAL_SERVO      (1)        /* 1=使用视觉伺服闭环，0=不使用视觉伺服闭环 */
#endif

//视觉伺服参数
#define TARGET_VS_KX_MM_S_PER_PX      (0.09f)    /* 200 mm 工作距离：X 轴约 0.5/s 闭环增益 */
#define TARGET_VS_KZ_MM_S_PER_PX      (-0.09f)   /* 图像向下对应机械臂 -Z，保留已验证方向 */
#define TARGET_VS_MAX_SPEED_MM_S      (1.0f)     /* 粗调速度上限，避免大误差阶段收敛过慢 */
#define TARGET_VS_FINE_MAX_SPEED_MM_S (0.5f)     /* 30 px 内精调速度上限，约 1.2 px/200 ms */
#define TARGET_VS_CMD_TIMEOUT_MS      (300u)     /* 容忍一次 200 ms 视觉帧抖动，仍小于视觉失效时间 */

//末端auto修正参数
#define TARGET_KX_MM_PER_PX          (0.38f)    /* 视觉 dcx 到机械臂 X 小步修正的比例，单位 mm/px */
#define TARGET_KY_MM_PER_PX          (0.35f)     /* 视觉 dcy 到机械臂 Z 小步修正的比例，单位 mm/px */
#define TARGET_MAX_STEP_MM           (2.5f)      /* 单次视觉对准最大修正步长，单位 mm */
#define TARGET_ALIGN_TOL_PX_COARSE   (30.0f)     /* 粗/精定位切换阈值，单位 px */
#define TARGET_MAX_STEP_MM_FINE      (0.8f)      /* 精定位阶段单次最大修正步长，单位 mm */
#define TARGET_ALIGN_TOL_PX          (2.0f)     /* 视觉误差进入该像素阈值内视为单帧对准 */
#define TARGET_ALIGN_STABLE_COUNT    (3u)        /* 连续满足对准阈值的帧数，达到后进入确认状态 */  
#define TARGET_CONFIRM_STABLE_COUNT  (2u)        /* 输出前二次确认所需连续新视觉帧数，3+2 共 5 帧 */
#define TARGET_ALIGN_PERIOD_MS       (250u)      /* 视觉闭环小步修正周期，单位 ms */
#define TARGET_VISION_VALID_MS       (500u)      /* 视觉数据有效期，超时后退出对准/输出 */
#define TARGET_PRE_POSITION_TIMEOUT_MS (8000u)    /* 预定位最长等待时间，超时进入恢复 */
#define TARGET_READY_STATUS_PERIOD_MS (300u)      /* 等待视觉误差时 READY 状态重发周期，单位 ms */
#define TARGET_OUTPUT_MAX_MS          (10000u)    /* 单次激光输出硬上限，默认 10 秒 */
#define TARGET_FIRE_KEY_DEBOUNCE_MS   (40u)       /* P000 按键去抖时间 */

typedef enum
{
    ROBOT_TARGET_STATE_INIT = 0,
    ROBOT_TARGET_STATE_WAIT_DETECT,
    ROBOT_TARGET_STATE_ALIGN,
    ROBOT_TARGET_STATE_CONFIRM,
    ROBOT_TARGET_STATE_OUTPUT,
    ROBOT_TARGET_STATE_HOLD,
    ROBOT_TARGET_STATE_FAULT,
} robot_target_state_t;

typedef enum
{
    ROBOT_TARGET_EVENT_NONE = 0,
    ROBOT_TARGET_EVENT_READY,
    ROBOT_TARGET_EVENT_VISION_RECOVERED,
    ROBOT_TARGET_EVENT_ALIGN_DONE,
    ROBOT_TARGET_EVENT_VISION_LOST,
    ROBOT_TARGET_EVENT_ALIGNMENT_LOST,
    ROBOT_TARGET_EVENT_HOLD,
    ROBOT_TARGET_EVENT_FAULT,
} robot_target_event_t;

typedef struct
{
    bool has_vision;
    bool vision_valid;
    int16_t dcx;
    int16_t dcy;
    uint32_t now_ms;
    bool fire_button;
} target_obs_t;

extern volatile bool ROBOT_TARGET_ENABLED;
extern volatile bool ROBOT_TARGET_FIRE_ENABLE;

void robot_target_init(void);
bool robot_target_start_at_current_pose(void);
void robot_target_stop_hold(void);
robot_target_event_t robot_target_event_consume(void);
bool robot_target_output_requested(void);
robot_target_state_t robot_target_state_get(void);
bool robot_target_enable_request(void);
void robot_target_disable_request(void);
void robot_target_mark_preposition_ready_once(void);
void robot_target_step(const target_obs_t *obs);

#ifdef __cplusplus
}
#endif

#endif /* ROBOT_TARGET_H_ */
