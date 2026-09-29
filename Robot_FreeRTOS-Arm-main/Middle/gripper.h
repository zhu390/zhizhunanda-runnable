#ifndef GRIPPER_H_
#define GRIPPER_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 夹爪模块（电机地址默认 6）。
 * - 张开：位置模式（回到“张开位”）。
 * - 闭合/夹取：建议用速度模式慢速闭合 + 读取相电流（mA）做力控判定。
 *
 * 注意：当前工程 CAN 接收侧只缓存 8 字节（见 `bsp_can.c`），
 * 因此不建议用 `S_State(0x43)` 这种长回包来取电流；本模块默认使用 `S_CPHA(0x27)`。
 */

#define GRIPPER_DEFAULT_ADDR      (6u)

/* VOFA FireWater 波形输出开关：
 * - 1: 使能（在夹取过程中输出采样行，便于画电流波形）
 * - 0: 关闭（减少串口带宽占用）
 */
#ifndef GRIPPER_VOFA_PLOT_ENABLE
#define GRIPPER_VOFA_PLOT_ENABLE  (0u)
#endif

/* FireWater 帧格式选择：
 * - 0: 纯数值帧（兼容性最高） "ch0,ch1,ch2,ch3\r\n"
 * - 1: 带前缀帧          "gripper:ch0,ch1,ch2,ch3\r\n"
 */
#ifndef GRIPPER_VOFA_NAMED_FRAME
#define GRIPPER_VOFA_NAMED_FRAME  (0u)
#endif

/* 电机状态标志位（来自你提供的说明，位含义以驱动器手册为准） */
#define GRIPPER_FLAG_ENABLED      (0x01u)
#define GRIPPER_FLAG_AT_POS       (0x02u)
#define GRIPPER_FLAG_STALL        (0x04u)
#define GRIPPER_FLAG_STALL_PROT   (0x08u)

typedef enum
{
    GRIPPER_OK = 0,
    GRIPPER_ERR_PARAM,
    GRIPPER_ERR_CAN_TIMEOUT,
    GRIPPER_ERR_CAN_BAD_FRAME,
    GRIPPER_ERR_TIMEOUT,
    GRIPPER_ERR_STALL,
} gripper_result_t;

typedef enum
{
    GRIPPER_FRUIT_STRAWBERRY = 0,
    GRIPPER_FRUIT_TOMATO,
    GRIPPER_FRUIT_GRAPE,
    GRIPPER_FRUIT_MAX,
} gripper_fruit_t;

/* 力控夹取参数（全部单位显式化，避免后续误用） */
typedef struct
{
    uint8_t  addr;               /* 电机地址，默认 6 */

    /* 速度闭合参数 */
    uint8_t  close_dir;          /* 0=CW，其它=CCW（按你机构实际方向设置） */
    uint16_t close_rpm;          /* 闭合速度（RPM），建议 30~120 */
    uint8_t  close_acc;          /* 加速度（0~255），建议 10~80 */

    /* 采样与判定 */
    uint16_t current_limit_ma;   /* 夹住判定电流阈值（mA） */
    uint32_t blind_ms;           /* 启动盲区：闭合开始后的忽略时间（ms），建议 100~300 */
    uint32_t debounce_ms;        /* 消抖时间：连续满足阈值才算夹住（ms），建议 50~150 */
    uint32_t poll_ms;            /* 轮询周期（ms），建议 10~50 */

    /* 保护 */
    uint32_t timeout_ms;         /* 整个夹取流程超时（ms），建议 1500~4000 */
} gripper_grasp_cfg_t;

/* 初始化（创建内部互斥锁等）。可重复调用。 */
void gripper_init(void);

/* 立即停止（任意模式通用）。 */
gripper_result_t gripper_stop(uint8_t addr);

/* 张开：固定参数 + 触发回零。
 * 固定参数：addr=6, dir=0, rpm=60, acc=30, pulses=16000, absolute=true
 * 回零模式：多圈无限位碰撞回零（o_mode=2）
 */
gripper_result_t gripper_open(void);


/* 读取相电流（mA），使用 `S_CPHA(0x27)`。 */
gripper_result_t gripper_read_phase_current_ma(uint8_t addr, uint16_t *out_ma);

/* 读取电机状态标志（使能/到位/堵转/堵转保护），使用 `S_FLAG(0x3A)`。 */
gripper_result_t gripper_read_flags(uint8_t addr, uint8_t *out_flags);

/* 力控夹取：速度闭合 + 电流阈值判定。
 * 成功时：会停止电机，并返回夹住时的电流（可用于调参/记录）。
 */
gripper_result_t gripper_grasp_force(const gripper_grasp_cfg_t *cfg, uint16_t *out_touch_current_ma);

/* 预设参数：按水果类型获取夹爪力控参数 */
bool gripper_get_preset(gripper_fruit_t type, gripper_grasp_cfg_t *out_cfg);

#ifdef __cplusplus
}
#endif

#endif /* GRIPPER_H_ */
