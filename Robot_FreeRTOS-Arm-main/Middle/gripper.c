#include "gripper.h"

#include "Emm_V5.h"
#include "bsp_can.h"
#include "bsp_uart.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"


//定义夹取水果类型，不同类型的参数已经定义
static const gripper_grasp_cfg_t s_gripper_presets[] = {
    [GRIPPER_FRUIT_STRAWBERRY] = {
        .addr = GRIPPER_DEFAULT_ADDR,
        .close_dir = 0u,				/* 0=CW，其它=CCW（按你机构实际方向设置） */
        .close_rpm = 100u,              /* 闭合速度（RPM），建议 30~120 */
        .close_acc = 50u,               /* 加速度（0~255），建议 10~80 */
        .current_limit_ma = 500,		/* 夹住判定电流阈值（mA） */
        .blind_ms = 60,					/* 启动盲区：闭合开始后的忽略时间（ms） */
        .debounce_ms = 40u,         	/* 消抖时间：连续满足阈值才算夹住（ms） */
        .poll_ms = 10u,					/* 轮询周期（ms），建议 10~50 */
        .timeout_ms = 4500u,			/* 整个夹取流程超时（ms），建议 1500~4000 */
    },
    [GRIPPER_FRUIT_TOMATO] = {
        .addr = GRIPPER_DEFAULT_ADDR,
        .close_dir = 0u,
        .close_rpm = 70u,
        .close_acc = 40u,
        .current_limit_ma = 550u,
        .blind_ms = 150u,
        .debounce_ms = 80u,
        .poll_ms = 20u,
        .timeout_ms = 2500u,
    },
    [GRIPPER_FRUIT_GRAPE] = {
        .addr = GRIPPER_DEFAULT_ADDR,
        .close_dir = 0u,
        .close_rpm = 50u,
        .close_acc = 20u,
        .current_limit_ma = 300u,
        .blind_ms = 150u,
        .debounce_ms = 80u,
        .poll_ms = 20u,
        .timeout_ms = 2500u,
    },
};

#if (GRIPPER_VOFA_PLOT_ENABLE == 1u)
static inline void gripper_vofa_emit(uint32_t tick_ms, uint16_t current_ma, uint16_t limit_ma, uint8_t state)
{
#if (GRIPPER_VOFA_NAMED_FRAME == 1u)
    LOG("gripper:%u,%u,%u\r\n",
        (unsigned)current_ma,
        (unsigned)limit_ma,
        (unsigned)state);
#else
    LOG("%u,%u,%u\r\n",
        (unsigned)current_ma,
        (unsigned)limit_ma,
        (unsigned)state);
#endif
}
#else
#define gripper_vofa_emit(tick_ms, current_ma, limit_ma, state) ((void)0)
#endif


/* 说明：
 * - CAN 接收是全局共享的 `g_can_context`（单一 rxData + 单一信号量）。
 * - 为避免夹爪 API 被多任务并发调用，本模块内部加一把互斥锁。
 * - 但这把锁不能阻止“别的任务也在用 CAN”，因此建议在夹取阶段尽量不要并发跑机械臂 PID。
 */
bool gripper_get_preset(gripper_fruit_t type, gripper_grasp_cfg_t *out_cfg)
{
    if (out_cfg == NULL) {
        return false;
    }
    if ((uint32_t)type >= (uint32_t)GRIPPER_FRUIT_MAX) {
        return false;
    }
    *out_cfg = s_gripper_presets[type];
    if (out_cfg->addr == 0u) {
        out_cfg->addr = GRIPPER_DEFAULT_ADDR;
    }
    return true;
}


static gripper_result_t lock_gripper(uint32_t timeout_ms)
{
    /* 夹爪与机械臂关节共享同一条 CAN 总线、同一套“单缓冲+binary sem”的接收架构。
     * 因此这里必须串行化请求-应答，否则会出现回包被其它任务抢走/覆盖。
     */
    if (!BSP_CAN_Lock(timeout_ms)) {
        return GRIPPER_ERR_TIMEOUT;
    }
    return GRIPPER_OK;
}

static void unlock_gripper(void)
{
    BSP_CAN_Unlock();
}

/* 等待某个电机的某个功能码回包。
 * 返回时：out_buf[0..dlc-1] 为接收数据（最多 8 字节）。
 */
static gripper_result_t wait_reply(uint8_t addr,
                                  uint8_t expected_func,
                                  uint8_t *out_buf,
                                  uint8_t *out_dlc,
                                  uint32_t timeout_ms)
{
    if ((out_buf == NULL) || (out_dlc == NULL)) {
        return GRIPPER_ERR_PARAM;
    }

    if (g_can_context.can_rx_sem == NULL) {
        return GRIPPER_ERR_CAN_TIMEOUT;
    }

    /* 注意：这里不要“清空/偷走”全局 CAN 信号量。
     * 工程里 CAN 接收信号量是全局共享的（单一 binary sem），清空会引入竞态：
     * - 可能把刚到的正确回包自己清掉（你指出的情况）
     * - 也可能误伤其它等待 CAN 回包的任务
     *
     * wait_reply() 通过 (addr + func) 过滤，并在循环中继续等待，因此允许“先收到无关帧”。
     */

    TickType_t start = xTaskGetTickCount();
    TickType_t deadline = start + pdMS_TO_TICKS(timeout_ms);

    while (1)
    {
        TickType_t now = xTaskGetTickCount();
        if (now >= deadline) {
            return GRIPPER_ERR_CAN_TIMEOUT;
        }

        TickType_t wait_ticks = deadline - now;
        if (pdPASS != xSemaphoreTake(g_can_context.can_rx_sem, wait_ticks)) {
            return GRIPPER_ERR_CAN_TIMEOUT;
        }

        /* 关键：RX 数据是 ISR 写入的全局缓冲。
         * 这里用“关中断快照”的方式，避免在解析/拷贝期间被下一帧覆盖。
         * 注意：如果两帧非常密集到来（binary sem 无法计数），仍可能丢掉更早的一帧；
         * 这是当前全局 CAN 接收架构的限制。
         */
        uint32_t ext_id;
        uint8_t dlc;
        uint8_t data[8] = {0};

        __disable_irq();
        ext_id = g_can_context.CAN_RxMsg.ExtId;
        dlc = (uint8_t) g_can_context.CAN_RxMsg.DLC;
        if (dlc > 8u) {
            dlc = 8u;
        }
        for (uint8_t i = 0; i < dlc; i++) {
            data[i] = g_can_context.rxData[i];
        }
        __enable_irq();

        uint8_t rx_addr = (uint8_t) (ext_id >> 8);

        /* 过滤：只接收目标电机、目标功能码 */
        if (rx_addr != addr) {
            continue;
        }
        if (dlc < 2u) {
            continue;
        }
        if (data[0] != expected_func) {
            continue;
        }

        /* 多数回包以 0x6B 结尾（校验字节），不是强制，但可做弱校验 */
        if (data[dlc - 1u] != 0x6Bu) {
            /* 不直接失败，继续等下一帧 */
            continue;
        }

        for (uint8_t i = 0; i < dlc; i++) {
            out_buf[i] = data[i];
        }
        *out_dlc = dlc;
        return GRIPPER_OK;

    }
}

/* ===== 内部不加锁版本（要求调用方已持有 CAN 总线锁） ===== */
static gripper_result_t gripper_read_phase_current_ma_unlocked(uint8_t addr, uint16_t *out_ma)
{
    if (out_ma == NULL) {
        return GRIPPER_ERR_PARAM;
    }

    uint8_t rx[8] = {0};
    uint8_t dlc = 0;

    Emm_V5_Read_Sys_Params(addr, S_CPHA); /* 0x27 */
    gripper_result_t r = wait_reply(addr, 0x27u, rx, &dlc, 50);
    if (r != GRIPPER_OK) {
        return r;
    }

    /* 兼容两种常见格式：
     * - [27][hi][lo][6B] -> dlc==4
     * - [27][sign][hi][lo][6B] -> dlc==5
     */
    if (dlc == 4u) {
        *out_ma = (uint16_t) (((uint16_t) rx[1] << 8) | (uint16_t) rx[2]);
        return GRIPPER_OK;
    }
    if (dlc >= 5u) {
        *out_ma = (uint16_t) (((uint16_t) rx[2] << 8) | (uint16_t) rx[3]);
        return GRIPPER_OK;
    }

    return GRIPPER_ERR_CAN_BAD_FRAME;
}

static gripper_result_t gripper_read_flags_unlocked(uint8_t addr, uint8_t *out_flags)
{
    if (out_flags == NULL) {
        return GRIPPER_ERR_PARAM;
    }

    uint8_t rx[8] = {0};
    uint8_t dlc = 0;

    Emm_V5_Read_Sys_Params(addr, S_FLAG); /* 0x3A */
    gripper_result_t r = wait_reply(addr, 0x3Au, rx, &dlc, 50);
    if (r != GRIPPER_OK) {
        return r;
    }

    /* 常见格式：[3A][flag][6B] */
    if (dlc == 3u) {
        *out_flags = rx[1];
        return GRIPPER_OK;
    }

    return GRIPPER_ERR_CAN_BAD_FRAME;
}

void gripper_init(void)
{
    /* 夹爪模块不再维护私有 mutex：统一使用 `BSP_CAN_Lock/Unlock` 串行化 CAN 请求-应答。
     * 因此这里保持空实现即可（可重复调用）。
     */
}


gripper_result_t gripper_stop(uint8_t addr)
{
    if (addr == 0u) {
        addr = GRIPPER_DEFAULT_ADDR;
    }

    gripper_result_t r = lock_gripper(200);
    if (r != GRIPPER_OK) {
        return r;
    }

    Emm_V5_Stop_Now(addr, false);
    unlock_gripper();
    return GRIPPER_OK;
}

gripper_result_t gripper_open(void)
{
    uint8_t addr = GRIPPER_DEFAULT_ADDR;

    gripper_result_t r = lock_gripper(8000);
    if (r != GRIPPER_OK) {
        return r;
    }

    /* 确保使能（不等待回包） */
    Emm_V5_En_Control(addr, true, false);

    /* 固定参数位置模式（绝对） */
    Emm_V5_Pos_Control(addr, 1u, 150u, 50u, 15000u, true, false);

    /* 等待到位回包：06 FD 9F 6B */
    uint8_t rx[8] = {0};
    uint8_t dlc = 0;
    r = wait_reply(addr, 0xFDu, rx, &dlc, 4000u);
    if (r != GRIPPER_OK) {
        unlock_gripper();
        return r;
    }
    /* 放宽到位判定：收到 FD 回包即可认为成功（末字节应为 6B） */
    if (dlc < 2u) {
        unlock_gripper();
        return GRIPPER_ERR_CAN_BAD_FRAME;
    }

    unlock_gripper();
    return GRIPPER_OK;
}





gripper_result_t gripper_read_phase_current_ma(uint8_t addr, uint16_t *out_ma)
{
    if (out_ma == NULL) {
        return GRIPPER_ERR_PARAM;
    }
    if (addr == 0u) {
        addr = GRIPPER_DEFAULT_ADDR;
    }

    gripper_result_t r = lock_gripper(200);
    if (r != GRIPPER_OK) {
        return r;
    }

    r = gripper_read_phase_current_ma_unlocked(addr, out_ma);
    unlock_gripper();
    return r;
}


gripper_result_t gripper_read_flags(uint8_t addr, uint8_t *out_flags)
{
    if (out_flags == NULL) {
        return GRIPPER_ERR_PARAM;
    }
    if (addr == 0u) {
        addr = GRIPPER_DEFAULT_ADDR;
    }

    gripper_result_t r = lock_gripper(200);
    if (r != GRIPPER_OK) {
        return r;
    }

    r = gripper_read_flags_unlocked(addr, out_flags);
    unlock_gripper();
    return r;
}


gripper_result_t gripper_grasp_force(const gripper_grasp_cfg_t *cfg, uint16_t *out_touch_current_ma)
{
    if ((cfg == NULL) || (cfg->current_limit_ma == 0u)) {
        return GRIPPER_ERR_PARAM;
    }

    gripper_grasp_cfg_t c = *cfg;
    if (c.addr == 0u) {
        c.addr = GRIPPER_DEFAULT_ADDR;
    }
    if (c.poll_ms == 0u) {
        c.poll_ms = 20u;
    }
    if (c.timeout_ms == 0u) {
        c.timeout_ms = 2000u;
    }

    gripper_result_t r = lock_gripper(1000);
    if (r != GRIPPER_OK) {
        return r;
    }

    /* 进入夹取前，尽量清理堵转保护（不等待回包） */
    Emm_V5_Reset_Clog_Pro(c.addr);

    /* 确保使能 */
    Emm_V5_En_Control(c.addr, true, false);

    /* 开始闭合（速度模式） */
    Emm_V5_Vel_Control(c.addr, c.close_dir, c.close_rpm, c.close_acc, false);

    /* 启动盲区：避免刚启动电流尖峰误判 */
    if (c.blind_ms > 0u) {
        vTaskDelay(pdMS_TO_TICKS(c.blind_ms));
    }

    TickType_t start = xTaskGetTickCount();
    TickType_t deadline = start + pdMS_TO_TICKS(c.timeout_ms);

    uint32_t ok_ms = 0u;
    uint16_t last_current = 0u;
 
    while (xTaskGetTickCount() < deadline)
    {
        /* 读电流（内部无锁版本；当前函数外层已持锁） */
        uint16_t cur_ma = 0u;
        r = gripper_read_phase_current_ma_unlocked(c.addr, &cur_ma);
        if (r != GRIPPER_OK)
        {
            /* CAN 读失败：直接退出，避免一直夹 */
            gripper_vofa_emit((uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS), last_current, c.current_limit_ma, 4u);
            (void) Emm_V5_Stop_Now(c.addr, false);
            unlock_gripper();
            return r;
        }

        last_current = cur_ma;
        gripper_vofa_emit((uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS), cur_ma, c.current_limit_ma, 0u);


        /* 读堵转标志（失败不致命，尽力而为） */
        uint8_t flags = 0u;
        gripper_result_t rf = gripper_read_flags_unlocked(c.addr, &flags);
        if ((rf == GRIPPER_OK) && ((flags & GRIPPER_FLAG_STALL_PROT) || (flags & GRIPPER_FLAG_STALL)))
        {
            gripper_vofa_emit((uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS), cur_ma, c.current_limit_ma, 2u);
            (void) Emm_V5_Stop_Now(c.addr, false);
            unlock_gripper();
            return GRIPPER_ERR_STALL;
        }



        /* 电流判定 + 消抖 */
        if (cur_ma >= c.current_limit_ma)
        {
            ok_ms += c.poll_ms;
            if (ok_ms >= c.debounce_ms)
            {
                gripper_vofa_emit((uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS), cur_ma, c.current_limit_ma, 1u);
                (void) Emm_V5_Stop_Now(c.addr, false);
                unlock_gripper();
                if (out_touch_current_ma != NULL) {
                    *out_touch_current_ma = cur_ma;
                }
                return GRIPPER_OK;
            }

        }
        else
        {
            ok_ms = 0u;
        }

        vTaskDelay(pdMS_TO_TICKS(c.poll_ms));
    }

    /* 超时：停止并返回 */
    gripper_vofa_emit((uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS), last_current, c.current_limit_ma, 3u);
    (void) Emm_V5_Stop_Now(c.addr, false);

    unlock_gripper();

    if (out_touch_current_ma != NULL) {
        *out_touch_current_ma = last_current;
    }

    return GRIPPER_ERR_TIMEOUT;
}

