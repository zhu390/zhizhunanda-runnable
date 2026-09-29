#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "hal_data.h"
#include <stdint.h>
#include <stdbool.h>
#include "robot.h" 
#include "r_can_api.h"
#include "FreeRTOS.h"
#include "semphr.h"

/* ============================================================ */
/*                STM32 风格的数据结构定义（兼容层）           */
/* ============================================================ */

/* 
 * 模拟 STM32 的 RxHeader 结构，仅保留关键字段 ExtId 和 DLC
 * 为保持上层逻辑兼容性而设计
 */
typedef struct {
    uint32_t ExtId; // 扩展帧 ID (29位)
    uint32_t StdId; // 标准帧 ID (11位，当前未使用，保留为兼容性)
    uint32_t DLC;   // 数据长度码 (0-8字节)
} CAN_RxHeader_Mock_t;

/* 
 * CAN 上下文结构体
 * 设计原则：兼容 STM32 原有接口风格，降低移植成本
 */
typedef struct {
    /* 模拟 STM32 的 RxHeader */
    CAN_RxHeader_Mock_t CAN_RxMsg;
    
    /* 接收数据缓冲区 (STM32 原为32字节，实际CAN标准帧仅8字节) */
    uint8_t rxData[32]; 
    
    /* 发送数据缓冲区 (FSP底层不提供缓冲，此处为上层协议预留) */
    uint8_t txData[32];

    /* CAN 接收信号量，用于替代 rxFrameFlag */
    SemaphoreHandle_t can_rx_sem;

    /* Batch position feedback cache for motor address 1..6. */
    volatile uint8_t motor_rx_buf[6][8];
    volatile uint8_t motor_rx_dlc[6];
    volatile uint8_t motor_rx_flag[6];

    /* Per-motor stop reply (0xFE) flag, set by ISR. */
    volatile uint8_t motor_stop_flag[6];

} CAN_Context_t;

/* ============================================================ */
/*                      全局变量声明                            */
/* ============================================================ */

extern CAN_Context_t g_can_context;

/* 
 * !!! 重要数据结构映射 !!!
 * 为保持代码兼容性，将全局上下文映射为\"can\"变量
 * 使用示例：can.rxFrameFlag 可直接访问接收标志
 * 注意：此宏定义使上层代码无需修改即可访问新数据结构
 */
#define can g_can_context

/*
 * CAN 接收/发送完成标志
 */
extern volatile bool g_can_tx_complete;

/* ============================================================ */
/*                        接口函数声明                          */
/* ============================================================ */

/**
 * @brief 初始化 CAN 外设
 * @note  此函数会调用 FSP 生成的 open API
 */
void BSP_CAN_Init(void);

/**
 * @brief 底层 CAN 发送函数
 * @param id CAN ID (标准帧或扩展帧)
 * @param id_mode ID 模式 (CAN_ID_MODE_STANDARD 或 CAN_ID_MODE_EXTENDED)
 * @param p_data 指向要发送数据的指针
 * @param dlc 数据长度 (0-8)
 * @return fsp_err_t FSP错误码
 */
fsp_err_t BSP_CAN_Send(uint32_t id, can_id_mode_t id_mode, uint8_t *p_data, uint8_t dlc);
void BSP_CAN_PrintInfo(const char *tag);

/**
 * @brief 上层应用接口：发送电机指令
 * @param cmd 指向命令数据的指针 (首字节为电机地址)
 * @param len 命令总长度
 */
void can_SendCmd(uint8_t *cmd, uint8_t len);

/* ============================================================ */
/*                 CAN 应答等待与总线互斥(请求-应答)              */
/* ============================================================ */

/**
 * @brief 锁住 CAN 总线（用于串行化“发命令-等回包”的事务）。
 * @note  由于当前工程的接收侧是“单缓冲 + binary 信号量”，
 *        多任务并发请求-应答会结构性丢帧/错帧；建议必须串行化。
 */
bool BSP_CAN_Lock(uint32_t timeout_ms);

/**
 * @brief 释放 CAN 总线锁
 */
void BSP_CAN_Unlock(void);

void BSP_CAN_ClearMotorFlags(void);
bool BSP_CAN_WaitAllMotors(uint8_t joint_num, uint32_t timeout_ms);
void BSP_CAN_ClearStopFlags(uint8_t joint_num);
bool BSP_CAN_WaitStopAll(uint8_t joint_num, uint32_t timeout_ms);
void BSP_CAN_DrainRx(void);

/**
 * @brief 等待指定电机(addr)的指定功能码(expected_func)回包。
 * @param addr 电机地址(1..)
 * @param expected_func 期望的功能码（回包 data[0]）
 * @param out_buf 输出数据缓存(>=8字节)
 * @param out_dlc 输出数据长度(0..8)
 * @param timeout_ms 超时时间(ms)
 * @param out_ext_id 可选输出扩展ID(可传NULL)
 * @return true=收到匹配回包，false=超时/参数错误
 */
bool BSP_CAN_WaitReply(uint8_t addr,
                      uint8_t expected_func,
                      uint8_t *out_buf,
                      uint8_t *out_dlc,
                      uint32_t timeout_ms,
                      uint32_t *out_ext_id);

#endif /* BSP_CAN_H */
