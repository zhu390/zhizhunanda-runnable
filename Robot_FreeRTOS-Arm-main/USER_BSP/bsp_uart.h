#ifndef __BSP_UART_H__
#define __BSP_UART_H__

#include "hal_data.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#define SAFE_BUFF_SIZE 2
#define UART1_RX_BUFF_SIZE 128
#define MQTT_RX_BUFF_SIZE 512

/* 串口相关初始化与打印函数声明 */
void BSP_UART_Init(void);

/* 任务级线程安全打印 */
void safe_printf(const char *format, ...);
/* 中断安全打印（可能存在冲突风险，仅用于紧急情况） */
void safe_printf_from_isr(const char *format, ...);

/* 打印宏封装，方便移植 */
#define LOG(format, ...)            safe_printf(format, ##__VA_ARGS__)
#define LOG_FROM_ISR(format, ...)   safe_printf_from_isr(format, ##__VA_ARGS__)


#endif /* __BSP_UART_H__ */


