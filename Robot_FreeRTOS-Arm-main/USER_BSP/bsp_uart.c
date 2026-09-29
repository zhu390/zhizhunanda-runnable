#include "bsp_uart.h"
#include "bsp_usb_cdc.h"
#include "robot.h"
#include "Middle/W800_mqtt.h"
#include "jetson_vision.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

/* ============================================================ */
/*                        变量定义                              */
/* ============================================================ */

/* 1. LOG/CMD 串口  */
static char uart1_rx_buff[UART1_RX_BUFF_SIZE + SAFE_BUFF_SIZE] = {0};
static uint32_t uart1_rx_pos = 0;
static volatile bool g_uart1_tx_complete = true; // 发送完成标志

/* 2. MQTT 串口 */

/* 3. Jetson 视觉串口 */
/* 打印互斥锁，保证多任务打印不冲突 */
static SemaphoreHandle_t g_log_mutex = NULL;
/* 发送完成信号量，用于任务与中断同步 */
static SemaphoreHandle_t g_uart1_tx_sem = NULL;

/* ============================================================ */
/*                        初始化                                */
/* ============================================================ */

void BSP_UART_Init(void)
{
    fsp_err_t err;

    /* 初始化互斥锁 */
    if (g_log_mutex == NULL) {
        g_log_mutex = xSemaphoreCreateMutex();
    }

    /* 初始化发送完成信号量 */
    if (g_uart1_tx_sem == NULL) {
        g_uart1_tx_sem = xSemaphoreCreateBinary();
    }

    /* 1. 初始化 Debug/LOG 串口 */
    err = R_SCI_UART_Open(&robot_uart1_ctrl, &robot_uart1_cfg);
    if(FSP_SUCCESS != err) __BKPT(0);

    /* 2. 初始化 MQTT 串口 */
    err = R_SCI_UART_Open(&robot_mqtt_ctrl, &robot_mqtt_cfg);
    if(FSP_SUCCESS != err) __BKPT(0);
}

/* ============================================================ */
/*                      回调函数 (中断处理)                     */
/* ============================================================ */

/**
 * @brief Robot UART1 回调 (Log & Command)
 */
void robot_uart1_callback(uart_callback_args_t * p_args)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    switch (p_args->event)
    {
        case UART_EVENT_RX_CHAR:
        {
            char data = (char)p_args->data;

            // 检查是否是行尾结束符
            if ((data == '\r') || (data == '\n'))
            {
                // 只有当缓冲区里有内容时，才处理
                if (uart1_rx_pos > 0)
                {
                    // 将字符串结束，并发送到队列
                    uart1_rx_buff[uart1_rx_pos] = '\0';
                    robot_cmd_send_from_isr(uart1_rx_buff, CMD_TYPE_UART1);

                    // 彻底清空缓冲区指针，为下一次接收做准备
                    uart1_rx_pos = 0;
                }
                // 如果缓冲区是空的(比如收到了\r\n中的\n)，则忽略，不处理
            }
            else
            {
                // 正常字符，存入缓冲区
                if (uart1_rx_pos < (UART1_RX_BUFF_SIZE - 1))
                {
                    uart1_rx_buff[uart1_rx_pos++] = data;
                }
                else
                {
                    // 缓冲区溢出，丢弃所有内容并清零，防止错误
                    uart1_rx_pos = 0;
                }
            }
            break;
        }
        case UART_EVENT_TX_COMPLETE:
        {
            /* 根据调度器状态，选择不同的同步方式 */
            if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
            {
                /* 调度器已运行：使用信号量 */
                if (g_uart1_tx_sem != NULL) {
                    xSemaphoreGiveFromISR(g_uart1_tx_sem, &xHigherPriorityTaskWoken);
                    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
                }
            }
            else
            {
                /* 调度器未运行：使用标志位 */
                g_uart1_tx_complete = true;
            }
            break;
        }
        default: break;
    }
}

/**
 * @brief Robot MQTT 回调
 */
void robot_mqtt_callback(uart_callback_args_t * p_args)
{
    if (p_args->event == UART_EVENT_RX_CHAR)
    {
        // Directly forward the byte to the W800 MQTT driver's ring buffer
        w800_uart_byte_received((uint8_t)p_args->data);
    }
}



/**
 * @brief Robot Jetson 回调
 */
void robot_jetson_callback(uart_callback_args_t * p_args)
{
    switch (p_args->event)
    {
        case UART_EVENT_RX_CHAR:
            jetson_notify_rx_char_from_isr((uint8_t)p_args->data);
            break;

        case UART_EVENT_TX_COMPLETE:
            jetson_notify_tx_complete_from_isr();
            break;

        default:
            break;
    }
}

/* ============================================================ */
/*                      打印功能 (核心)                         */
/* ============================================================ */

/**
 * @brief 线程安全的 printf (任务级)
 */
void safe_printf(const char *format, ...)
{
    char buffer[256];
    va_list args;

    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    uint32_t len = (uint32_t)strlen(buffer);

    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
    {
        /* 任务运行中：检查锁是否已创建，防止初始化竞态导致的断言卡死 */
        if (g_log_mutex != NULL && g_uart1_tx_sem != NULL)
        {
            if (xSemaphoreTake(g_log_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
            {
                xSemaphoreTake(g_uart1_tx_sem, 0); 
                R_SCI_UART_Write(&robot_uart1_ctrl, (uint8_t *)buffer, len);
                xSemaphoreTake(g_uart1_tx_sem, pdMS_TO_TICKS(100));
                BSP_USB_CDC_WriteBestEffort(buffer, len);
                xSemaphoreGive(g_log_mutex);
            }
        }
        else
        {
            /* 锁还没准备好，回退到忙等模式发送，避免断言失败 */
            g_uart1_tx_complete = false;
            R_SCI_UART_Write(&robot_uart1_ctrl, (uint8_t *)buffer, len);
            uint32_t timeout = 0xFFFFF;
            while(!g_uart1_tx_complete && timeout--);
            BSP_USB_CDC_WriteBestEffort(buffer, len);
        }
    }

    else
    {
        /* 调度器未启动：使用旧的标志位 + 忙等待方式 */
        g_uart1_tx_complete = false;
        R_SCI_UART_Write(&robot_uart1_ctrl, (uint8_t *)buffer, len);
        uint32_t timeout = 0xFFFFF;
        while(!g_uart1_tx_complete && timeout--);
        BSP_USB_CDC_WriteBestEffort(buffer, len);
    }
}

/**
 * @brief 中断安全的 printf
 * 替代 STM32 的 safe_printf_from_isr
 * 注意：在瑞萨 SCI 中，如果在 ISR 里调用 Write 而此时 Task 正在 Write，可能会冲突。
 * 但为了 Crash Dump 等紧急情况，我们尝试直接发送。
 */
void safe_printf_from_isr(const char *format, ...)
{
    char buffer[128];
    va_list args;
    
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    /* 在中断里不能拿 Mutex，直接发 */
    /* 风险提示：如果此时串口正在忙，数据可能会乱，但这是 ISR 打印的代价 */
    R_SCI_UART_Write(&robot_uart1_ctrl, (uint8_t *)buffer, (uint32_t)strlen(buffer));
    
    /* 中断里不能死等 loop，发了就不管了 */
}
