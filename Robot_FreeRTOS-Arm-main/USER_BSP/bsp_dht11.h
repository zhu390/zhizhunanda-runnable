#ifndef __DHT11_H
#define __DHT11_H

#include "hal_data.h"
#include <stdint.h>

/* 选择接 DHT11 DATA 的引脚，这里默认 P0_02。若需更换，改成对应 BSP_IO_PORT_xx_PIN_xx。 */
#ifndef DHT11_PIN
#define DHT11_PIN   BSP_IO_PORT_00_PIN_02
#endif

/* 方向切换：输出用于拉低起始信号，输入用于采样（可结合外部上拉）。 */
static inline void DHT11_IO_OUT(void)
{
    R_IOPORT_PinCfg(g_ioport.p_ctrl, DHT11_PIN, IOPORT_CFG_PORT_DIRECTION_OUTPUT);
}

static inline void DHT11_IO_IN_PULLUP(void)
{
    R_IOPORT_PinCfg(g_ioport.p_ctrl, DHT11_PIN,
                    IOPORT_CFG_PORT_DIRECTION_INPUT | IOPORT_CFG_PULLUP_ENABLE);
}

/* IO 操作 */
static inline void DHT11_DQ_LOW(void)
{
    R_IOPORT_PinWrite(g_ioport.p_ctrl, DHT11_PIN, BSP_IO_LEVEL_LOW);
}

static inline void DHT11_DQ_HIGH(void)
{
    R_IOPORT_PinWrite(g_ioport.p_ctrl, DHT11_PIN, BSP_IO_LEVEL_HIGH);
}

static inline uint8_t DHT11_DQ_READ(void)
{
    bsp_io_level_t level = BSP_IO_LEVEL_LOW;
    R_IOPORT_PinRead(g_ioport.p_ctrl, DHT11_PIN, &level);
    return (uint8_t)(level == BSP_IO_LEVEL_HIGH);
}

/* API */
uint8_t DHT11_Init(void); /* 初始化并检测设备 */
uint8_t DHT11_Read_Data(uint8_t *humiH,uint8_t *humiL,uint8_t *tempH,uint8_t *tempL);
uint8_t DHT11_Read_Byte(void);
uint8_t DHT11_Read_Bit(void);
uint8_t DHT11_Check(void);
void    DHT11_Rst(void);

#endif

