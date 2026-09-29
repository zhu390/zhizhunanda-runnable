#include "bsp_dht11.h"
#include "hal_data.h"   /* R_BSP_SoftwareDelay, g_ioport */
#include <stddef.h>
#include <stdbool.h>

/* 声明延时函数（部分编译环境未自动提供原型时使用） */

#define DHT11_ACK_WAIT_TIMEOUT_US      (200u)
#define DHT11_BIT_EDGE_TIMEOUT_US      (120u)
#define DHT11_INIT_RETRY_COUNT         (5u)
#define DHT11_INIT_RETRY_INTERVAL_MS   (120u)
#define DHT11_FIRST_POWERUP_DELAY_MS   (1000u)

static bool s_dht_powerup_wait_done = false;
static bool s_dht_last_bit_timeout = false;

static inline void dht_delay_ms(uint32_t ms)
{
    R_BSP_SoftwareDelay(ms, BSP_DELAY_UNITS_MILLISECONDS);
}

static inline void dht_delay_us(uint32_t us)
{
    R_BSP_SoftwareDelay(us, BSP_DELAY_UNITS_MICROSECONDS);
}

static bool dht_wait_level(uint8_t expect_high, uint32_t timeout_us)
{
    while (timeout_us > 0u)
    {
        uint8_t level = DHT11_DQ_READ() ? 1u : 0u;
        if (level == expect_high) {
            return true;
        }
        dht_delay_us(1);
        timeout_us--;
    }
    return false;
}

/* 复位 DHT11 */
void DHT11_Rst(void)
{
    DHT11_IO_OUT();      /* 输出模式 */
    DHT11_DQ_LOW();      /* 拉低 DQ */
    dht_delay_ms(20);    /* 至少 18 ms */
    DHT11_DQ_HIGH();     /* 释放 */
    dht_delay_us(30);    /* 主机拉高 20~40 us */
}


/* 等待 DHT11 回应
 * 返回 0: 存在；1: 未检测到
 */
uint8_t DHT11_Check(void)
{
    DHT11_IO_IN_PULLUP();
    dht_delay_us(5);

    if (!dht_wait_level(0u, DHT11_ACK_WAIT_TIMEOUT_US)) {
        return 1;
    }

    if (!dht_wait_level(1u, DHT11_ACK_WAIT_TIMEOUT_US)) {
        return 1;
    }

    return 0;
}

/* 读 1 bit */
uint8_t DHT11_Read_Bit(void)
{
    if (!dht_wait_level(0u, DHT11_BIT_EDGE_TIMEOUT_US)) {
        s_dht_last_bit_timeout = true;
        return 0u;
    }

    if (!dht_wait_level(1u, DHT11_BIT_EDGE_TIMEOUT_US)) {
        s_dht_last_bit_timeout = true;
        return 0u;
    }

    dht_delay_us(35);
    return DHT11_DQ_READ() ? 1u : 0u;
}

/* 读 1 字节 */
uint8_t DHT11_Read_Byte(void)
{
    uint8_t i, dat = 0;
    for (i = 0; i < 8; i++) {
        dat <<= 1;
        dat |= DHT11_Read_Bit();
    }
    return dat;
}

/* 读一次数据，返回 0 正常，1 失败 */
uint8_t DHT11_Read_Data(uint8_t *humiH,uint8_t *humiL,uint8_t *tempH,uint8_t *tempL)
{
    uint8_t buf[5] = {0};
    uint8_t i;
    uint8_t attempt;

    if ((humiH == NULL) || (humiL == NULL) || (tempH == NULL) || (tempL == NULL)) {
        return 1;
    }

    for (attempt = 0; attempt < 2u; attempt++)
    {
        s_dht_last_bit_timeout = false;

        DHT11_Rst();
        if (DHT11_Check() == 0)
        {
            for (i = 0; i < 5; i++) {
                buf[i] = DHT11_Read_Byte();
            }

            if ((!s_dht_last_bit_timeout) && ((uint8_t)(buf[0] + buf[1] + buf[2] + buf[3]) == buf[4]))
            {
                *humiH = buf[0];
                *humiL = buf[1];
                *tempH = buf[2];
                *tempL = buf[3];
                return 0;
            }
        }

        dht_delay_ms(30);
    }

    return 1;
}

/* 初始化 IO 并检测设备，返回 0 存在，1 不存在 */
uint8_t DHT11_Init(void)
{
    uint8_t i;

    /* 初始设为输出高，随后按协议复位检测 */
    DHT11_IO_OUT();
    DHT11_DQ_HIGH();

    /* DHT11 上电后需要稳定时间，避免首次/复位后偶发失败 */
    if (!s_dht_powerup_wait_done)
    {
        dht_delay_ms(DHT11_FIRST_POWERUP_DELAY_MS);
        s_dht_powerup_wait_done = true;
    }

    for (i = 0; i < DHT11_INIT_RETRY_COUNT; i++)
    {
        DHT11_Rst();
        if (DHT11_Check() == 0) {
            return 0;
        }
        dht_delay_ms(DHT11_INIT_RETRY_INTERVAL_MS);
    }

    return 1;
}
