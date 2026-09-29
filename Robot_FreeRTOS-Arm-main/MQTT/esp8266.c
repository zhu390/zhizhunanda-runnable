#include "hal_data.h"
#include "esp8266.h"
#include "bsp_uart.h"

#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <stddef.h>

#define ESP8266_RST_PIN                 BSP_IO_PORT_05_PIN_00    // P500 复位脚，高电平正常

#define ESP8266_WIFI_INFO               "AT+CWJAP=\"" WIFI_SSID "\",\"" WIFI_PSWD "\"\r\n"
#define ESP8266_ONENET_INFO             "AT+CIPSTART=\"TCP\",\"" SERVER_HOST "\"," SERVER_PORT "\r\n"

static uint8_t  esp8266_buf[ESP_RX_MAX];
static uint16_t esp8266_cnt = 0, esp8266_cntPre = 0;

uint8_t ESP8266_INIT_OK = 0;

/* UART 实例：使用 FSP 生成的 esp8266 串口实例 */
extern const uart_instance_t esp8266;

static volatile bool s_tx_done = false;

static inline void esp_delay_ms(uint32_t ms)
{
    (void) R_BSP_SoftwareDelay(ms, BSP_DELAY_UNITS_MILLISECONDS);
}

/* 简单阻塞发送：启动写入并等待 TEI 回调置位 */
static void esp_uart_send(const uint8_t *data, uint32_t len)
{
    if ((NULL == data) || (len == 0u)) {
        return;
    }

    s_tx_done = false;
    (void) esp8266.p_api->write(esp8266.p_ctrl, data, len);

    /* 等待发送完成 */
    while (!s_tx_done) {
        (void) R_BSP_SoftwareDelay(1, BSP_DELAY_UNITS_MICROSECONDS);
    }
}

//==========================================================
//	函数名称：	ESP8266_Clear
//
//	函数功能：	清空缓存
//
//	入口参数：	无
//
//	返回参数：	无
//
//	说明：		
//==========================================================
void ESP8266_Clear(void)
{
    if (!ESP8266_ENABLE) {
        return;
    }
    memset(esp8266_buf, 0, sizeof(esp8266_buf));
    esp8266_cnt    = 0;
    esp8266_cntPre = 0;
}

//==========================================================
//	函数名称：	ESP8266_WaitRecive
//
//	函数功能：	等待接收完成
//
//	入口参数：	无
//
//	返回参数：	REV_OK-接收完成		REV_WAIT-接收超时未完成
//
//	说明：		循环调用检测是否接收完成
//==========================================================
_Bool ESP8266_WaitRecive(void)
{
    if (!ESP8266_ENABLE) {
        return REV_WAIT;
    }
    if (esp8266_cnt == 0) {
        return REV_WAIT;
    }

    if (esp8266_cnt == esp8266_cntPre) {
        esp8266_cnt = 0;
        return REV_OK;
    }

    esp8266_cntPre = esp8266_cnt;
    return REV_WAIT;
}

//==========================================================
//	函数名称：	ESP8266_SendCmd
//
//	函数功能：	发送命令
//
//	入口参数：	cmd：命令
//				res：需要检查的返回指令
//
//	返回参数：	0-成功	1-失败
//
//	说明：		
//==========================================================
_Bool ESP8266_SendCmd(char *cmd, char *res)
{
    if (!ESP8266_ENABLE) {
        return 1;
    }
    unsigned char timeOut = 200;

    esp_uart_send((uint8_t *)cmd, strlen((const char *)cmd));

    while (timeOut--)
    {
        if (ESP8266_WaitRecive() == REV_OK)
        {
            if (strstr((const char *)esp8266_buf, res) != NULL)
            {
                ESP8266_Clear();
                return 0;
            }
        }

        esp_delay_ms(10);
    }

    return 1;
}

//==========================================================
//	函数名称：	ESP8266_SendData
//
//	函数功能：	发送数据
//
//	入口参数：	data：数据
//				len：长度
//
//	返回参数：	无
//
//	说明：		
//==========================================================
void ESP8266_SendData(unsigned char *data, unsigned short len)
{
    char cmdBuf[32];

    if (!ESP8266_ENABLE) {
        return;
    }
    ESP8266_Clear();
    sprintf(cmdBuf, "AT+CIPSEND=%d\r\n", len);
    if (!ESP8266_SendCmd(cmdBuf, ">"))
    {
        esp_uart_send(data, len);
    }
}

//==========================================================
//	函数名称：	ESP8266_GetIPD
//
//	函数功能：	获取平台返回的数据
//
//	入口参数：	等待的时间(乘以10ms)
//
//	返回参数：	平台返回的原始数据
//
//	说明：		不同网络设备返回的格式不同，需要去调试
//				如ESP8266的返回格式为	"+IPD,x:yyy"	x代表数据长度，yyy是数据内容
//==========================================================
unsigned char *ESP8266_GetIPD(unsigned short timeOut)
{
    char *ptrIPD = NULL;

    if (!ESP8266_ENABLE) {
        return NULL;
    }

    do
    {
        if (ESP8266_WaitRecive() == REV_OK)
        {
            ptrIPD = strstr((char *)esp8266_buf, "IPD,");
            if (ptrIPD == NULL)
            {
                // LOG("\"IPD\" not found\r\n");
            }
            else
            {
                ptrIPD = strchr(ptrIPD, ':');
                if (ptrIPD != NULL)
                {
                    ptrIPD++;
                    return (unsigned char *)(ptrIPD);
                }
                else
                {
                    return NULL;
                }
            }
        }
        esp_delay_ms(5);
        timeOut--;
    } while (timeOut > 0);

    return NULL;
}

//==========================================================
//	函数名称：	ESP8266_Init
//
//	函数功能：	初始化ESP8266
//
//	入口参数：	无
//
//	返回参数：	无
//
//	说明：		
//==========================================================
void ESP8266_Init(void)
{
    if (!ESP8266_ENABLE) {
        ESP8266_INIT_OK = 0;
        return;
    }
    /* 打开 UART */
    if (FSP_SUCCESS != esp8266.p_api->open(esp8266.p_ctrl, esp8266.p_cfg))
    {
        LOG("ESP8266 UART open failed\r\n");
        return;
    }

    /* 复位脚 P500：低 250ms 再拉高 500ms */
    (void) R_IOPORT_PinWrite(&g_ioport_ctrl, ESP8266_RST_PIN, BSP_IO_LEVEL_LOW);
    esp_delay_ms(250);
    (void) R_IOPORT_PinWrite(&g_ioport_ctrl, ESP8266_RST_PIN, BSP_IO_LEVEL_HIGH);
    esp_delay_ms(500);

    ESP8266_Clear();

    LOG("0. AT - test MCU-8266 link");
    while (ESP8266_SendCmd("AT\r\n", "OK"))
    {
        esp_delay_ms(500);
    }

    LOG("1. AT+RST - soft reset 8266");
    ESP8266_SendCmd("AT+RST\r\n", "");
    esp_delay_ms(500);
    ESP8266_SendCmd("AT+CIPCLOSE\r\n", "");
    esp_delay_ms(500);

    LOG("2. AT+CWMODE=1,1 - set mode STA");
    while (ESP8266_SendCmd("AT+CWMODE=1\r\n", "OK"))
    {
        esp_delay_ms(500);
    }

    LOG("3. AT+CWDHCP=1,1 - enable DHCP in STA");
    while (ESP8266_SendCmd("AT+CWDHCP=1,1\r\n", "OK"))
    {
        esp_delay_ms(500);
    }

    LOG("4. AT+CWJAP - connect WIFI -> [ SSID: %s ]  -> [ Password: %s ] ", WIFI_SSID, WIFI_PSWD);
    while (ESP8266_SendCmd(ESP8266_WIFI_INFO, "GOT IP"))
    {
        esp_delay_ms(500);
    }

    LOG("5. AT+CIPSTART - connect narou -> [ %s:%s ]", SERVER_HOST, SERVER_PORT);
    while (ESP8266_SendCmd(ESP8266_ONENET_INFO, "CONNECT"))
    {
        esp_delay_ms(500);
    }

    ESP8266_INIT_OK = 1;
    LOG("6. ESP8266 Init OK - ESP8266-Init-success!!!");
    LOG("ESP8266--InIt            [OK]");
}

/* UART 回调：处理接收和发送完成事件 */
void esp8266_uart_callback(uart_callback_args_t * p_args)
{
    if (!ESP8266_ENABLE) {
        return;
    }
    if (p_args == NULL) {
        return;
    }

    if (UART_EVENT_RX_CHAR == p_args->event)
    {
        if (esp8266_cnt >= sizeof(esp8266_buf))
        {
            esp8266_cnt = 0; // 防止被刷爆
        }
        esp8266_buf[esp8266_cnt++] = (uint8_t) p_args->data;
    }
    else if (UART_EVENT_TX_COMPLETE == p_args->event)
    {
        s_tx_done = true;
    }
}
