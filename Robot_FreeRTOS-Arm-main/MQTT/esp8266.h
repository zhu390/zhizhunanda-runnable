#ifndef _ESP8266_H_
#define _ESP8266_H_

#include <stdint.h>
#include <stdbool.h>

#ifndef ESP8266_ENABLE
#define ESP8266_ENABLE  1
#endif

#define REV_OK		0	//接收完成标志
#define REV_WAIT	1	//接收未完成标志

//#define WIFI_SSID                       "HUAWEI-R1CTR7"       // WIFI 名称（2.4G）
//#define WIFI_PSWD                       "dianxue409"          // WIFI 密码

#define WIFI_SSID                       "OPPOFindX8"       // WIFI 名称（2.4G）
#define WIFI_PSWD                       "Ww12345678"          // WIFI 密码

#define SERVER_HOST                     "broker.emqx.io"
#define SERVER_PORT                     "1883"


#define ESP_RX_MAX	1024//接收最大缓存

void ESP8266_Init(void);

void ESP8266_Clear(void);

void ESP8266_SendData(uint8_t *data, uint16_t len);

uint8_t *ESP8266_GetIPD(uint16_t timeOut);

#endif
