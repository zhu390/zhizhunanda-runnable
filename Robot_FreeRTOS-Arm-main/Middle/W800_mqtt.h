#ifndef __W800_MQTT_H__
#define __W800_MQTT_H__

#include "hal_data.h"
#include <string.h>
#include <stdio.h>
#include <stdint.h>

// --- WiFi and MQTT Credentials ---
#define WIFI_SSID       "HUAWEI-R1CTR7"
#define WIFI_PASSWORD   "dianxie409"

// IMPORTANT: Replace with your actual MQTT broker IP address
#define MQTT_SERVER     "10.193.207.112"       
#define MQTT_PORT       1883
#define MQTT_CLIENT_ID  "mqttx_d255f440"
#define MQTT_TOPIC      "robort"
#define MQTT_USERNAME   "pzbc"
#define MQTT_PASSWORD   "Ww1314520"

// --- Public Functions ---

/**
 * @brief Main initialization function. Creates the MQTT background service task.
 */
int w800_mqtt_service_init(void);

/**
 * @brief Publishes a message to a topic. Non-blocking.
 * This function is thread-safe and can be called from any task.
 */
int w800_publish_message(const char *topic, const char *message, uint8_t qos);


// --- Internal Functions (called by BSP layer) ---

/**
 * @brief Callback to be placed in the UART ISR (e.g., in bsp_uart.c).
 * This function is the entry point for all data coming from the W800 module.
 * @param data The single byte received from the UART peripheral.
 */
void w800_uart_byte_received(uint8_t data);

#endif
