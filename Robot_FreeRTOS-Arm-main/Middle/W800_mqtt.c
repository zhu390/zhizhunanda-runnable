/**
 * @file W800_mqtt.c
 * @brief W800 (Generic AT Socket) MQTT driver using libemqtt and a ring buffer.
 *        Implements a robust, byte-stream based parser for AT commands and MQTT data.
 * @author Gemini (with guidance from the Master Teacher)
 */
 
#include "W800_mqtt.h"
#include "robot.h" // For LOG / ROBOT_MQTT_ENABLE / robot_cmd_send

/*
 * IMPORTANT:
 * If MQTT is disabled, do NOT create the W800 MQTT task and do NOT send any AT commands.
 * This avoids spamming the serial monitor when `ROBOT_MQTT_ENABLE` is set to 0.
 */
#if !defined(ROBOT_MQTT_ENABLE) || (ROBOT_MQTT_ENABLE != 1)

void w800_uart_byte_received(uint8_t data)
{
    (void)data;
}

int w800_mqtt_service_init(void)
{
    return 0;
}

int w800_publish_message(const char *topic, const char *message, uint8_t qos)
{
    (void)topic;
    (void)message;
    (void)qos;
    return 0;
}

#else

#include "bsp_uart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "libemqtt.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

/* --- Ring Buffer Implementation --- */
#define RX_BUF_SIZE 2048
typedef struct {
    uint8_t *buffer;
    uint32_t size;
    volatile uint32_t head;
    volatile uint32_t tail;
} RingBuffer_t;

static uint8_t rx_mem[RX_BUF_SIZE];
static RingBuffer_t g_rx_rb = { .buffer = rx_mem, .size = RX_BUF_SIZE, .head = 0, .tail = 0 };

void w800_uart_byte_received(uint8_t data)
{
    uint32_t next = (g_rx_rb.head + 1) % g_rx_rb.size;
    if (next != g_rx_rb.tail) { // Prevent overflow
        g_rx_rb.buffer[g_rx_rb.head] = data;
        g_rx_rb.head = next;
    }
}

int rb_read_byte(uint8_t *byte, uint32_t timeout_ms)
{
    TickType_t start = xTaskGetTickCount();
    while ((xTaskGetTickCount() - start) < pdMS_TO_TICKS(timeout_ms))
    {
        if (g_rx_rb.head != g_rx_rb.tail) {
            *byte = g_rx_rb.buffer[g_rx_rb.tail];
            g_rx_rb.tail = (g_rx_rb.tail + 1) % g_rx_rb.size;
            return 1; // Success
        }
        vTaskDelay(pdMS_TO_TICKS(1)); // Wait for data
    }
    return 0; // Timeout
}

int rb_read_bytes(uint8_t *buf, int len, int timeout_ms)
{
    for (int i = 0; i < len; i++) {
        if (!rb_read_byte(&buf[i], timeout_ms)) {
            return 0; // Timeout
        }
    }
    return 1; // Success
}

static void rb_flush(void)
{
    /* Drop any pending bytes (boot logs/noise). */
    g_rx_rb.tail = g_rx_rb.head;
}

/* --- Core Driver Logic --- */
static mqtt_broker_handle_t g_broker;
static int g_mqtt_connected = 0;
static int g_socket_id = -1; // socket id returned by AT+SKCT (+OK=<id>)

void w800_send_at_command(const char *command)
{
    char full_cmd[128];
    snprintf(full_cmd, sizeof(full_cmd), "%s\r\n", command);
    R_SCI_UART_Write(&robot_mqtt_ctrl, (uint8_t *)full_cmd, (uint32_t)strlen(full_cmd));
    vTaskDelay(pdMS_TO_TICKS(10));
}

int w800_read_line(char *buf, int max_len, int timeout_ms)
{
    int idx = 0;
    uint8_t c;

    // Clear buffer first
    memset(buf, 0, max_len);

    while(idx < max_len -1)
    {
        if (!rb_read_byte(&c, timeout_ms)) {
             return 0; // Overall timeout
        }

        if (c == '\n') {
            if (idx > 0 && buf[idx-1] == '\r') {
               buf[idx-1] = '\0'; // Overwrite CR
            } else {
               buf[idx] = '\0';
            }
            // Trim leading whitespace
            size_t len = strlen(buf);
            size_t start_index = 0;
            while(start_index < len && isspace((unsigned char)buf[start_index])) {
                start_index++;
            }
            if (start_index > 0) {
                memmove(buf, buf + start_index, len - start_index + 1);
            }
            return 1; // Got a line
        }
        buf[idx++] = (char)c;
    }
    buf[max_len-1] = '\0';
    return 0; // Buffer full but no newline
}


int w800_expect_responses(const char* const responses[], int num_responses, int timeout_ms)
{
    char line_buf[128];
    TickType_t start = xTaskGetTickCount();
    while ((xTaskGetTickCount() - start) < pdMS_TO_TICKS(timeout_ms))
    {
        if (w800_read_line(line_buf, sizeof(line_buf), timeout_ms))
        {
            if (strlen(line_buf) > 0) {
                LOG("W800: <<< %s\n", line_buf);
            }
            for (int i = 0; i < num_responses; i++) {
                 if (strlen(line_buf) > 0 && strstr(line_buf, responses[i]) != NULL) {
                    return i; // Return the index of the matched response
                }
            }
        }
    }
    return -1; // Timeout
}

static int w800_wait_ok_err(int timeout_ms)
{
    const char* rsp[] = {"+OK", "OK", "ERROR", "+ERR"};
    int r = w800_expect_responses(rsp, 4, timeout_ms);
    return (r == 0 || r == 1) ? 0 : -1;
}

static int w800_skct_connect_try(const char *fmt,
                                 const char *host,
                                 int remote_port,
                                 int local_port,
                                 int *out_socket,
                                 int *out_err_code)
{
    char cmd[128];
    char line[128];
    TickType_t start;

    if (out_err_code) {
        *out_err_code = 0;
    }

    snprintf(cmd, sizeof(cmd), fmt, host, remote_port, local_port);
    LOG("W800: >>> %s\n", cmd);
    w800_send_at_command(cmd);

    start = xTaskGetTickCount();
    while ((xTaskGetTickCount() - start) < pdMS_TO_TICKS(10000))
    {
        if (!w800_read_line(line, (int)sizeof(line), 20)) {
            continue;
        }
        if (strlen(line) > 0) {
            LOG("W800: <<< %s\n", line);
        }

        /* Success patterns */
        {
            char *p = strstr(line, "+OK=");
            if (p) {
                int sid = -1;
                if (sscanf(p + 4, "%d", &sid) == 1 && sid >= 0) {
                    *out_socket = sid;
                    return 0;
                }
            }
            if (strstr(line, "OK") || strstr(line, "+OK")) {
                /* Some firmware may only return OK without socket id. */
                *out_socket = 0;
                return 0;
            }
        }

        /* Error patterns */
        {
            char *e = strstr(line, "+ERR=");
            if (e) {
                int code = 0;
                if (sscanf(e + 5, "%d", &code) == 1) {
                    if (out_err_code) {
                        *out_err_code = code;
                    }
                    LOG("W800: SKCT failed with +ERR=%d\n", code);
                }
                return -1;
            }
            if (strstr(line, "ERROR")) {
                if (out_err_code) {
                    *out_err_code = 1;
                }
                LOG("W800: SKCT failed with ERROR\n");
                return -1;
            }
        }
    }
    return -2; /* Timeout */
}

static int w800_skct_connect(const char *host, int remote_port, int *out_socket)
{
    /*
     * Observed firmware behavior:
     * - Some firmwares reject local_port=0 with +ERR=-4.
     * - Some accept 5 args, some accept only 4 args.
     *
     * So we try: non-zero local port first, then fallback variants.
     */
    const char *fmts[] = {
        "AT+SKCT=0,0,\"%s\",%d,%d",
        "AT+SKCT=0,0,%s,%d,%d",
        /* 4-arg variant (no local port). */
        "AT+SKCT=0,0,\"%s\",%d",
        "AT+SKCT=0,0,%s,%d",
    };

    /* Try non-zero ports first; keep 0 as last fallback for other firmwares. */
    const int local_ports[] = {10000, 20000, 0};

    for (size_t p = 0; p < (sizeof(local_ports) / sizeof(local_ports[0])); p++)
    {
        for (size_t i = 0; i < (sizeof(fmts) / sizeof(fmts[0])); i++)
        {
            int sid = -1;
            int err_code = 0;
            int ok;

            ok = w800_skct_connect_try(fmts[i], host, remote_port, local_ports[p], &sid, &err_code);

            if (ok == 0) {
                *out_socket = sid;
                return 0;
            }

            /* If this firmware rejects local_port=0, don't waste more tries with 0. */
            if (local_ports[p] == 0 && err_code == -4) {
                break;
            }

            vTaskDelay(pdMS_TO_TICKS(200));
        }
    }

    return -1;
}

int w800_at_socket_send(int socket_info, const void* buf, unsigned int count)
{
    char command[64];
    snprintf(command, sizeof(command), "AT+SKSND=%d,%u", socket_info, count);

    // 1. Send command header
    w800_send_at_command(command);

    // 2. Wait for the ">" prompt (some firmware does not end it with \r\n)
    TickType_t start = xTaskGetTickCount();
    uint8_t ch = 0;
    int got_prompt = 0;
    int err_state = 0;
    const char *err_pat = "ERROR";
    char dbg_buf[64];
    int dbg_len = 0;

    while ((xTaskGetTickCount() - start) < pdMS_TO_TICKS(2000))
    {
        if (rb_read_byte(&ch, 10))
        {
            if (dbg_len < (int)sizeof(dbg_buf) - 1) {
                dbg_buf[dbg_len++] = (char)((ch >= 32 && ch <= 126) ? ch : '.');
                dbg_buf[dbg_len] = 0;
            }

            if (ch == '>') { got_prompt = 1; break; }

            if (ch == (uint8_t)err_pat[err_state]) {
                err_state++;
                if (err_state == 5) {
                    LOG("W800: SKSND got ERROR (rx='%s')\n", dbg_buf);
                    return -1;
                }
            } else {
                err_state = (ch == (uint8_t)err_pat[0]) ? 1 : 0;
            }
        }
    }

    if (!got_prompt)
    {
        LOG("W800: SKSND prompt not received (rx='%s')\n", dbg_buf);
        return -1;
    }

    // 3. Send the binary data
    R_SCI_UART_Write(&robot_mqtt_ctrl, (uint8_t *)buf, count);

    // 4. Wait for send success confirmation
    const char* ok_responses[] = {"OK", "+OK"};
    if (w800_expect_responses(ok_responses, 2, 5000) != 0) {
        LOG("W800: SKSND data send confirmation failed!\n");
        return -1;
    }
    return count;
}

static void parse_and_process_packet(uint8_t* packet, int len)
{
    (void)len; // Explicitly mark 'len' as unused to suppress compiler warnings.
    uint8_t msg_type = MQTTParseMessageType(packet);

    switch(msg_type)
    {
        case MQTT_MSG_CONNACK:
            if (packet[3] == 0) { // Check return code
                g_mqtt_connected = 1;
                LOG("W800: MQTT CONNECTED! Received CONNACK.\n");
            }
            break;

        case MQTT_MSG_SUBACK:
            LOG("W800: Subscribed to topic!\n");
            break;

        case MQTT_MSG_PUBLISH:
        {
            const uint8_t *topic_ptr, *msg_ptr;
            uint16_t topic_len = mqtt_parse_pub_topic_ptr(packet, &topic_ptr);
            uint16_t msg_len = mqtt_parse_pub_msg_ptr(packet, &msg_ptr);

            if (topic_len > 0 && msg_len > 0) {
                // NOTE: The pointers are NOT null-terminated strings!
                // We need to copy them to a buffer to treat as strings.
                                static char topic_buf[128];
                static char msg_buf[MQTT_RX_BUFF_SIZE];
                
                memcpy(topic_buf, topic_ptr, topic_len < sizeof(topic_buf)-1 ? topic_len : sizeof(topic_buf)-1);
                topic_buf[topic_len] = 0;

                memcpy(msg_buf, msg_ptr, msg_len < sizeof(msg_buf)-1 ? msg_len : sizeof(msg_buf)-1);
                msg_buf[msg_len] = 0;

                LOG("W800: RX on [%s]: %s\n", topic_buf, msg_buf);
                robot_cmd_send(msg_buf, CMD_TYPE_MQTT);
            }
            break;
        }

        case MQTT_MSG_PINGRESP:
            LOG("W800: PINGRESP received.\n");
            break;

        default:
            break;
    }
}

void w800_main_task(void *pvParameters)
{
    (void)pvParameters;

    // --- Outer loop for auto-reconnection ---
    while(1)
    {
        char command[128];
        g_mqtt_connected = 0; // Reset connection state

        // --- Hardware Reset for W800 Module ---
        // TODO: Please replace with the actual GPIO pin connected to W800's RST pin.
        #define W800_RST_PIN BSP_IO_PORT_05_PIN_08  // <<< IMPORTANT: Replace with the correct pin from your schematic!

        R_BSP_PinWrite(W800_RST_PIN, BSP_IO_LEVEL_LOW);
        vTaskDelay(pdMS_TO_TICKS(200));
        R_BSP_PinWrite(W800_RST_PIN, BSP_IO_LEVEL_HIGH);
        vTaskDelay(pdMS_TO_TICKS(1500)); // Wait for the module to boot up

        rb_flush();
        w800_send_at_command("AT");
        const char* at_rsp[] = {"OK", "+OK"};
        (void) w800_expect_responses(at_rsp, 2, 1500);

        LOG("W800: (Re)Starting Sequence...\n");

        /*
         * Note for competition: For maximum stability, consider adding a hardware
         * reset for the W800 module here via a GPIO pin before starting the sequence.
        */

    // --- Stage 1: Connect to WiFi ---
    LOG("W800: Setting STA mode...\n");
    w800_send_at_command("AT+WPRT=0");
    if (w800_wait_ok_err(2000) != 0) { LOG("W800: Failed to set STA mode.\n"); vTaskDelay(pdMS_TO_TICKS(3000)); continue; }

    LOG("W800: Setting SSID...\n");
    snprintf(command, sizeof(command), "AT+SSID=\"%s\"", WIFI_SSID);
    w800_send_at_command(command);
    if (w800_wait_ok_err(2000) != 0) { LOG("W800: Failed to set SSID.\n"); vTaskDelay(pdMS_TO_TICKS(3000)); continue; }

    LOG("W800: Setting password...\n");
    snprintf(command, sizeof(command), "AT+KEY=1,0,\"%s\"", WIFI_PASSWORD);
    w800_send_at_command(command);
    if (w800_wait_ok_err(2000) != 0) { LOG("W800: Failed to set password.\n"); vTaskDelay(pdMS_TO_TICKS(3000)); continue; }

    LOG("W800: Joining AP...\n");
    w800_send_at_command("AT+WJOIN");
    if (w800_wait_ok_err(20000) != 0) { LOG("W800: Failed to join AP.\n"); vTaskDelay(pdMS_TO_TICKS(3000)); continue; }
    LOG("W800: WiFi Connected!\n");

    // --- Stage 2: Connect to MQTT Broker ---
    LOG("W800: Creating TCP socket...\n");
    g_socket_id = -1;
    if (w800_skct_connect(MQTT_SERVER, MQTT_PORT, &g_socket_id) != 0) {
        LOG("W800: Failed to create socket.\n");
        vTaskDelay(pdMS_TO_TICKS(3000));
        continue;
    }
    LOG("W800: TCP Socket created. socket=%d\n", g_socket_id);
    vTaskDelay(pdMS_TO_TICKS(200)); // Add a small delay for the module to settle

    
    LOG("W800: Enabling active data reporting...\n");
    {
        /* Some W800 firmwares use different parameter styles for SKRPTM. */
        const char *skrptm_cmds[] = {"AT+SKRPTM=1", "AT+SKRPTM=1,0", "AT+SKRPTM=1,1"};
        int skrptm_ok = 0;
        for (size_t i = 0; i < (sizeof(skrptm_cmds) / sizeof(skrptm_cmds[0])); i++)
        {
            w800_send_at_command(skrptm_cmds[i]);
            if (w800_wait_ok_err(2000) == 0) { skrptm_ok = 1; break; }
        }
        if (!skrptm_ok) { LOG("W800: Failed to set SKRPTM.\n"); vTaskDelay(pdMS_TO_TICKS(3000)); continue; }
    }

    // Init libemqtt
    mqtt_init(&g_broker, MQTT_CLIENT_ID);
    mqtt_init_auth(&g_broker, MQTT_USERNAME, MQTT_PASSWORD);
    g_broker.mqttsend = w800_at_socket_send;
    g_broker.socketid = g_socket_id;

    LOG("W800: Sending MQTT CONNECT packet...\n");
    if (mqtt_connect(&g_broker) < 0) { LOG("W800: mqtt_connect() failed.\n"); vTaskDelay(pdMS_TO_TICKS(3000)); continue; }

    // --- Stage 3: The Main Loop (Parser & Ping) ---
    TickType_t last_ping = xTaskGetTickCount();
        static uint8_t packet_buf[512];

    while(1)
    {
        uint8_t c;
        // 1. 逐字节扫描 RingBuffer，寻找 "+SKRPTM:" 的特征
        // 这是一个简化版的高效扫描，不需要把数据读出来放到 line_buf
        // 只要 RingBuffer 里有数据，我们就 peek (或者 read) 看看是不是头

        if (rb_read_byte(&c, 10)) // 短超时读取
        {
            // 简单的状态机检测 "+SKRPTM:"
            // 既然我们在 RTOS 任务里，用临时缓冲匹配法最简单直观
            static int match_state = 0;
            const char *header = "+SKRPTM:";

            if (c == header[match_state]) {
                match_state++;
                if (match_state == (int)strlen(header)) {
                    // --- 匹配到了头！关键时刻 ---
                    match_state = 0; // 重置状态

                    // 现在的格式是 "+SKRPTM:" 后面紧接着 "socket,len:"
                    // 我们需要继续读，直到读到冒号 ':'，解析出长度

                    char param_buf[32];
                    int p_idx = 0;
                    int socket = 0, len = 0;

                    // 循环读取参数，直到遇到 ':' 或换行(兼容性)
                    while(p_idx < 31) {
                        uint8_t pc;
                        if(rb_read_byte(&pc, 100)) {
                            if(pc == ':' || pc == '\r' || pc == '\n') {
                                param_buf[p_idx] = 0;
                                // 如果是冒号，说明参数结束，后面紧跟数据
                                if(pc == ':') break;
                            } else {
                                param_buf[p_idx++] = (char)pc;
                            }
                        } else {
                            break; // 超时
                        }
                    }

                    // 解析 "0,5" 这种格式
                    if (sscanf(param_buf, "%d,%d", &socket, &len) == 2) {
                        if (len > 0 && len < sizeof(packet_buf)) {
                            // --- 核心修正 ---
                            // 此时，RingBuffer 的指针正好指向数据的第一个字节
                            // 直接读取 len 长度的数据，不会错位！
                            if (rb_read_bytes(packet_buf, len, 1000)) {
                                parse_and_process_packet(packet_buf, len);
                            }
                        }
                    }
                }
            } else {
                // 如果这一字节不匹配，回退状态机
                // 严谨的做法是 KMP 算法，但这里简单处理：
                // 如果遇到 '+', 尝试作为新的开始，否则归零
                if (c == '+') match_state = 1;
                else match_state = 0;
            }
        }

        // If connected, subscribe if not already (logic inside parse_and_process_packet handles g_mqtt_connected flag)
        if (g_mqtt_connected == 1) {
             uint16_t msg_id;
             LOG("W800: Subscribing to topic: %s\n", MQTT_TOPIC);
             if (mqtt_subscribe(&g_broker, MQTT_TOPIC, &msg_id) < 0) {
                 LOG("W800: Failed to send subscribe packet.\n");
             }
             g_mqtt_connected = 2; // Move to state 2 (subscribed)
        }

        // Keep-alive ping and disconnection check
        if (g_mqtt_connected >= 1 && (xTaskGetTickCount() - last_ping > pdMS_TO_TICKS(50000)))
        {
            LOG("W800: Sending MQTT PINGREQ.\n");
            uint8_t ping_packet[] = {0xC0, 0x00};
            if (w800_at_socket_send(g_socket_id, ping_packet, sizeof(ping_packet)) < 0) {
                LOG("W800: Ping failed! Link possibly broken.\n");
                break; // Exit inner loop to trigger reconnection
            }
            last_ping = xTaskGetTickCount();
        }
    } // End of inner while(1)

    LOG("W800: Connection lost. Retrying in 3 seconds...\n");
    vTaskDelay(pdMS_TO_TICKS(3000));
    } // End of outer while(1)
}

int w800_mqtt_service_init(void)
{
    // The UART init is done in BSP_UART_Init
        if (xTaskCreate(w800_main_task, "w800_mqtt", 1024, NULL, 4, NULL) != pdPASS)
    {
        LOG("Failed to create w800_mqtt task!\n");
        return 0;
    }
    return 1;
}

int w800_publish_message(const char *topic, const char *message, uint8_t qos)
{
    if (g_mqtt_connected < 2) { // must be connected and subscribed
        return 0;
    }

    if (mqtt_publish(&g_broker, topic, message, strlen(message), qos) < 0) {
        LOG("Failed to queue publish packet.\n");
        return 0;
    }
    LOG("W800: Queued publish packet to %s\n", topic);
    return 1;
}

#endif /* ROBOT_MQTT_ENABLE */
