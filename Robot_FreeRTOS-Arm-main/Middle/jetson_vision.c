#include "jetson_vision.h"
#include "hal_data.h"
#include "bsp_uart.h"
#include "crc16.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* FSP generated UART names. */
extern sci_uart_instance_ctrl_t robot_jeston_ctrl;
extern const uart_cfg_t robot_jeston_cfg;

#define JETSON_RX_RING_SIZE             (1024u)
#define JETSON_RX_RING_MASK             (JETSON_RX_RING_SIZE - 1u)
#define JETSON_UART_REPORT_PERIOD_MS    (1000u)

#if ((JETSON_RX_RING_SIZE & JETSON_RX_RING_MASK) != 0u)
#error "JETSON_RX_RING_SIZE must be a power of two"
#endif

typedef enum
{
    JETSON_PARSER_WAIT_SOF = 0,
    JETSON_PARSER_READ_LEN,
    JETSON_PARSER_READ_FUNC,
    JETSON_PARSER_READ_PAYLOAD,
    JETSON_PARSER_READ_CHECKSUM,
    JETSON_PARSER_READ_EOF,
    JETSON_PARSER_READ_CTRL_FUNC,
    JETSON_PARSER_READ_CTRL_VALUE,
    JETSON_PARSER_READ_CTRL_EOF,
    JETSON_PARSER_READ_UNIFIED_SOF1,
    JETSON_PARSER_READ_UNIFIED_VER,
    JETSON_PARSER_READ_UNIFIED_TYPE,
    JETSON_PARSER_READ_UNIFIED_SEQ,
    JETSON_PARSER_READ_UNIFIED_LEN,
    JETSON_PARSER_READ_UNIFIED_PAYLOAD,
    JETSON_PARSER_READ_UNIFIED_CRC_LO,
    JETSON_PARSER_READ_UNIFIED_CRC_HI,
} jetson_parser_state_t;

typedef struct
{
    jetson_parser_state_t state;
    uint8_t len;
    uint8_t func;
    uint8_t payload[JETSON_VISION_PAYLOAD_LEN];
    uint8_t payload_index;
    uint8_t checksum;
    uint8_t ctrl_value;
    uint8_t unified_type;
    uint8_t unified_seq;
    uint8_t unified_len;
    uint8_t unified_payload[JETSON_UNIFIED_MAX_PAYLOAD];
    uint8_t unified_crc_data[4u + JETSON_UNIFIED_MAX_PAYLOAD];
    uint8_t unified_crc_len;
    uint8_t unified_crc_lo;
} jetson_parser_t;

static jetson_parser_t s_parser = { .state = JETSON_PARSER_WAIT_SOF };
static uint8_t s_rx_ring[JETSON_RX_RING_SIZE];
static volatile uint32_t s_rx_write_index = 0u;
static volatile uint32_t s_rx_read_index = 0u;
static volatile uint32_t s_rx_byte_count = 0u;
static volatile uint32_t s_rx_overflow_count = 0u;
static uint32_t s_rx_frames_ok = 0u;
static uint32_t s_rx_crc_fail = 0u;
static uint32_t s_tx_ok_count = 0u;
static uint32_t s_tx_start_fail_count = 0u;
static uint32_t s_tx_timeout_count = 0u;
static uint32_t s_last_uart_report_ms = 0u;

static int16_t s_latest_dcx = 0;
static int16_t s_latest_dcy = 0;
static bool s_latest_vision_valid = false;
static uint8_t s_latest_vision_seq = 0u;
static bool s_vision_seq_seen = false;
static volatile bool s_new_error = false;
static uint16_t s_latest_distance_mm = 0u;
static bool s_latest_distance_valid = false;
static uint8_t s_latest_distance_seq = 0u;
static bool s_distance_seq_seen = false;
static volatile bool s_new_safe_distance = false;
static uint8_t s_capture_action = 0u;
static uint8_t s_capture_point_id = 0u;
static uint8_t s_capture_seq = 0u;
static volatile bool s_new_capture_control = false;
static uint8_t s_target_control_value = 0u;
static uint8_t s_target_control_seq = 0u;
static volatile bool s_new_target_control = false;
static uint8_t s_workflow_action = 0u;
static uint8_t s_workflow_seq = 0u;
static volatile bool s_new_workflow_control = false;
static volatile bool s_unified_protocol_active = false;
static volatile bool s_heartbeat_seen = false;
static uint32_t s_last_heartbeat_ms = 0u;
static uint8_t s_last_unified_seq = 0u;
static uint8_t s_last_unified_type = 0u;

static SemaphoreHandle_t s_tx_sem = NULL;
static uint8_t s_tx_frame[2u + 4u + 3u + 2u];

static uint32_t jetson_now_ms(void)
{
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

static void parser_reset(void)
{
    s_parser.state = JETSON_PARSER_WAIT_SOF;
    s_parser.len = 0u;
    s_parser.func = 0u;
    s_parser.payload_index = 0u;
    s_parser.checksum = 0u;
    s_parser.ctrl_value = 0u;
    s_parser.unified_type = 0u;
    s_parser.unified_seq = 0u;
    s_parser.unified_len = 0u;
    s_parser.unified_crc_len = 0u;
    s_parser.unified_crc_lo = 0u;
}

static void parser_start_unified_frame(void)
{
    s_parser.unified_crc_len = 0u;
    s_parser.payload_index = 0u;
    s_parser.state = JETSON_PARSER_READ_UNIFIED_VER;
}

static void parser_resync(uint8_t byte)
{
    parser_reset();
#if JETSON_LEGACY_PROTOCOL_ENABLE
    if (byte == JETSON_SOF)
    {
        s_parser.state = JETSON_PARSER_READ_LEN;
        return;
    }
    if (byte == JETSON_CTRL_SOF)
    {
        s_parser.state = JETSON_PARSER_READ_CTRL_FUNC;
        return;
    }
#endif
    if (byte == JETSON_UNIFIED_SOF0)
    {
        s_parser.state = JETSON_PARSER_READ_UNIFIED_SOF1;
    }
}

static bool jetson_rx_ring_pop(uint8_t *byte)
{
    uint32_t read_index = s_rx_read_index;
    if ((byte == NULL) || (read_index == s_rx_write_index))
    {
        return false;
    }

    *byte = s_rx_ring[read_index];
    __DMB();
    s_rx_read_index = (read_index + 1u) & JETSON_RX_RING_MASK;
    return true;
}

static void handle_vision_error(int16_t dcx, int16_t dcy, bool valid, uint8_t seq)
{
    if (s_unified_protocol_active && s_vision_seq_seen &&
        (seq == s_latest_vision_seq))
    {
        LOG("[JETSON_RX] duplicate vision seq=%u ignored\r\n", (unsigned)seq);
        return;
    }

    __disable_irq();
    s_latest_dcx = dcx;
    s_latest_dcy = dcy;
    s_latest_vision_valid = valid;
    s_latest_vision_seq = seq;
    s_vision_seq_seen = true;
    s_new_error = true;
    __enable_irq();

    LOG("[JETSON_RX] vision seq=%u valid=%u dcx=%d dcy=%d\r\n",
        (unsigned)seq, valid ? 1u : 0u, (int)dcx, (int)dcy);
}

static void handle_valid_error_frame(void)
{
    int16_t dcx = (int16_t)(((uint16_t)s_parser.payload[1] << 8) | (uint16_t)s_parser.payload[0]);
    int16_t dcy = (int16_t)(((uint16_t)s_parser.payload[3] << 8) | (uint16_t)s_parser.payload[2]);

    handle_vision_error(dcx, dcy, true, 0u);
}

static void handle_target_control(uint8_t value, uint8_t seq)
{
    __disable_irq();
    s_target_control_value = value;
    s_target_control_seq = seq;
    s_new_target_control = true;
    __enable_irq();

    LOG("[JETSON_RX] target_ctrl=%u\r\n", (unsigned)value);
}

static void handle_safe_distance(uint16_t distance_mm, bool valid, uint8_t seq)
{
    if (s_unified_protocol_active && s_distance_seq_seen &&
        (seq == s_latest_distance_seq))
    {
        LOG("[JETSON_RX] duplicate safe_distance seq=%u ignored\r\n",
            (unsigned)seq);
        return;
    }

    __disable_irq();
    s_latest_distance_mm = distance_mm;
    s_latest_distance_valid = valid;
    s_latest_distance_seq = seq;
    s_distance_seq_seen = true;
    s_new_safe_distance = true;
    __enable_irq();

    LOG("[JETSON_RX] safe_distance=%u valid=%u\r\n",
        (unsigned)distance_mm,
        valid ? 1u : 0u);
}

static void handle_capture_control(uint8_t action, uint8_t point_id, uint8_t seq)
{
    __disable_irq();
    s_capture_action = action;
    s_capture_point_id = point_id;
    s_capture_seq = seq;
    s_new_capture_control = true;
    __enable_irq();

    LOG("[JETSON_RX] capture_ctrl action=%u point=%u\r\n",
        (unsigned)action,
        (unsigned)point_id);
}

static void handle_workflow_control(uint8_t action, uint8_t seq)
{
    __disable_irq();
    s_workflow_action = action;
    s_workflow_seq = seq;
    s_new_workflow_control = true;
    __enable_irq();

    LOG("[JETSON_RX] workflow_ctrl seq=%u action=%u\r\n",
        (unsigned)seq, (unsigned)action);
}

static void handle_valid_target_control_frame(void)
{
    handle_target_control(s_parser.ctrl_value, 0u);
}

static void unified_crc_push(uint8_t byte)
{
    if (s_parser.unified_crc_len < sizeof(s_parser.unified_crc_data))
    {
        s_parser.unified_crc_data[s_parser.unified_crc_len++] = byte;
    }
}

static void mark_unified_protocol_seen(void)
{
    __disable_irq();
    s_unified_protocol_active = true;
    s_last_unified_seq = s_parser.unified_seq;
    s_last_unified_type = s_parser.unified_type;
    __enable_irq();
}

static void mark_heartbeat_alive(void)
{
    __disable_irq();
    s_unified_protocol_active = true;
    s_heartbeat_seen = true;
    s_last_heartbeat_ms = jetson_now_ms();
    s_last_unified_seq = s_parser.unified_seq;
    __enable_irq();
}

static void handle_valid_unified_frame(void)
{
    if (s_parser.unified_seq == 0u)
    {
        LOG("[JETSON_RX] invalid zero seq type=0x%02X\r\n",
            (unsigned)s_parser.unified_type);
        (void)jetson_send_error(0u, JETSON_ERROR_INVALID_PARAM);
        return;
    }

    mark_unified_protocol_seen();

    switch (s_parser.unified_type)
    {
        case JETSON_MSG_HEARTBEAT:
            if (s_parser.unified_len == 4u)
            {
                mark_heartbeat_alive();
                LOG("[JETSON_RX] heartbeat seq=%u\r\n", (unsigned)s_parser.unified_seq);
            }
            else
            {
                (void)jetson_send_error(s_parser.unified_seq, JETSON_ERROR_INVALID_PARAM);
            }
            break;

        case JETSON_MSG_TARGET_CTRL:
            if (s_parser.unified_len == 1u)
            {
                handle_target_control(s_parser.unified_payload[0], s_parser.unified_seq);
            }
            else
            {
                (void)jetson_send_error(s_parser.unified_seq, JETSON_ERROR_INVALID_PARAM);
            }
            break;

        case JETSON_MSG_VISION_ERROR:
            if (s_parser.unified_len == 5u)
            {
                if (s_parser.unified_payload[4] > 1u)
                {
                    (void)jetson_send_error(s_parser.unified_seq,
                                            JETSON_ERROR_INVALID_PARAM);
                    break;
                }
                int16_t dcx = (int16_t)(((uint16_t)s_parser.unified_payload[1] << 8) |
                                        (uint16_t)s_parser.unified_payload[0]);
                int16_t dcy = (int16_t)(((uint16_t)s_parser.unified_payload[3] << 8) |
                                        (uint16_t)s_parser.unified_payload[2]);
                bool valid = (s_parser.unified_payload[4] != 0u);
                handle_vision_error(dcx, dcy, valid, s_parser.unified_seq);
            }
            else
            {
                (void)jetson_send_error(s_parser.unified_seq, JETSON_ERROR_INVALID_PARAM);
            }
            break;

        case JETSON_MSG_SAFE_DISTANCE:
            if (s_parser.unified_len == 3u)
            {
                if (s_parser.unified_payload[2] > 1u)
                {
                    (void)jetson_send_error(s_parser.unified_seq,
                                            JETSON_ERROR_INVALID_PARAM);
                    break;
                }
                uint16_t distance_mm = (uint16_t)(((uint16_t)s_parser.unified_payload[1] << 8) |
                                                 (uint16_t)s_parser.unified_payload[0]);
                bool valid = (s_parser.unified_payload[2] != 0u);
                handle_safe_distance(distance_mm, valid, s_parser.unified_seq);
            }
            else
            {
                (void)jetson_send_error(s_parser.unified_seq, JETSON_ERROR_INVALID_PARAM);
            }
            break;

        case JETSON_MSG_CAPTURE_CTRL:
            if (s_parser.unified_len == 2u)
            {
                uint8_t action = s_parser.unified_payload[0];
                uint8_t point_id = s_parser.unified_payload[1];
                handle_capture_control(action, point_id, s_parser.unified_seq);
            }
            else
            {
                (void)jetson_send_error(s_parser.unified_seq, JETSON_ERROR_INVALID_PARAM);
            }
            break;

        case JETSON_MSG_WORKFLOW_CTRL:
            if (s_parser.unified_len == 1u)
            {
                handle_workflow_control(s_parser.unified_payload[0], s_parser.unified_seq);
            }
            else
            {
                (void)jetson_send_error(s_parser.unified_seq, JETSON_ERROR_INVALID_PARAM);
            }
            break;

        default:
            LOG("[JETSON_RX] unsupported unified type=0x%02X len=%u\r\n",
                (unsigned)s_parser.unified_type,
                (unsigned)s_parser.unified_len);
            (void)jetson_send_error(s_parser.unified_seq, JETSON_ERROR_UNKNOWN_TYPE);
            break;
    }
}

static void process_byte(uint8_t byte)
{
    switch (s_parser.state)
    {
        case JETSON_PARSER_WAIT_SOF:
#if JETSON_LEGACY_PROTOCOL_ENABLE
            if (byte == JETSON_SOF)
            {
                s_parser.state = JETSON_PARSER_READ_LEN;
                s_parser.payload_index = 0u;
                s_parser.checksum = 0u;
            }
            else if (byte == JETSON_CTRL_SOF)
            {
                s_parser.state = JETSON_PARSER_READ_CTRL_FUNC;
            }
            else
#endif
            if (byte == JETSON_UNIFIED_SOF0)
            {
                s_parser.state = JETSON_PARSER_READ_UNIFIED_SOF1;
            }
            break;

        case JETSON_PARSER_READ_LEN:
            if (byte != JETSON_FRAME_LEN)
            {
                parser_resync(byte);
                break;
            }
            s_parser.len = byte;
            s_parser.checksum = (uint8_t)(s_parser.checksum + byte);
            s_parser.state = JETSON_PARSER_READ_FUNC;
            break;

        case JETSON_PARSER_READ_FUNC:
            if (byte != JETSON_FUNC_VISION_ERROR)
            {
                parser_resync(byte);
                break;
            }
            s_parser.func = byte;
            s_parser.checksum = (uint8_t)(s_parser.checksum + byte);
            s_parser.state = JETSON_PARSER_READ_PAYLOAD;
            break;

        case JETSON_PARSER_READ_PAYLOAD:
            s_parser.payload[s_parser.payload_index++] = byte;
            s_parser.checksum = (uint8_t)(s_parser.checksum + byte);
            if (s_parser.payload_index >= JETSON_VISION_PAYLOAD_LEN)
            {
                s_parser.state = JETSON_PARSER_READ_CHECKSUM;
            }
            break;

        case JETSON_PARSER_READ_CHECKSUM:
            if (byte == s_parser.checksum)
            {
                s_parser.state = JETSON_PARSER_READ_EOF;
            }
            else
            {
                parser_resync(byte);
            }
            break;

        case JETSON_PARSER_READ_EOF:
            if (byte == JETSON_EOF)
            {
                handle_valid_error_frame();
                parser_reset();
            }
            else
            {
                parser_resync(byte);
            }
            break;

        case JETSON_PARSER_READ_CTRL_FUNC:
            if (byte != JETSON_FUNC_TARGET_CTRL)
            {
                parser_resync(byte);
                break;
            }
            s_parser.func = byte;
            s_parser.state = JETSON_PARSER_READ_CTRL_VALUE;
            break;

        case JETSON_PARSER_READ_CTRL_VALUE:
            if ((byte != 0u) && (byte != 1u))
            {
                parser_resync(byte);
                break;
            }
            s_parser.ctrl_value = byte;
            s_parser.state = JETSON_PARSER_READ_CTRL_EOF;
            break;

        case JETSON_PARSER_READ_CTRL_EOF:
            if (byte == JETSON_CTRL_EOF)
            {
                handle_valid_target_control_frame();
                parser_reset();
            }
            else
            {
                parser_resync(byte);
            }
            break;

        case JETSON_PARSER_READ_UNIFIED_SOF1:
            if (byte == JETSON_UNIFIED_SOF1)
            {
                parser_start_unified_frame();
            }
            else
            {
                parser_resync(byte);
            }
            break;

        case JETSON_PARSER_READ_UNIFIED_VER:
            if (byte != JETSON_UNIFIED_VERSION)
            {
                parser_resync(byte);
                break;
            }
            unified_crc_push(byte);
            s_parser.state = JETSON_PARSER_READ_UNIFIED_TYPE;
            break;

        case JETSON_PARSER_READ_UNIFIED_TYPE:
            s_parser.unified_type = byte;
            unified_crc_push(byte);
            s_parser.state = JETSON_PARSER_READ_UNIFIED_SEQ;
            break;

        case JETSON_PARSER_READ_UNIFIED_SEQ:
            s_parser.unified_seq = byte;
            unified_crc_push(byte);
            s_parser.state = JETSON_PARSER_READ_UNIFIED_LEN;
            break;

        case JETSON_PARSER_READ_UNIFIED_LEN:
            if (byte > JETSON_UNIFIED_MAX_PAYLOAD)
            {
                parser_resync(byte);
                break;
            }
            s_parser.unified_len = byte;
            s_parser.payload_index = 0u;
            unified_crc_push(byte);
            s_parser.state = (byte == 0u) ? JETSON_PARSER_READ_UNIFIED_CRC_LO :
                                            JETSON_PARSER_READ_UNIFIED_PAYLOAD;
            break;

        case JETSON_PARSER_READ_UNIFIED_PAYLOAD:
            s_parser.unified_payload[s_parser.payload_index++] = byte;
            unified_crc_push(byte);
            if (s_parser.payload_index >= s_parser.unified_len)
            {
                s_parser.state = JETSON_PARSER_READ_UNIFIED_CRC_LO;
            }
            break;

        case JETSON_PARSER_READ_UNIFIED_CRC_LO:
            s_parser.unified_crc_lo = byte;
            s_parser.state = JETSON_PARSER_READ_UNIFIED_CRC_HI;
            break;

        case JETSON_PARSER_READ_UNIFIED_CRC_HI:
        {
            uint16_t rx_crc = (uint16_t)((uint16_t)s_parser.unified_crc_lo |
                                         ((uint16_t)byte << 8));
            uint16_t calc_crc = crc_modbus(s_parser.unified_crc_data,
                                            s_parser.unified_crc_len);
            if (rx_crc == calc_crc)
            {
                s_rx_frames_ok++;
                handle_valid_unified_frame();
                parser_reset();
            }
            else
            {
                bool next_header_in_crc =
                    (s_parser.unified_crc_lo == JETSON_UNIFIED_SOF0) &&
                    (byte == JETSON_UNIFIED_SOF1);
                s_rx_crc_fail++;
                LOG("[JETSON_RX] crc failed type=0x%02X rx=0x%04X calc=0x%04X\r\n",
                    (unsigned)s_parser.unified_type,
                    (unsigned)rx_crc,
                    (unsigned)calc_crc);
                if (next_header_in_crc)
                {
                    parser_reset();
                    parser_start_unified_frame();
                }
                else
                {
                    parser_resync(byte);
                }
            }
            break;
        }

        default:
            parser_resync(byte);
            break;
    }
}
void jetson_notify_rx_char_from_isr(uint8_t byte)
{
    uint32_t write_index = s_rx_write_index;
    uint32_t next_index = (write_index + 1u) & JETSON_RX_RING_MASK;

    s_rx_byte_count++;
    if (next_index == s_rx_read_index)
    {
        s_rx_overflow_count++;
        return;
    }

    s_rx_ring[write_index] = byte;
    __DMB();
    s_rx_write_index = next_index;
}

void jetson_vision_init(void)
{
    if (s_tx_sem == NULL)
    {
        s_tx_sem = xSemaphoreCreateBinary();
    }

    parser_reset();
    s_rx_write_index = 0u;
    s_rx_read_index = 0u;
    s_rx_byte_count = 0u;
    s_rx_overflow_count = 0u;
    s_rx_frames_ok = 0u;
    s_rx_crc_fail = 0u;
    s_tx_ok_count = 0u;
    s_tx_start_fail_count = 0u;
    s_tx_timeout_count = 0u;
    s_last_uart_report_ms = 0u;
    s_new_error = false;
    s_new_safe_distance = false;
    s_new_capture_control = false;
    s_new_target_control = false;
    s_new_workflow_control = false;
    s_vision_seq_seen = false;
    s_distance_seq_seen = false;
    s_unified_protocol_active = false;
    s_heartbeat_seen = false;
    s_last_heartbeat_ms = 0u;
    s_last_unified_seq = 0u;
    s_last_unified_type = 0u;

    fsp_err_t err = R_SCI_UART_Open(&robot_jeston_ctrl, &robot_jeston_cfg);
    if (FSP_SUCCESS != err)
    {
        LOG("Jetson UART open failed, err=%d\r\n", (int)err);
        __BKPT(0);
    }
}

void jetson_vision_process(void)
{
    uint8_t byte = 0u;
    while (jetson_rx_ring_pop(&byte))
    {
        process_byte(byte);
    }

    uint32_t now_ms = jetson_now_ms();
    if ((uint32_t)(now_ms - s_last_uart_report_ms) >= JETSON_UART_REPORT_PERIOD_MS)
    {
        s_last_uart_report_ms = now_ms;
        LOG("[JETSON_UART] rx_bytes=%lu frames_ok=%lu crc_fail=%lu overflow=%lu "
            "tx_ok=%lu tx_start_fail=%lu tx_timeout=%lu last_type=0x%02X last_seq=%u\r\n",
            (unsigned long)s_rx_byte_count,
            (unsigned long)s_rx_frames_ok,
            (unsigned long)s_rx_crc_fail,
            (unsigned long)s_rx_overflow_count,
            (unsigned long)s_tx_ok_count,
            (unsigned long)s_tx_start_fail_count,
            (unsigned long)s_tx_timeout_count,
            (unsigned)s_last_unified_type,
            (unsigned)s_last_unified_seq);
    }
}
bool jetson_get_vision_error_ex(int16_t *dcx, int16_t *dcy, bool *valid, uint8_t *seq)
{
    if ((dcx == NULL) || (dcy == NULL) || (valid == NULL) || (seq == NULL))
    {
        return false;
    }

    if (!s_new_error)
    {
        return false;
    }

    __disable_irq();
    *dcx = s_latest_dcx;
    *dcy = s_latest_dcy;
    *valid = s_latest_vision_valid;
    *seq = s_latest_vision_seq;
    s_new_error = false;
    __enable_irq();
    return true;
}

bool jetson_get_vision_error(int16_t *dcx, int16_t *dcy)
{
    bool valid = false;
    uint8_t seq = 0u;

    if (!jetson_get_vision_error_ex(dcx, dcy, &valid, &seq))
    {
        return false;
    }
    return valid;
}

bool jetson_get_target_control_value(uint8_t *value, uint8_t *seq)
{
    if ((value == NULL) || (seq == NULL))
    {
        return false;
    }

    if (!s_new_target_control)
    {
        return false;
    }

    __disable_irq();
    *value = s_target_control_value;
    *seq = s_target_control_seq;
    s_new_target_control = false;
    __enable_irq();
    return true;
}

bool jetson_get_target_control_ex(bool *enable, uint8_t *seq)
{
    uint8_t value = 0u;
    if ((enable == NULL) || !jetson_get_target_control_value(&value, seq))
    {
        return false;
    }
    *enable = (value != 0u);
    return true;
}

bool jetson_get_target_control(bool *enable)
{
    uint8_t seq = 0u;
    return jetson_get_target_control_ex(enable, &seq);
}

bool jetson_get_capture_control_ex(uint8_t *action, uint8_t *point_id, uint8_t *seq)
{
    if ((action == NULL) || (point_id == NULL) || (seq == NULL))
    {
        return false;
    }

    if (!s_new_capture_control)
    {
        return false;
    }

    __disable_irq();
    *action = s_capture_action;
    *point_id = s_capture_point_id;
    *seq = s_capture_seq;
    s_new_capture_control = false;
    __enable_irq();
    return true;
}

bool jetson_get_capture_control(uint8_t *action, uint8_t *point_id)
{
    uint8_t seq = 0u;
    return jetson_get_capture_control_ex(action, point_id, &seq);
}

bool jetson_get_safe_distance_ex(uint16_t *distance_mm, bool *valid, uint8_t *seq)
{
    if ((distance_mm == NULL) || (valid == NULL) || (seq == NULL))
    {
        return false;
    }

    if (!s_new_safe_distance)
    {
        return false;
    }

    __disable_irq();
    *distance_mm = s_latest_distance_mm;
    *valid = s_latest_distance_valid;
    *seq = s_latest_distance_seq;
    s_new_safe_distance = false;
    __enable_irq();
    return true;
}

bool jetson_get_safe_distance(uint16_t *distance_mm, bool *valid)
{
    uint8_t seq = 0u;
    return jetson_get_safe_distance_ex(distance_mm, valid, &seq);
}

bool jetson_get_workflow_control(uint8_t *action, uint8_t *seq)
{
    if ((action == NULL) || (seq == NULL) || !s_new_workflow_control)
    {
        return false;
    }

    __disable_irq();
    *action = s_workflow_action;
    *seq = s_workflow_seq;
    s_new_workflow_control = false;
    __enable_irq();
    return true;
}

bool jetson_is_unified_protocol_active(void)
{
    return s_unified_protocol_active;
}

bool jetson_is_link_alive(uint32_t now_ms)
{
    if (!s_unified_protocol_active)
    {
#if JETSON_LEGACY_PROTOCOL_ENABLE
        return true;
#else
        return false;
#endif
    }

    if (!s_heartbeat_seen)
    {
        return false;
    }

    if (now_ms < s_last_heartbeat_ms)
    {
        return true;
    }

    return ((now_ms - s_last_heartbeat_ms) <= JETSON_HEARTBEAT_TIMEOUT_MS);
}

static uint32_t build_unified_status_frame(uint8_t seq, uint8_t event,
                                           uint8_t value, uint8_t error_code)
{
    uint8_t payload[3] = {event, value, error_code};
    uint8_t payload_len = 3u;

    s_tx_frame[0] = JETSON_UNIFIED_SOF0;
    s_tx_frame[1] = JETSON_UNIFIED_SOF1;
    s_tx_frame[2] = JETSON_UNIFIED_VERSION;
    s_tx_frame[3] = JETSON_MSG_STATUS;
    s_tx_frame[4] = seq;
    s_tx_frame[5] = payload_len;
    for (uint8_t i = 0u; i < payload_len; i++)
    {
        s_tx_frame[6u + i] = payload[i];
    }

    uint16_t crc = crc_modbus(&s_tx_frame[2], (uint16_t)(4u + payload_len));
    s_tx_frame[6u + payload_len] = (uint8_t)(crc & 0xFFu);
    s_tx_frame[7u + payload_len] = (uint8_t)((crc >> 8) & 0xFFu);

    return (uint32_t)(8u + payload_len);
}

static uint32_t build_unified_error_frame(uint8_t seq, uint8_t error_code)
{
    const uint8_t payload_len = 1u;

    s_tx_frame[0] = JETSON_UNIFIED_SOF0;
    s_tx_frame[1] = JETSON_UNIFIED_SOF1;
    s_tx_frame[2] = JETSON_UNIFIED_VERSION;
    s_tx_frame[3] = JETSON_MSG_ERROR;
    s_tx_frame[4] = seq;
    s_tx_frame[5] = payload_len;
    s_tx_frame[6] = error_code;

    uint16_t crc = crc_modbus(&s_tx_frame[2], 5u);
    s_tx_frame[7] = (uint8_t)(crc & 0xFFu);
    s_tx_frame[8] = (uint8_t)((crc >> 8) & 0xFFu);
    return 9u;
}

static bool jetson_send_tx_frame(uint32_t tx_len)
{
    if (s_tx_sem == NULL)
    {
        s_tx_start_fail_count++;
        return false;
    }

    (void)xSemaphoreTake(s_tx_sem, 0);
    fsp_err_t err = R_SCI_UART_Write(&robot_jeston_ctrl, s_tx_frame, tx_len);
    if (FSP_SUCCESS != err)
    {
        s_tx_start_fail_count++;
        LOG("Jetson TX start failed, err=%d\r\n", (int)err);
        return false;
    }

    if (xSemaphoreTake(s_tx_sem, pdMS_TO_TICKS(20)) != pdTRUE)
    {
        s_tx_timeout_count++;
        LOG("Jetson TX wait complete timeout\r\n");
        return false;
    }

    s_tx_ok_count++;
    return true;
}

bool jetson_send_status(uint8_t seq, uint8_t event, uint8_t value, uint8_t error_code)
{
    uint32_t tx_len = build_unified_status_frame(seq, event, value, error_code);
    return jetson_send_tx_frame(tx_len);
}

bool jetson_send_error(uint8_t seq, uint8_t error_code)
{
    uint32_t tx_len = build_unified_error_frame(seq, error_code);
    return jetson_send_tx_frame(tx_len);
}

bool jetson_send_status_u8(uint8_t func, uint8_t value)
{
    if (s_unified_protocol_active)
    {
        if (func == RA6_TO_JETSON_ERROR)
        {
            return jetson_send_error(s_last_unified_seq, value);
        }
        return jetson_send_status(s_last_unified_seq, func, value, JETSON_ERROR_NONE);
    }

#if JETSON_LEGACY_PROTOCOL_ENABLE
    s_tx_frame[0] = RA6_TO_JETSON_SOF;
    s_tx_frame[1] = func;
    s_tx_frame[2] = value;
    s_tx_frame[3] = RA6_TO_JETSON_EOF;
    return jetson_send_tx_frame(4u);
#else
    return false;
#endif
}

void jetson_notify_tx_complete_from_isr(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (s_tx_sem != NULL)
    {
        xSemaphoreGiveFromISR(s_tx_sem, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}
