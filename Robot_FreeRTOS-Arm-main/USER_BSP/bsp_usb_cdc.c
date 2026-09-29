#include "bsp_usb_cdc.h"
#include "robot.h"

#include "FreeRTOS.h"
#include "task.h"

#include <string.h>

static uint8_t s_usb_rx_buf[USB_CDC_RX_CHUNK_SIZE];
static char s_usb_cmd_buf[USB_CDC_CMD_BUF_SIZE];
static uint32_t s_usb_cmd_pos;

static uint8_t s_usb_tx_buf[USB_CDC_TX_BUF_SIZE];
static volatile bool s_usb_opened;
static volatile bool s_usb_configured;
static volatile bool s_usb_write_busy;

static void usb_cdc_start_read(void)
{
    if (!s_usb_opened || !s_usb_configured)
    {
        return;
    }

    (void)R_USB_Read(&g_basic0_ctrl, s_usb_rx_buf, sizeof(s_usb_rx_buf), USB_CLASS_PCDC);
}

static void usb_cdc_clear_rx_line(void)
{
    s_usb_cmd_pos = 0;
    memset(s_usb_cmd_buf, 0, sizeof(s_usb_cmd_buf));
}

static void usb_cdc_submit_command(void)
{
    s_usb_cmd_buf[s_usb_cmd_pos] = '\0';

    if (s_usb_cmd_pos == 0)
    {
        return;
    }

    if (xPortIsInsideInterrupt() != pdFALSE)
    {
        robot_cmd_send_from_isr(s_usb_cmd_buf, CMD_TYPE_USB);
    }
    else
    {
        (void)robot_cmd_send(s_usb_cmd_buf, CMD_TYPE_USB);
    }

    usb_cdc_clear_rx_line();
}

static void usb_cdc_process_rx(uint8_t const *data, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++)
    {
        char ch = (char)data[i];

        if ((ch == '\r') || (ch == '\n'))
        {
            usb_cdc_submit_command();
            continue;
        }

        if (s_usb_cmd_pos < (USB_CDC_CMD_BUF_SIZE - 1U))
        {
            s_usb_cmd_buf[s_usb_cmd_pos++] = ch;
        }
        else
        {
            usb_cdc_clear_rx_line();
        }
    }
}

bool BSP_USB_CDC_Init(void)
{
    fsp_err_t err = R_USB_Open(&g_basic0_ctrl, &g_basic0_cfg);
    if (FSP_SUCCESS != err)
    {
        s_usb_opened = false;
        s_usb_configured = false;
        s_usb_write_busy = false;
        return false;
    }

    s_usb_opened = true;
    s_usb_configured = false;
    s_usb_write_busy = false;
    usb_cdc_clear_rx_line();
    return true;
}

bool BSP_USB_CDC_IsConfigured(void)
{
    return s_usb_configured;
}

void BSP_USB_CDC_WriteBestEffort(const char *buffer, uint32_t len)
{
    if ((buffer == NULL) || (len == 0U))
    {
        return;
    }

    if (!s_usb_opened || !s_usb_configured || s_usb_write_busy)
    {
        return;
    }

    if (len > sizeof(s_usb_tx_buf))
    {
        len = sizeof(s_usb_tx_buf);
    }

    s_usb_write_busy = true;
    memcpy(s_usb_tx_buf, buffer, len);

    fsp_err_t err = R_USB_Write(&g_basic0_ctrl, s_usb_tx_buf, len, USB_CLASS_PCDC);
    if (FSP_SUCCESS != err)
    {
        s_usb_write_busy = false;
    }
}

void usb_pcdc_callback(usb_event_info_t *p_event, usb_hdl_t cur_task, usb_onoff_t usb_state)
{
    FSP_PARAMETER_NOT_USED(cur_task);
    FSP_PARAMETER_NOT_USED(usb_state);

    if (p_event == NULL)
    {
        return;
    }

    switch (p_event->event)
    {
        case USB_STATUS_CONFIGURED:
            s_usb_configured = true;
            s_usb_write_busy = false;
            usb_cdc_clear_rx_line();
            usb_cdc_start_read();
            break;

        case USB_STATUS_READ_COMPLETE:
        {
            uint32_t len = p_event->data_size;
            if (len > sizeof(s_usb_rx_buf))
            {
                len = sizeof(s_usb_rx_buf);
            }
            usb_cdc_process_rx(s_usb_rx_buf, len);
            usb_cdc_start_read();
            break;
        }

        case USB_STATUS_WRITE_COMPLETE:
            s_usb_write_busy = false;
            break;

        case USB_STATUS_RESUME:
            if (s_usb_opened)
            {
                s_usb_configured = true;
                s_usb_write_busy = false;
                usb_cdc_start_read();
            }
            break;

        case USB_STATUS_DETACH:
        case USB_STATUS_SUSPEND:
            s_usb_configured = false;
            s_usb_write_busy = false;
            usb_cdc_clear_rx_line();
            break;

        default:
            break;
    }
}
