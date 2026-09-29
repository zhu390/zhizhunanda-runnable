#ifndef __BSP_USB_CDC_H__
#define __BSP_USB_CDC_H__

#include "hal_data.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define USB_CDC_RX_CHUNK_SIZE      (64U)
#define USB_CDC_CMD_BUF_SIZE       (128U)
#define USB_CDC_TX_BUF_SIZE        (256U)

bool BSP_USB_CDC_Init(void);
bool BSP_USB_CDC_IsConfigured(void);
void BSP_USB_CDC_WriteBestEffort(const char *buffer, uint32_t len);

void usb_pcdc_callback(usb_event_info_t *p_event, usb_hdl_t cur_task, usb_onoff_t usb_state);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_USB_CDC_H__ */
