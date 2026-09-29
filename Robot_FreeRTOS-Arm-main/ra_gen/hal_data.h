/* generated HAL header file - do not edit */
#ifndef HAL_DATA_H_
#define HAL_DATA_H_
#include <stdint.h>
#include "bsp_api.h"
#include "common_data.h"
#include "r_usb_basic.h"
#include "r_usb_basic_api.h"
#include "r_usb_pcdc_api.h"
#include "r_sci_uart.h"
            #include "r_uart_api.h"
#include "r_dmac.h"
#include "r_transfer_api.h"
#include "r_canfd.h"
#include "r_can_api.h"
#include "r_dtc.h"
#include "r_transfer_api.h"
FSP_HEADER
/* Basic on USB Instance. */
extern const usb_instance_t g_basic0;

/** Access the USB instance using these structures when calling API functions directly (::p_api is not used). */
extern usb_instance_ctrl_t g_basic0_ctrl;
extern const usb_cfg_t g_basic0_cfg;

#ifndef NULL
void NULL(void *);
#endif

#if 0 == BSP_CFG_RTOS
#ifndef usb_pcdc_callback
void usb_pcdc_callback(usb_callback_args_t *);
#endif
#endif

#if 2 == BSP_CFG_RTOS
#ifndef usb_pcdc_callback
void usb_pcdc_callback(usb_event_info_t *, usb_hdl_t, usb_onoff_t);
#endif
#endif
/** CDC Driver on USB Instance. */
/** UART on SCI Instance. */
            extern const uart_instance_t      esp8266;

            /** Access the UART instance using these structures when calling API functions directly (::p_api is not used). */
            extern sci_uart_instance_ctrl_t     esp8266_ctrl;
            extern const uart_cfg_t esp8266_cfg;
            extern const sci_uart_extended_cfg_t esp8266_cfg_extend;

            #ifndef esp8266_uart_callback
            void esp8266_uart_callback(uart_callback_args_t * p_args);
            #endif
/* Transfer on DMAC Instance. */
extern const transfer_instance_t g_dma_jeston_rx;

/** Access the DMAC instance using these structures when calling API functions directly (::p_api is not used). */
extern dmac_instance_ctrl_t g_dma_jeston_rx_ctrl;
extern const transfer_cfg_t g_dma_jeston_rx_cfg;

#ifndef NULL
void NULL(transfer_callback_args_t * p_args);
#endif
/** CANFD on CANFD Instance. */
extern const can_instance_t g_canfd0;
/** Access the CANFD instance using these structures when calling API functions directly (::p_api is not used). */
extern canfd_instance_ctrl_t g_canfd0_ctrl;
extern const can_cfg_t g_canfd0_cfg;
extern const canfd_extended_cfg_t g_canfd0_cfg_extend;

#ifndef canfd0_callback
void canfd0_callback(can_callback_args_t * p_args);
#endif

/* Global configuration (referenced by all instances) */
extern canfd_global_cfg_t g_canfd_global_cfg;
/* Transfer on DMAC Instance. */
extern const transfer_instance_t robot_mqtt_dma;

/** Access the DMAC instance using these structures when calling API functions directly (::p_api is not used). */
extern dmac_instance_ctrl_t robot_mqtt_dma_ctrl;
extern const transfer_cfg_t robot_mqtt_dma_cfg;

#ifndef NULL
void NULL(transfer_callback_args_t * p_args);
#endif
/* Transfer on DTC Instance. */
extern const transfer_instance_t g_transfer_jeston_rx;

/** Access the DTC instance using these structures when calling API functions directly (::p_api is not used). */
extern dtc_instance_ctrl_t g_transfer_jeston_rx_ctrl;
extern const transfer_cfg_t g_transfer_jeston_rx_cfg;
/** UART on SCI Instance. */
            extern const uart_instance_t      robot_jeston;

            /** Access the UART instance using these structures when calling API functions directly (::p_api is not used). */
            extern sci_uart_instance_ctrl_t     robot_jeston_ctrl;
            extern const uart_cfg_t robot_jeston_cfg;
            extern const sci_uart_extended_cfg_t robot_jeston_cfg_extend;

            #ifndef robot_jetson_callback
            void robot_jetson_callback(uart_callback_args_t * p_args);
            #endif
/** UART on SCI Instance. */
            extern const uart_instance_t      robot_mqtt;

            /** Access the UART instance using these structures when calling API functions directly (::p_api is not used). */
            extern sci_uart_instance_ctrl_t     robot_mqtt_ctrl;
            extern const uart_cfg_t robot_mqtt_cfg;
            extern const sci_uart_extended_cfg_t robot_mqtt_cfg_extend;

            #ifndef robot_mqtt_callback
            void robot_mqtt_callback(uart_callback_args_t * p_args);
            #endif
/** UART on SCI Instance. */
            extern const uart_instance_t      robot_uart1;

            /** Access the UART instance using these structures when calling API functions directly (::p_api is not used). */
            extern sci_uart_instance_ctrl_t     robot_uart1_ctrl;
            extern const uart_cfg_t robot_uart1_cfg;
            extern const sci_uart_extended_cfg_t robot_uart1_cfg_extend;

            #ifndef robot_uart1_callback
            void robot_uart1_callback(uart_callback_args_t * p_args);
            #endif
void hal_entry(void);
void g_hal_init(void);
FSP_FOOTER
#endif /* HAL_DATA_H_ */
