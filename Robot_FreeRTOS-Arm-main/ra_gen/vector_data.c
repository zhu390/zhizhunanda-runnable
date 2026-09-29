/* generated vector source file - do not edit */
        #include "bsp_api.h"
        /* Do not build these data structures if no interrupts are currently allocated because IAR will have build errors. */
        #if VECTOR_DATA_IRQ_COUNT > 0
        BSP_DONT_REMOVE const fsp_vector_t g_vector_table[BSP_ICU_VECTOR_NUM_ENTRIES] BSP_PLACE_IN_SECTION(BSP_SECTION_APPLICATION_VECTORS) =
        {
                        [0] = r_icu_isr, /* ICU IRQ1 (External pin interrupt 1) */
            [1] = r_icu_isr, /* ICU IRQ2 (External pin interrupt 2) */
            [2] = r_icu_isr, /* ICU IRQ3 (External pin interrupt 3) */
            [3] = r_icu_isr, /* ICU IRQ4 (External pin interrupt 4) */
            [4] = r_icu_isr, /* ICU IRQ5 (External pin interrupt 5) */
            [5] = r_icu_isr, /* ICU IRQ6 (External pin interrupt 6) */
            [6] = sci_uart_rxi_isr, /* SCI9 RXI (Receive data full) */
            [7] = sci_uart_txi_isr, /* SCI9 TXI (Transmit data empty) */
            [8] = sci_uart_tei_isr, /* SCI9 TEI (Transmit end) */
            [9] = sci_uart_eri_isr, /* SCI9 ERI (Receive error) */
            [10] = sci_uart_rxi_isr, /* SCI6 RXI (Receive data full) */
            [11] = sci_uart_txi_isr, /* SCI6 TXI (Transmit data empty) */
            [12] = sci_uart_tei_isr, /* SCI6 TEI (Transmit end) */
            [13] = sci_uart_eri_isr, /* SCI6 ERI (Receive error) */
            [14] = sci_uart_rxi_isr, /* SCI2 RXI (Receive data full) */
            [15] = sci_uart_txi_isr, /* SCI2 TXI (Transmit data empty) */
            [16] = sci_uart_tei_isr, /* SCI2 TEI (Transmit end) */
            [17] = sci_uart_eri_isr, /* SCI2 ERI (Receive error) */
            [18] = dmac_int_isr, /* DMAC6 INT (DMAC6 transfer end) */
            [19] = canfd_error_isr, /* CAN0 CHERR (Channel  error) */
            [20] = canfd_channel_tx_isr, /* CAN0 TX (Transmit interrupt) */
            [21] = canfd_common_fifo_rx_isr, /* CAN0 COMFRX (Common FIFO receive interrupt) */
            [22] = canfd_error_isr, /* CAN GLERR (Global error) */
            [23] = canfd_rx_fifo_isr, /* CAN RXF (Global receive FIFO interrupt) */
            [24] = sci_uart_rxi_isr, /* SCI5 RXI (Receive data full) */
            [25] = sci_uart_txi_isr, /* SCI5 TXI (Transmit data empty) */
            [26] = sci_uart_tei_isr, /* SCI5 TEI (Transmit end) */
            [27] = sci_uart_eri_isr, /* SCI5 ERI (Receive error) */
            [28] = usbfs_interrupt_handler, /* USBFS INT (USBFS interrupt) */
            [29] = usbfs_resume_handler, /* USBFS RESUME (USBFS resume interrupt) */
            [30] = usbfs_d0fifo_handler, /* USBFS FIFO 0 (DMA/DTC transfer request 0) */
            [31] = usbfs_d1fifo_handler, /* USBFS FIFO 1 (DMA/DTC transfer request 1) */
        };
        #if BSP_FEATURE_ICU_HAS_IELSR
        const bsp_interrupt_event_t g_interrupt_event_link_select[BSP_ICU_VECTOR_NUM_ENTRIES] =
        {
            [0] = BSP_PRV_VECT_ENUM(EVENT_ICU_IRQ1,GROUP0), /* ICU IRQ1 (External pin interrupt 1) */
            [1] = BSP_PRV_VECT_ENUM(EVENT_ICU_IRQ2,GROUP1), /* ICU IRQ2 (External pin interrupt 2) */
            [2] = BSP_PRV_VECT_ENUM(EVENT_ICU_IRQ3,GROUP2), /* ICU IRQ3 (External pin interrupt 3) */
            [3] = BSP_PRV_VECT_ENUM(EVENT_ICU_IRQ4,GROUP3), /* ICU IRQ4 (External pin interrupt 4) */
            [4] = BSP_PRV_VECT_ENUM(EVENT_ICU_IRQ5,GROUP4), /* ICU IRQ5 (External pin interrupt 5) */
            [5] = BSP_PRV_VECT_ENUM(EVENT_ICU_IRQ6,GROUP5), /* ICU IRQ6 (External pin interrupt 6) */
            [6] = BSP_PRV_VECT_ENUM(EVENT_SCI9_RXI,GROUP6), /* SCI9 RXI (Receive data full) */
            [7] = BSP_PRV_VECT_ENUM(EVENT_SCI9_TXI,GROUP7), /* SCI9 TXI (Transmit data empty) */
            [8] = BSP_PRV_VECT_ENUM(EVENT_SCI9_TEI,GROUP0), /* SCI9 TEI (Transmit end) */
            [9] = BSP_PRV_VECT_ENUM(EVENT_SCI9_ERI,GROUP1), /* SCI9 ERI (Receive error) */
            [10] = BSP_PRV_VECT_ENUM(EVENT_SCI6_RXI,GROUP2), /* SCI6 RXI (Receive data full) */
            [11] = BSP_PRV_VECT_ENUM(EVENT_SCI6_TXI,GROUP3), /* SCI6 TXI (Transmit data empty) */
            [12] = BSP_PRV_VECT_ENUM(EVENT_SCI6_TEI,GROUP4), /* SCI6 TEI (Transmit end) */
            [13] = BSP_PRV_VECT_ENUM(EVENT_SCI6_ERI,GROUP5), /* SCI6 ERI (Receive error) */
            [14] = BSP_PRV_VECT_ENUM(EVENT_SCI2_RXI,GROUP6), /* SCI2 RXI (Receive data full) */
            [15] = BSP_PRV_VECT_ENUM(EVENT_SCI2_TXI,GROUP7), /* SCI2 TXI (Transmit data empty) */
            [16] = BSP_PRV_VECT_ENUM(EVENT_SCI2_TEI,GROUP0), /* SCI2 TEI (Transmit end) */
            [17] = BSP_PRV_VECT_ENUM(EVENT_SCI2_ERI,GROUP1), /* SCI2 ERI (Receive error) */
            [18] = BSP_PRV_VECT_ENUM(EVENT_DMAC6_INT,GROUP2), /* DMAC6 INT (DMAC6 transfer end) */
            [19] = BSP_PRV_VECT_ENUM(EVENT_CAN0_CHERR,GROUP3), /* CAN0 CHERR (Channel  error) */
            [20] = BSP_PRV_VECT_ENUM(EVENT_CAN0_TX,GROUP4), /* CAN0 TX (Transmit interrupt) */
            [21] = BSP_PRV_VECT_ENUM(EVENT_CAN0_COMFRX,GROUP5), /* CAN0 COMFRX (Common FIFO receive interrupt) */
            [22] = BSP_PRV_VECT_ENUM(EVENT_CAN_GLERR,GROUP6), /* CAN GLERR (Global error) */
            [23] = BSP_PRV_VECT_ENUM(EVENT_CAN_RXF,GROUP7), /* CAN RXF (Global receive FIFO interrupt) */
            [24] = BSP_PRV_VECT_ENUM(EVENT_SCI5_RXI,GROUP0), /* SCI5 RXI (Receive data full) */
            [25] = BSP_PRV_VECT_ENUM(EVENT_SCI5_TXI,GROUP1), /* SCI5 TXI (Transmit data empty) */
            [26] = BSP_PRV_VECT_ENUM(EVENT_SCI5_TEI,GROUP2), /* SCI5 TEI (Transmit end) */
            [27] = BSP_PRV_VECT_ENUM(EVENT_SCI5_ERI,GROUP3), /* SCI5 ERI (Receive error) */
            [28] = BSP_PRV_VECT_ENUM(EVENT_USBFS_INT,GROUP4), /* USBFS INT (USBFS interrupt) */
            [29] = BSP_PRV_VECT_ENUM(EVENT_USBFS_RESUME,GROUP5), /* USBFS RESUME (USBFS resume interrupt) */
            [30] = BSP_PRV_VECT_ENUM(EVENT_USBFS_FIFO_0,GROUP6), /* USBFS FIFO 0 (DMA/DTC transfer request 0) */
            [31] = BSP_PRV_VECT_ENUM(EVENT_USBFS_FIFO_1,GROUP7), /* USBFS FIFO 1 (DMA/DTC transfer request 1) */
        };
        #endif
        #endif