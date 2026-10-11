/**
  **************************************************************************
  * @file     main.c
  * @brief    main program
  **************************************************************************
  *
  * Copyright (c) 2025, Artery Technology, All rights reserved.
  *
  * The software Board Support Package (BSP) that is made available to
  * download from Artery official website is the copyrighted work of Artery.
  * Artery authorizes customers to use, copy, and distribute the BSP
  * software and its related documentation for the purpose of design and
  * development in conjunction with Artery microcontrollers. Use of the
  * software is governed by this copyright notice and the following disclaimer.
  *
  * THIS SOFTWARE IS PROVIDED ON "AS IS" BASIS WITHOUT WARRANTIES,
  * GUARANTEES OR REPRESENTATIONS OF ANY KIND. ARTERY EXPRESSLY DISCLAIMS,
  * TO THE FULLEST EXTENT PERMITTED BY LAW, ALL EXPRESS, IMPLIED OR
  * STATUTORY OR OTHER WARRANTIES, GUARANTEES OR REPRESENTATIONS,
  * INCLUDING BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE, OR NON-INFRINGEMENT.
  *
  **************************************************************************
  */

#include "at32f45x_board.h"
#include "at32f45x_clock.h"
#include "bsp_can.h"
#include "led.h"
#include "bsp_usb.h"
#include "gs_usb_protocol.h"
#include "can_ring.h"

#include <string.h>

/* External USB core structure from bsp_usb.c */
extern otg_core_type otg_core_struct;

/** @addtogroup AT32F456_periph_examples
  * @{
  */

/** @addtogroup 456_QSPI_xip_port_read_write_sram QSPI_xip_port_read_write_sram
  * @{
  */

extern void system_tick_config(void);

int main(void)
{
    system_clock_config();
    at32_board_init();

    system_tick_config();

    usb_init();
    gs_usb_init();

    can_init();
    led_init();

    /* Mode indicator (PA15): steady on in gs_usb mode */
    led_mode_indicator(LED_APPMODE_GS_USB);

    while(1)
    {
        /* ====== USB <-> CAN bridge processing ====== */
        /* Process bulk OUT: handle CAN frames from host */
        gs_usb_process_bulk_out(&otg_core_struct.dev);

        /* Move frames from ISR queue to USB queue (SRAM ring; frames are counted as dropped when full) */
        can_ring_process();

        /* Interleave IN/OUT calls for fair scheduling in both directions.
         * OUT processing is interspersed between IN calls so the IN while-loop
         * does not delay OUT handling. */
        gs_usb_process_bulk_in(&otg_core_struct.dev);
        gs_usb_process_bulk_out(&otg_core_struct.dev);
        gs_usb_process_bulk_in(&otg_core_struct.dev);
        gs_usb_process_bulk_in(&otg_core_struct.dev);

#if (CAN_USE_ISR == 0)
        CAN1_RX_IRQHandler();
        CAN1_ERR_IRQHandler();

        CAN2_RX_IRQHandler();
        CAN2_ERR_IRQHandler();

        CAN3_RX_IRQHandler();
        CAN3_ERR_IRQHandler();
#endif

        /* Process pending error frames from ISR (drain per-channel ring buffer,
         * push to RX queue with rate limiting). Must be called BEFORE
         * can_handle_busoff_recovery() so BUSOFF frames are sent before
         * the corresponding RESTARTED frame. */
        can_process_pending_error_frames();

        /* Bus-off recovery: detect auto-recovery (128×11 recessive bits) or
         * force manual reset after timeout. Pushes CAN_ERR_RESTARTED frame
         * to notify host when recovery completes. Must run in main loop
         * context (not ISR) because can_stub_push_error_frame touches the
         * error frame ring buffer shared with the ISR. */
        can_handle_busoff_recovery();

        /* Mode indicator (PA15): steady on in gs_usb mode */
        led_mode_indicator(LED_APPMODE_GS_USB);
        led_poll();
    }
}


/**
  * @}
  */

/**
  * @}
  */
