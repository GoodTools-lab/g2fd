/*
 * G2FD - three-channel CANFD gs_usb adapter firmware (AT32F456)
 * Copyright (c) 2026 G2FD contributors
 * SPDX-License-Identifier: MIT
 */

/**
  **************************************************************************
  * @file     gs_usb_protocol.c
  * @brief    gs_usb (candleLight) protocol handler implementation
  *           Based on candleLight usbd_gs_can.c architecture
  *           Matches original candleLight USBD_GS_CAN_Config_Request and
  *           USBD_GS_CAN_EP0_RxReady flow exactly.
  **************************************************************************
  */

#include "gs_usb_protocol.h"
#include "bsp_can.h"
#include "winusb_class.h"
#include "can_ring.h"
#include "gsusb_arena.h"   /* queue/buffer storage reuses gtl_data_buffer */
#include "at32f45x_board.h"
#include <string.h>

/* ====== Compile-time feature flags (matching candleLight config.h) ====== */
#define CONFIG_CANFD        1  /* always enabled on AT32F456 FDCAN board */
#define CONFIG_TERMINATION  0  /* not supported */
#define CONFIG_CAN_FILTER   0  /* not supported */
#define IS_ENABLED(cfg)     (cfg)

/* ====== Device configuration constants (matching candleLight) ====== */
static const struct gs_device_config gs_usb_dconf __attribute__((aligned(4))) = {
    .reserved1 = 0,
    .reserved2 = 0,
    .reserved3 = 0,
    .icount = GS_USB_ACTIVE_CHANNELS - 1,
    .sw_version = 2,
    .hw_version = 1,
};

/*
 * AT32F456 FDCAN bit timing constants:
 *   CAN kernel clock = PCLK = 160MHz (APB1 div=1, SCLK=160MHz)
 *   gs_usb brp → AT32 bittime_div (= NBTP.NBRP + 1)
 *   time_quantum = fclk_can / brp = 160MHz / brp
 *
 * - Nominal (arbitration) registers:
 *   NBRP:   9 bits, 1..512   → protocol brp_max = 512
 *   NTSEG1: 8 bits, 1..256   → protocol tseg1_max = 255 (register = tseg1+1)
 *   NTSEG2: 7 bits, 1..128   → protocol tseg2_max = 128
 *   NSJW:   7 bits, 1..128   → protocol sjw_max = 128
 * - Data phase registers (AT32F456 FDCAN data phase has narrower limits):
 *   DBRP:   5 bits, 1..32    → protocol brp_max = 32
 *   DTSEG1: 5 bits, 1..32    → protocol tseg1_max = 31 (register = tseg1+1)
 *   DTSEG2: 4 bits, 1..16    → protocol tseg2_max = 16
 *   DSJW:   4 bits, 1..16    → protocol sjw_max = 16
 */
static const struct gs_device_bt_const gs_usb_bt_const __attribute__((aligned(4))) = {
    .feature = GS_USB_DEV_FEATURES,
    .fclk_can = 160000000,
    .btc = {
        .tseg1_min = 1,
        .tseg1_max = 255,
        .tseg2_min = 1,
        .tseg2_max = 128,
        .sjw_max = 128,
        .brp_min = 1,
        .brp_max = 512,
        .brp_inc = 1,
    },
};

static const struct gs_device_bt_const_extended gs_usb_bt_const_ext __attribute__((aligned(4))) = {
    .feature = GS_USB_DEV_FEATURES,
    .fclk_can = 160000000,
    .btc = {
        .tseg1_min = 1,
        .tseg1_max = 255,
        .tseg2_min = 1,
        .tseg2_max = 128,
        .sjw_max = 128,
        .brp_min = 1,
        .brp_max = 512,
        .brp_inc = 1,
    },
    .dbtc = {
        .tseg1_min = 1,
        .tseg1_max = 31,
        .tseg2_min = 1,
        .tseg2_max = 16,
        .sjw_max = 16,
        .brp_min = 1,
        .brp_max = 32,
        .brp_inc = 1,
    },
};

/* ====== Runtime state ====== */
static gs_usb_state_t gs_usb_state;

static union gs_usb_ep0_buf gs_usb_ep0 __attribute__((aligned(4)));

static usb_setup_type gs_usb_last_setup;

static uint8_t gs_usb_req_pending = 0;

/* ====== Bulk OUT (TX direction) state ====== */
/* TX ring queue: replaces the former 2-slot double buffer; capacity 128 frames (10KB).
 * When the CAN STB is full, frames wait in the queue instead of being
 * overwritten and lost as with the double buffer.
 * - gs_usb_bulk_rx_notify() accumulates USB OUT packets and pushes complete
 *   frames into the ring head
 * - gs_usb_process_bulk_out() takes frames from the tail and transmits them
 *   to the CAN STB
 * The storage objects (gs_usb_tx_ring / frame_len / accum_buf /
 * bulk_tx_buf_a/b) are placed per gsusb_arena.h, reusing the gtl_data_buffer
 * address space, and are reset via gs_usb_init() on the logging→gs_usb
 * mode switch */
#ifndef GSUSB_USE_ARENA
/* Test-build (GTL_BUF_SIZE=0x400) fallback: the arena cannot hold these,
 * so use plain static arrays. Depth/geometry macros are provided centrally
 * by gsusb_arena.h */
static uint8_t gs_usb_tx_ring[GS_USB_TX_RING_SIZE][GS_USB_TX_RING_SLOT_SIZE];
static volatile uint16_t gs_usb_tx_ring_frame_len[GS_USB_TX_RING_SIZE]; /* actual length of each frame */
static uint8_t gs_usb_rx_accum_buf[GSUSB_ACCUM_SIZE];
static uint8_t gs_usb_bulk_tx_buf_a[GS_USB_TX_BUF_SIZE] __attribute__((aligned(4)));
static uint8_t gs_usb_bulk_tx_buf_b[GS_USB_TX_BUF_SIZE] __attribute__((aligned(4)));
#endif
static volatile uint16_t gs_usb_tx_ring_head = 0;       /* write position (notify side) */
static volatile uint16_t gs_usb_tx_ring_tail = 0;       /* read position (main loop) */
static volatile uint16_t gs_usb_tx_ring_count = 0;      /* frames queued */

/* USB OUT packet accumulation offset (for single-frame reassembly; USB FS
 * 64B packets need multiple packets to rebuild 76B/80B frames) */
static volatile uint16_t gs_usb_rx_accum_offset = 0;

/* Ping-pong state:
 * filling_buf: main loop fills this with frames from RX queue
 * active_buf:  currently being transmitted by USB (NULL if idle)
 * When USB TX completes and filling_buf has data, swap and send immediately */
static uint8_t *gs_usb_filling_buf = NULL;
static uint8_t *gs_usb_active_buf  = NULL;
static uint16_t gs_usb_filling_len = 0;      /* bytes filled in filling_buf */
static uint8_t  gs_usb_filling_count = 0;    /* frame count in filling_buf */


/* Global SOF timestamp for external access */
uint32_t gs_usb_sof_timestamp_us = 0;

/* ====== Bit timing validation (matching candleLight can_check_bittiming_ok) ====== */
static uint8_t gs_usb_check_bittiming_ok(const struct can_bittiming_const *btc,
                                         const struct gs_device_bittiming *timing)
{
    uint32_t tseg1 = timing->prop_seg + timing->phase_seg1;

    if (tseg1 < btc->tseg1_min || tseg1 > btc->tseg1_max ||
        timing->phase_seg2 < btc->tseg2_min || timing->phase_seg2 > btc->tseg2_max ||
        timing->sjw > btc->sjw_max ||
        timing->brp < btc->brp_min || timing->brp > btc->brp_max) {
        return 0;
    }
    return 1;
}

/* ====== Public API ====== */
void gs_usb_init(void)
{
    memset(&gs_usb_state, 0, sizeof(gs_usb_state));
    memset(&gs_usb_ep0, 0, sizeof(gs_usb_ep0));
    gs_usb_req_pending = 0;
    /* Initialize the TX ring queue */
    gs_usb_tx_ring_head = 0;
    gs_usb_tx_ring_tail = 0;
    gs_usb_tx_ring_count = 0;
    gs_usb_rx_accum_offset = 0;
    gs_usb_sof_timestamp_us = 0;
    gs_usb_filling_buf = gs_usb_bulk_tx_buf_a;
    gs_usb_active_buf = NULL;
    gs_usb_filling_len = 0;
    gs_usb_filling_count = 0;
}

void gs_usb_apply_bittiming(uint8_t ch, const struct gs_device_bittiming *timing)
{
    can_bittime_type bittime;
    memset(&bittime, 0, sizeof(can_bittime_type));

    if (ch >= GS_USB_ACTIVE_CHANNELS) {
        return;
    }

    /* AT32 CAN uses a SHARED prescaler for arbitration and data phases.
     * When nominal brp changes, FD data parameters must be scaled to
     * maintain the same data bitrate.
     * old_data_bitrate = fclk / (old_brp * (old_fd_bts1 + old_fd_bts2))
     * new_data_bitrate = fclk / (new_brp * (new_fd_bts1 + new_fd_bts2))
     * To keep same bitrate: new_fd_total = old_fd_total * old_brp / new_brp
     */
    uint32_t old_brp = gs_usb_state.current_timing[ch].brp;
    uint32_t new_brp = timing->brp;

    /* If old_brp is 0 (initial state, no prior BITTIMING command),
     * use the hardware's current bittime_div as the effective old brp.
     * This ensures FD parameters are correctly scaled when the first
     * BITTIMING changes the prescaler from the power-on default. */
    if (old_brp == 0) {
        old_brp = g_can_config[ch].bittime_div;
    }

    memcpy(&gs_usb_state.current_timing[ch], timing, sizeof(struct gs_device_bittiming));

    /* AT32 CAN bittime_set() does: presc=bittime_div-1, ac_seg_1=ac_bts1_size-2,
     * ac_seg_2=ac_bts2_size-1, ac_sjw=ac_rsaw_size-1
     * Actual bitrate = fclk / (bittime_div * (ac_bts1_size + ac_bts2_size))
     * gs_usb: bitrate = fclk / (brp * (1 + prop_seg + phase_seg1 + phase_seg2))
     * Mapping: bittime_div=brp, ac_bts1_size=1+prop_seg+phase_seg1, ac_bts2_size=phase_seg2, ac_rsaw_size=sjw
     */
    bittime.bittime_div = (uint8_t)timing->brp;
    bittime.ac_bts1_size = (uint16_t)(1 + timing->prop_seg + timing->phase_seg1);
    bittime.ac_bts2_size = (uint8_t)timing->phase_seg2;
    bittime.ac_rsaw_size = (uint8_t)timing->sjw;

    /* Clamp arbitration sample point to max 85% for reliable BRS frame reception.
     * AT32F456 CAN FD controller needs sufficient phase_seg2 time after the
     * sample point to prepare for the arbitration→data phase rate switch.
     * High sample point (e.g., 88%) leaves too little time, causing bus errors
     * on BRS frame RX. Move excess bts1 to bts2 to lower the sample point. */
    {
        uint32_t total_tq = bittime.ac_bts1_size + bittime.ac_bts2_size;
        uint32_t max_bts1 = (total_tq * 85) / 100;  /* 85% sample point limit */
        if (bittime.ac_bts1_size > max_bts1) {
            uint32_t excess = bittime.ac_bts1_size - max_bts1;
            bittime.ac_bts1_size = (uint16_t)max_bts1;
            bittime.ac_bts2_size = (uint8_t)(bittime.ac_bts2_size + excess);
            /* Ensure sjw does not exceed phase_seg2 */
            if (bittime.ac_rsaw_size > bittime.ac_bts2_size) {
                bittime.ac_rsaw_size = bittime.ac_bts2_size;
            }
        }
    }

    /* Scale FD data phase parameters for the new prescaler */
    if (old_brp > 0 && old_brp != new_brp) {
        uint32_t old_fd_total = g_can_config[ch].fd_bts1_size + g_can_config[ch].fd_bts2_size;
        uint32_t new_fd_total = (old_fd_total * old_brp) / new_brp;
        /* Preserve sample point ratio */
        uint32_t new_fd_bts1 = (g_can_config[ch].fd_bts1_size * new_fd_total) / old_fd_total;
        /* Clamp bts1 to AT32 FD register limit (fd_seg_1: 5 bits, max=33) */
        if (new_fd_bts1 < 3) new_fd_bts1 = 3;
        if (new_fd_bts1 > 33) new_fd_bts1 = 33;
        /* Calculate bts2 from remaining tq to maintain total */
        uint32_t new_fd_bts2 = new_fd_total - new_fd_bts1;
        /* Clamp bts2 to AT32 FD register limit (fd_seg_2: 4 bits, max=16) */
        if (new_fd_bts2 < 2) {
            new_fd_bts2 = 2;
            new_fd_bts1 = new_fd_total - new_fd_bts2;
            if (new_fd_bts1 > 33) new_fd_bts1 = 33;
        }
        if (new_fd_bts2 > 16) new_fd_bts2 = 16;
        bittime.fd_bts1_size = (uint16_t)new_fd_bts1;
        bittime.fd_bts2_size = (uint8_t)new_fd_bts2;
        bittime.fd_rsaw_size = g_can_config[ch].fd_rsaw_size;
        bittime.fd_ssp_offset = (uint8_t)(new_fd_bts1 + 1);
    } else {
        /* Keep existing FD parameters, pass them to can_set_bittime */
        bittime.fd_bts1_size = g_can_config[ch].fd_bts1_size;
        bittime.fd_bts2_size = g_can_config[ch].fd_bts2_size;
        bittime.fd_rsaw_size = g_can_config[ch].fd_rsaw_size;
        bittime.fd_ssp_offset = g_can_config[ch].fd_ssp_offset;
    }

    can_set_bittime(ch, &bittime);
}

void gs_usb_apply_data_bittiming(uint8_t ch, const struct gs_device_bittiming *timing)
{
    can_bittime_type bittime;
    const struct gs_device_bittiming *nom = &gs_usb_state.current_timing[ch];
    memset(&bittime, 0, sizeof(can_bittime_type));

    if (ch >= GS_USB_ACTIVE_CHANNELS) {
        return;
    }

    memcpy(&gs_usb_state.current_data_timing[ch], timing, sizeof(struct gs_device_bittiming));

    /* AT32 CAN uses a SHARED prescaler for arbitration and data phases.
     * The prescaler is set by the nominal bittiming (nom->brp).
     * Host may send a different brp for data phase, so we must scale:
     *   host_data_bitrate = fclk / (data_brp * data_total_tq)
     *   at32_data_bitrate = fclk / (nom_brp * fd_total_tq)
     *   fd_total_tq = (data_brp * data_total_tq) / nom_brp
     */
    uint32_t data_total_tq = 1 + timing->prop_seg + timing->phase_seg1 + timing->phase_seg2;
    uint32_t data_tseg1 = 1 + timing->prop_seg + timing->phase_seg1;
    uint32_t data_tseg2 __attribute__((unused)) = timing->phase_seg2;

    /* Scale data tq to match the shared (nominal) prescaler (rounded to nearest,
     * truncation loses up to 1 tq and directly shifts the data bitrate) */
    uint32_t fd_total_tq = (timing->brp * data_total_tq + nom->brp / 2) / nom->brp;
    uint32_t fd_bts1 = (data_tseg1 * fd_total_tq + data_total_tq / 2) / data_total_tq;

    /* Clamp bts1 to AT32 FD register limit FIRST, then calculate bts2 */
    if (fd_bts1 < 3) fd_bts1 = 3;
    if (fd_bts1 > 33) fd_bts1 = 33;
    uint32_t fd_bts2 = fd_total_tq - fd_bts1;  /* Calculate AFTER clamping */
    if (fd_bts2 < 2) fd_bts2 = 2;
    if (fd_bts2 > 16) fd_bts2 = 16;

    bittime.bittime_div = (uint8_t)nom->brp;
    bittime.ac_bts1_size = (uint16_t)(1 + nom->prop_seg + nom->phase_seg1);
    bittime.ac_bts2_size = (uint8_t)nom->phase_seg2;
    bittime.ac_rsaw_size = (uint8_t)nom->sjw;

    /* Clamp arbitration sample point to max 85% (same as gs_usb_apply_bittiming) */
    {
        uint32_t total_tq = bittime.ac_bts1_size + bittime.ac_bts2_size;
        uint32_t max_bts1 = (total_tq * 85) / 100;
        if (bittime.ac_bts1_size > max_bts1) {
            uint32_t excess = bittime.ac_bts1_size - max_bts1;
            bittime.ac_bts1_size = (uint16_t)max_bts1;
            bittime.ac_bts2_size = (uint8_t)(bittime.ac_bts2_size + excess);
            if (bittime.ac_rsaw_size > bittime.ac_bts2_size) {
                bittime.ac_rsaw_size = bittime.ac_bts2_size;
            }
        }
    }
    bittime.fd_bts1_size = (uint16_t)fd_bts1;
    bittime.fd_bts2_size = (uint8_t)fd_bts2;
    bittime.fd_rsaw_size = (uint8_t)timing->sjw;
    bittime.fd_ssp_offset = (uint8_t)(fd_bts1 + 1);  /* SSP = fd_bts1_size + 1 */

    can_set_fd_bittime(ch, &bittime);
}

void gs_usb_can_enable(uint8_t ch, uint32_t feature)
{
    if (ch >= GS_USB_ACTIVE_CHANNELS) {
        return;
    }

    gs_usb_state.feature[ch] = feature;
    gs_usb_state.can_enabled[ch] = 1;

    /* Set BERR_REPORTING flag so ERR ISR knows whether to push error frames.
     * The host sets GS_CAN_MODE_BERR_REPORTING in gs_device_mode.flags when it
     * wants SocketCAN-style error frames. ISR checks g_can_berr_reporting[]
     * before pushing to avoid flooding the RX queue when the host doesn't want them. */
    g_can_berr_reporting[ch] = (feature & GS_CAN_MODE_BERR_REPORTING) ? 1 : 0;

    can_enable(ch);
}

void gs_usb_can_disable(uint8_t ch)
{
    if (ch >= GS_USB_ACTIVE_CHANNELS) {
        return;
    }

    gs_usb_state.can_enabled[ch] = 0;
    gs_usb_state.feature[ch] = 0;
    g_can_berr_reporting[ch] = 0;  /* stop error frame generation */
    can_disable(ch);
}

/*
 * gs_usb_handle_setup - matches candleLight USBD_GS_CAN_Config_Request flow:
 *   1. Channel validation (skip for per-device requests)
 *   2. Feature checks (CONFIG_CANFD/CONFIG_TERMINATION/CONFIG_CAN_FILTER)
 *   3. Switch on bRequest to determine len and src
 *   4. wLength validation
 *   5. Separate Host→Device (OUT) vs Device→Host (IN) handling
 */
uint8_t gs_usb_handle_setup(void *udev, usb_setup_type *setup)
{
    usbd_core_type *pudev = (usbd_core_type *)udev;
    const void *src = NULL;
    uint16_t len = 0;

    gs_usb_last_setup = *setup;
    gs_usb_req_pending = 1;

    /*
     * For all "per device" USB control messages
     * (GS_USB_BREQ_HOST_FORMAT and GS_USB_BREQ_DEVICE_CONFIG) the
     * Linux gs_usb driver uses a setup->wValue = 1.
     *
     * All other control messages are "per channel" and specify the
     * channel number in setup->wValue. So check setup->wValue for valid
     * CAN channel.
     */
    if (!(setup->bRequest == GS_USB_BREQ_HOST_FORMAT ||
          setup->bRequest == GS_USB_BREQ_DEVICE_CONFIG)) {
        if (setup->wValue >= GS_USB_ACTIVE_CHANNELS) {
            goto out_fail;
        }
        gs_usb_state.current_channel = (uint8_t)setup->wValue;
    }

    /* Feature checks: STALL requests for unsupported features */
    if (!IS_ENABLED(CONFIG_CANFD)) {
        switch (setup->bRequest) {
            case GS_USB_BREQ_DATA_BITTIMING:
            case GS_USB_BREQ_BT_CONST_EXT:
                goto out_fail;
            default:
                break;
        }
    }

    if (!IS_ENABLED(CONFIG_TERMINATION)) {
        switch (setup->bRequest) {
            case GS_USB_BREQ_SET_TERMINATION:
            case GS_USB_BREQ_GET_TERMINATION:
                goto out_fail;
            default:
                break;
        }
    }

    if (!IS_ENABLED(CONFIG_CAN_FILTER)) {
        switch (setup->bRequest) {
            case GS_USB_BREQ_SET_FILTER:
            case GS_USB_BREQ_GET_FILTER:
                goto out_fail;
            default:
                break;
        }
    }

    /* Step 1: Determine len and src for each request */
    switch (setup->bRequest) {
        /* Host -> Device (OUT) */
        case GS_USB_BREQ_HOST_FORMAT:
            len = sizeof(gs_usb_ep0.host_config);
            break;
        case GS_USB_BREQ_BITTIMING:
            len = sizeof(gs_usb_ep0.bittiming);
            break;
        case GS_USB_BREQ_MODE:
            len = sizeof(gs_usb_ep0.mode);
            break;
        case GS_USB_BREQ_IDENTIFY:
            len = sizeof(gs_usb_ep0.identify_mode);
            break;
        case GS_USB_BREQ_DATA_BITTIMING:
            len = sizeof(gs_usb_ep0.data_bittiming);
            break;
        case GS_USB_BREQ_SET_TERMINATION:
            len = sizeof(uint32_t); /* gs_device_termination_state */
            break;
        case GS_USB_BREQ_SET_FILTER:
            len = sizeof(gs_usb_ep0.raw); /* filter struct */
            break;

        /* Device -> Host (IN) */
        case GS_USB_BREQ_BT_CONST:
            src = &gs_usb_bt_const;
            len = sizeof(gs_usb_bt_const);
            break;
        case GS_USB_BREQ_DEVICE_CONFIG:
            src = &gs_usb_dconf;
            len = sizeof(gs_usb_dconf);
            break;
        case GS_USB_BREQ_TIMESTAMP:
            src = &gs_usb_state.sof_timestamp_us;
            len = sizeof(gs_usb_state.sof_timestamp_us);
            break;
        case GS_USB_BREQ_BT_CONST_EXT:
            src = &gs_usb_bt_const_ext;
            len = sizeof(gs_usb_bt_const_ext);
            break;
        case GS_USB_BREQ_GET_STATE: {
            /* Return CAN controller state (state enum + RX/TX error counters).
             * Uses a static buffer because usbd_ctrl_send is asynchronous and
             * the data must persist until the transfer completes. */
            static struct gs_device_state state_buf;
            can_get_device_state(gs_usb_state.current_channel, &state_buf);
            src = &state_buf;
            len = sizeof(state_buf);
            break;
        }
        case GS_USB_BREQ_GET_TERMINATION: {
            static const uint32_t term_unsupported = GS_CAN_TERMINATION_UNSUPPORTED;
            src = &term_unsupported;
            len = sizeof(term_unsupported);
            break;
        }
        case GS_USB_BREQ_GET_FILTER:
            /* filter info not supported */
            goto out_fail;

        default:
            goto out_fail;
    }

    /* Step 2: wLength validation */
    if (setup->wLength < len) {
        goto out_fail;
    }

    /* Step 3: Separate Host→Device (OUT) vs Device→Host (IN) */
    switch (setup->bRequest) {
        /* Host -> Device: prepare to receive data */
        case GS_USB_BREQ_HOST_FORMAT:
        case GS_USB_BREQ_BITTIMING:
        case GS_USB_BREQ_MODE:
        case GS_USB_BREQ_IDENTIFY:
        case GS_USB_BREQ_DATA_BITTIMING:
        case GS_USB_BREQ_SET_TERMINATION:
        case GS_USB_BREQ_SET_FILTER:
            if (setup->wLength > sizeof(gs_usb_ep0)) {
                goto out_fail;
            }
            /* candleLight: USBD_CtlPrepareRx(pdev, ep0.buf, req->wLength) */
            usbd_ctrl_recv(pudev, gs_usb_ep0.raw, setup->wLength);
            break;

        /* Device -> Host: send data */
        case GS_USB_BREQ_BT_CONST:
        case GS_USB_BREQ_DEVICE_CONFIG:
        case GS_USB_BREQ_TIMESTAMP:
        case GS_USB_BREQ_BT_CONST_EXT:
        case GS_USB_BREQ_GET_STATE:
        case GS_USB_BREQ_GET_TERMINATION:
        case GS_USB_BREQ_GET_FILTER:
            /* candleLight: USBD_CtlSendData(pdev, (uint8_t *)src, len) */
            usbd_ctrl_send(pudev, (uint8_t *)src, len);
            break;

        default:
            goto out_fail;
    }

    return USB_OK;

out_fail:
    usbd_ctrl_unsupport(pudev);
    gs_usb_req_pending = 0;
    return USB_FAIL;
}

/*
 * gs_usb_handle_ep0_rx - matches candleLight USBD_GS_CAN_EP0_RxReady flow:
 *   1. Get channel from saved state
 *   2. Feature checks
 *   3. Switch on bRequest to process received data
 *   4. Mark request as handled (bRequest = 0xFF)
 */
uint8_t gs_usb_handle_ep0_rx(void *udev)
{
    usb_setup_type *setup = &gs_usb_last_setup;
    uint8_t ch = gs_usb_state.current_channel;
    gs_usb_req_pending = 0;

    /*
     * The control messages GS_USB_BREQ_HOST_FORMAT and
     * GS_USB_BREQ_DEVICE_CONFIG are per device, all others are per
     * channel. The validity of setup->wValue has been checked in
     * gs_usb_handle_setup() already.
     */
    if (!(setup->bRequest == GS_USB_BREQ_HOST_FORMAT ||
          setup->bRequest == GS_USB_BREQ_DEVICE_CONFIG)) {
        if (ch >= GS_USB_ACTIVE_CHANNELS) {
            return USB_FAIL;
        }
    }

    /* Feature checks: STALL requests for unsupported features */
    if (!IS_ENABLED(CONFIG_CANFD)) {
        switch (setup->bRequest) {
            case GS_USB_BREQ_DATA_BITTIMING:
                goto out_fail;
            default:
                break;
        }
    }

    if (!IS_ENABLED(CONFIG_TERMINATION)) {
        switch (setup->bRequest) {
            case GS_USB_BREQ_SET_TERMINATION:
            case GS_USB_BREQ_GET_TERMINATION:
                goto out_fail;
            default:
                break;
        }
    }

    if (!IS_ENABLED(CONFIG_CAN_FILTER)) {
        switch (setup->bRequest) {
            case GS_USB_BREQ_SET_FILTER:
                goto out_fail;
            default:
                break;
        }
    }

    switch (setup->bRequest) {
        case GS_USB_BREQ_HOST_FORMAT:
            /*
             * The firmware on the original USB2CAN by Geschwister Schneider
             * Technologie Entwicklungs- und Vertriebs UG exchanges all data
             * between the host and the device in host byte order. This is done
             * with the struct gs_host_config::byte_order member, which is sent
             * first to indicate the desired byte order.
             *
             * The widely used open source firmware candleLight doesn't support
             * this feature and exchanges the data in little endian byte order.
             */
            break;

        case GS_USB_BREQ_BITTIMING: {
            const struct gs_device_bittiming *timing = &gs_usb_ep0.bittiming;

            if (!gs_usb_check_bittiming_ok(&gs_usb_bt_const.btc, timing))
                goto out_fail;

            gs_usb_apply_bittiming(ch, timing);
            break;
        }

        case GS_USB_BREQ_MODE: {
            const struct gs_device_mode *mode = &gs_usb_ep0.mode;
            if (mode->mode == GS_CAN_MODE_RESET) {
                /* Reset only the specified channel (Linux gs_usb.c gs_can_close/gs_cmd_reset) */
                gs_usb_can_disable(ch);
            } else if (mode->mode == GS_CAN_MODE_START) {
                /* Start only the specified channel with the requested flags
                 * (Linux gs_usb.c gs_can_open: dm.flags = flags, wValue=dev->channel) */
                gs_usb_can_enable(ch, mode->flags);
            }
            break;
        }

        case GS_USB_BREQ_IDENTIFY:
            /* Identify mode: LED identify sequence not implemented */
            break;

        case GS_USB_BREQ_DATA_BITTIMING: {
            const struct gs_device_bittiming *timing = &gs_usb_ep0.data_bittiming;

            if (!gs_usb_check_bittiming_ok(&gs_usb_bt_const_ext.dbtc, timing))
                goto out_fail;

            gs_usb_apply_data_bittiming(ch, timing);
            break;
        }

        case GS_USB_BREQ_SET_TERMINATION:
            /* Termination not supported */
            break;

        case GS_USB_BREQ_SET_FILTER:
            /* Filter not supported */
            break;

        default:
            break;
    }

    /* Mark as handled (matching candleLight: req->bRequest = 0xFF) */
    gs_usb_last_setup.bRequest = 0xFF;
    return USB_OK;

out_fail:
    return USB_FAIL;
}

uint8_t gs_usb_is_pending_request(void)
{
    return gs_usb_req_pending;
}

void gs_usb_update_sof_timestamp(uint32_t timestamp_us)
{
    gs_usb_state.sof_timestamp_us = timestamp_us;
    gs_usb_sof_timestamp_us = timestamp_us;
}

/* ====== Bulk endpoint frame processing ====== */

/**
  * @brief Process bulk OUT endpoint - send TX frames from ring queue to CAN bus
  *        Called from main loop. Drains gs_usb_tx_ring → can_stub_transmit().
  *        When the STB is full, frames are not consumed (kept at queue head)
  *        and retried on the next call, preventing frame loss.
  */
/* Maximum frames processed per bulk_out call.
 * Same rationale as the 8-iteration cap in bulk_in: prevents a TX burst
 * (e.g. draining after STB-full clears) from monopolizing the main loop and
 * preserves the RX drain cadence of can_ring_process
 * (128 frames @ 2000 fps = 64 ms). */
#define GS_USB_BULK_OUT_MAX_FRAMES  16

extern uint8_t usb_winusb_poll_bulk_out(void *udev);

void gs_usb_process_bulk_out(void *udev)
{
    struct gs_host_frame local_frame;  /* local copy, guards against ISR overwrite */
    uint8_t ch;
    uint8_t is_fd;
    uint16_t expected_size;
    uint8_t tx_ret;
    uint8_t processed = 0;

    /* Fix: consume USB OUT completion events in the main loop (replaces the
     * former in-ISR notify). At this point the OTG ISR has exited, so the
     * RXFLVL copy of the XferCompl packet from FIFO to g_rx_buff must already
     * be done and the [content, length] snapshot is consistent; the endpoint
     * stays NAKed during consumption and nothing new is written to g_rx_buff.
     * This eliminates both the content/length skew caused by XferCompl
     * arriving before the copy in the ISR (root cause of torn frames) and the
     * orphaned-frame loss when resume resets trans_len. */
    usb_winusb_poll_bulk_out(udev);

    /* Process all pending frames in the ring queue.
     * Break when the STB is full (can_stub_transmit returns 2); the frame
     * stays at the tail and is retried on the next call. */
    while (gs_usb_tx_ring_count > 0) {
        if (processed >= GS_USB_BULK_OUT_MAX_FRAMES) {
            break;  /* frames stay in the ring; resume next call so RX/IN paths get serviced */
        }
        processed++;
        /* Fix 1a: read frame data inside a critical section to guard against
         * the ISR's drop-oldest overwrite */
        __disable_irq();
        if (gs_usb_tx_ring_count == 0) {
            __enable_irq();
            break;
        }
        uint16_t read_idx = gs_usb_tx_ring_tail;
        uint16_t rx_len = gs_usb_tx_ring_frame_len[read_idx];
        /* Copy only the valid-length bytes to avoid reading stale tail bytes
         * in the slot (the old implementation always copied 80 bytes) */
        memcpy(&local_frame, gs_usb_tx_ring[read_idx],
               (rx_len > sizeof(struct gs_host_frame)) ? sizeof(struct gs_host_frame) : rx_len);
        __enable_irq();

        /* Validate channel */
        if (local_frame.channel >= GS_USB_ACTIVE_CHANNELS) {
            goto consume_and_continue;
        }

        /* Check if channel is enabled */
        if (!gs_usb_state.can_enabled[local_frame.channel]) {
            goto consume_and_continue;
        }

        /* Validate frame size - host-to-device frames do NOT include timestamp */
        ch = local_frame.channel;
        is_fd = (local_frame.flags & GS_CAN_FLAG_FD) ? 1 : 0;
        expected_size = is_fd ? GS_USB_FRAME_FD_NO_TS : GS_USB_FRAME_CLASSIC_NO_TS;
        if (rx_len < expected_size) {
            goto consume_and_continue;
        }

        /* Pass DLC (0-15) to can_stub_transmit, not byte count.
         * can_stub_transmit() internally converts DLC→byte count via can_fd_dlc2len() */
        tx_ret = can_stub_transmit(ch, local_frame.echo_id, local_frame.can_id, local_frame.can_dlc, local_frame.flags, local_frame.data);
        if (tx_ret == 2) {
            /* STB or echo queue full: do not consume the frame; leave the
             * loop and retry on the next call */
            break;
        }

    consume_and_continue:
        /* Send succeeded or other error: consume the frame and advance the tail */
        __disable_irq();
        gs_usb_tx_ring_tail = (gs_usb_tx_ring_tail + 1) % GS_USB_TX_RING_SIZE;
        gs_usb_tx_ring_count--;
        __enable_irq();
    }

    /* NAK flow-control recovery: after consuming frames the ring has room,
     * so re-arm the OUT endpoint.
     * g_usb_out_paused is set by class_out_handler when the ring is full.
     * Once resumed, the USB hardware stops NAKing and accepts the host's
     * retried OUT packets.
     * Key: even if the while loop broke early on STB-full, resume as long as
     * the ring has room, so the normal TX flow is unaffected (new frames keep
     * being accepted while the ring has room even if the STB is only
     * temporarily full). */
    if (g_usb_out_paused && gs_usb_tx_ring_count < GS_USB_TX_RING_SIZE) {
        usb_winusb_resume_out(udev);
    }
}

/**
  * @brief Process bulk IN endpoint - send CAN frame to host
  *        Called from main loop to check for TX echo or RX frames
  */
void gs_usb_process_bulk_in(void *udev)
{
    usbd_core_type *pudev = (usbd_core_type *)udev;
    struct gs_host_frame *frame;
    can_stub_tx_frame_t tx_echo;
    can_stub_rx_frame_t rx_frame;
    uint32_t feature;
    uint8_t loop_count = 0;  /* cap while iterations so the main loop is not monopolized */

    /* Completion-event watchdog with hardware-state classification
     * (ARMED/XFERC/IDLE); fixes link stalls caused by a lost tx_complete or
     * the host stopping IN polling */
    winusb_bulk_in_watchdog(udev);

    while (1) {
        if (++loop_count > 8) {
            break;  /* exit after 8 iterations so OUT and can_ring get serviced */
        }

        if (g_bulk_in_tx_completed > 1) {
            g_bulk_in_tx_completed = 1;
        }

        /* Step 1: If USB is idle and filling_buf has data, swap and send immediately */
        if (g_bulk_in_tx_completed && gs_usb_filling_len > 0) {
            /* Swap: filling_buf becomes active_buf, send it */
            uint8_t *tmp = gs_usb_filling_buf;
            gs_usb_filling_buf = gs_usb_active_buf ? gs_usb_active_buf :
                                 (tmp == gs_usb_bulk_tx_buf_a) ? gs_usb_bulk_tx_buf_b : gs_usb_bulk_tx_buf_a;
            gs_usb_active_buf = tmp;

            error_status ret = usb_winusb_send_data(pudev, gs_usb_active_buf, gs_usb_filling_len);
            if (ret != SUCCESS) {
                /* Restore buffer swap so frames are not lost and can retry */
                gs_usb_filling_buf = gs_usb_active_buf;
                gs_usb_active_buf = NULL;
            } else {
                gs_usb_filling_len = 0;
                gs_usb_filling_count = 0;
            }
        }

        if (!g_bulk_in_tx_completed) {
            /* USB busy: fill filling_buf with echo (priority) AND RX frames.
             * Echo frames are batched with RX frames to ensure they are sent
             * even when USB is continuously busy. This prevents echo queue
             * overflow which would cause TX frame loss (can_stub_transmit
             * returning 2 and gs_usb_tx_ring filling up). */
            while (can_stub_peek_tx_echo(&tx_echo)) {
                uint8_t is_fd_tx = (tx_echo.flags & GS_CAN_FLAG_FD) ? 1 : 0;
                feature = gs_usb_state.feature[tx_echo.channel];
                uint8_t has_ts = (feature & GS_CAN_FEATURE_HW_TIMESTAMP) ? 1 : 0;
                uint16_t need_len = is_fd_tx ?
                    (has_ts ? GS_USB_FRAME_FD_TS : GS_USB_FRAME_FD_NO_TS) :
                    (has_ts ? GS_USB_FRAME_CLASSIC_TS : GS_USB_FRAME_CLASSIC_NO_TS);

                /* Check frame count and space limits */
                if (gs_usb_filling_count >= USB_BULK_TX_MAX_FRAMES) break;
                if (gs_usb_filling_len + need_len > GS_USB_TX_BUF_SIZE) break;

                if (is_fd_tx) {
                    /* FD echo frame: data[64], timestamp at offset 76 */
                    frame = (struct gs_host_frame *)(gs_usb_filling_buf + gs_usb_filling_len);
                    frame->echo_id = tx_echo.ech0_id;
                    frame->can_id = tx_echo.can_id;
                    frame->can_dlc = can_fd_len2dlc(tx_echo.can_dlc);
                    frame->channel = tx_echo.channel;
                    frame->flags = tx_echo.flags;
                    frame->reserved = 0;
                    memcpy(frame->data, tx_echo.data, tx_echo.can_dlc > 64 ? 64 : tx_echo.can_dlc);
                    if (has_ts) {
                        frame->timestamp_us = tx_echo.timestamp_us;
                    }
                } else {
                    /* Classic echo frame: data[8], timestamp at offset 20 */
                    struct gs_host_frame_classic *cframe =
                        (struct gs_host_frame_classic *)(gs_usb_filling_buf + gs_usb_filling_len);
                    cframe->echo_id = tx_echo.ech0_id;
                    cframe->can_id = tx_echo.can_id;
                    cframe->can_dlc = tx_echo.can_dlc;
                    cframe->channel = tx_echo.channel;
                    cframe->flags = tx_echo.flags;
                    cframe->reserved = 0;
                    memcpy(cframe->data, tx_echo.data, tx_echo.can_dlc > 8 ? 8 : tx_echo.can_dlc);
                    if (has_ts) {
                        cframe->timestamp_us = tx_echo.timestamp_us;
                    }
                }

                gs_usb_filling_len += need_len;
                gs_usb_filling_count++;
                //echo_sent_this_call++;

                can_stub_consume_tx_echo();
            }

            /* Then fill remaining space with RX frames */
            while (can_stub_peek_rx_frame(&rx_frame)) {
                uint8_t is_fd_rx = (rx_frame.flags & GS_CAN_FLAG_FD) ? 1 : 0;
                feature = gs_usb_state.feature[rx_frame.channel];
                uint8_t has_ts = (feature & GS_CAN_FEATURE_HW_TIMESTAMP) ? 1 : 0;
                uint16_t need_len = is_fd_rx ?
                    (has_ts ? GS_USB_FRAME_FD_TS : GS_USB_FRAME_FD_NO_TS) :
                    (has_ts ? GS_USB_FRAME_CLASSIC_TS : GS_USB_FRAME_CLASSIC_NO_TS);

                /* Check frame count and space limits */
                if (gs_usb_filling_count >= USB_BULK_TX_MAX_FRAMES) break;
                if (gs_usb_filling_len + need_len > GS_USB_TX_BUF_SIZE) break;

                if (is_fd_rx) {
                    /* FD frame: data[64], timestamp at offset 76 */
                    frame = (struct gs_host_frame *)(gs_usb_filling_buf + gs_usb_filling_len);
                    frame->echo_id = 0xFFFFFFFF;
                    frame->can_id = rx_frame.can_id;
                    frame->can_dlc = can_fd_len2dlc(rx_frame.can_dlc);
                    frame->channel = rx_frame.channel;
                    frame->flags = rx_frame.flags;
                    frame->reserved = 0;
                    memcpy(frame->data, rx_frame.data, rx_frame.can_dlc > 64 ? 64 : rx_frame.can_dlc);
                    if (has_ts) {
                        frame->timestamp_us = rx_frame.timestamp_us;
                    }
                } else {
                    /* Classic frame: data[8], timestamp at offset 20 */
                    struct gs_host_frame_classic *cframe =
                        (struct gs_host_frame_classic *)(gs_usb_filling_buf + gs_usb_filling_len);
                    cframe->echo_id = 0xFFFFFFFF;
                    cframe->can_id = rx_frame.can_id;
                    cframe->can_dlc = rx_frame.can_dlc;
                    cframe->channel = rx_frame.channel;
                    cframe->flags = rx_frame.flags;
                    cframe->reserved = 0;
                    memcpy(cframe->data, rx_frame.data, rx_frame.can_dlc > 8 ? 8 : rx_frame.can_dlc);
                    if (has_ts) {
                        cframe->timestamp_us = rx_frame.timestamp_us;
                    }
                }

                gs_usb_filling_len += need_len;
                gs_usb_filling_count++;

                can_stub_consume_rx_frame();
            }
            return;
        }

        /* USB is idle, no pending data in filling_buf */

        /* Fill filling_buf with echo (priority) AND RX frames.
         * All frames are sent via ping-pong swap to ensure the transmitting
         * buffer (active_buf) is never overwritten by filling. Echoes filled
         * first to preserve TX confirmation priority. */
        while (can_stub_peek_tx_echo(&tx_echo)) {
            uint8_t is_fd_tx = (tx_echo.flags & GS_CAN_FLAG_FD) ? 1 : 0;
            feature = gs_usb_state.feature[tx_echo.channel];
            uint8_t has_ts = (feature & GS_CAN_FEATURE_HW_TIMESTAMP) ? 1 : 0;
            uint16_t need_len = is_fd_tx ?
                (has_ts ? GS_USB_FRAME_FD_TS : GS_USB_FRAME_FD_NO_TS) :
                (has_ts ? GS_USB_FRAME_CLASSIC_TS : GS_USB_FRAME_CLASSIC_NO_TS);

            /* Check frame count and space limits */
            if (gs_usb_filling_count >= USB_BULK_TX_MAX_FRAMES) break;
            if (gs_usb_filling_len + need_len > GS_USB_TX_BUF_SIZE) break;

            if (is_fd_tx) {
                /* FD echo frame: data[64], timestamp at offset 76 */
                frame = (struct gs_host_frame *)(gs_usb_filling_buf + gs_usb_filling_len);
                frame->echo_id = tx_echo.ech0_id;
                frame->can_id = tx_echo.can_id;
                frame->can_dlc = can_fd_len2dlc(tx_echo.can_dlc);
                frame->channel = tx_echo.channel;
                frame->flags = tx_echo.flags;
                frame->reserved = 0;
                memcpy(frame->data, tx_echo.data, tx_echo.can_dlc > 64 ? 64 : tx_echo.can_dlc);
                if (has_ts) {
                    frame->timestamp_us = tx_echo.timestamp_us;
                }
            } else {
                /* Classic echo frame: data[8], timestamp at offset 20 */
                struct gs_host_frame_classic *cframe =
                    (struct gs_host_frame_classic *)(gs_usb_filling_buf + gs_usb_filling_len);
                cframe->echo_id = tx_echo.ech0_id;
                cframe->can_id = tx_echo.can_id;
                cframe->can_dlc = tx_echo.can_dlc;
                cframe->channel = tx_echo.channel;
                cframe->flags = tx_echo.flags;
                cframe->reserved = 0;
                memcpy(cframe->data, tx_echo.data, tx_echo.can_dlc > 8 ? 8 : tx_echo.can_dlc);
                if (has_ts) {
                    cframe->timestamp_us = tx_echo.timestamp_us;
                }
            }

            gs_usb_filling_len += need_len;
            gs_usb_filling_count++;

            can_stub_consume_tx_echo();
        }

        /* Then fill remaining space with RX frames */
        while (can_stub_peek_rx_frame(&rx_frame)) {
            uint8_t is_fd_rx = (rx_frame.flags & GS_CAN_FLAG_FD) ? 1 : 0;
            feature = gs_usb_state.feature[rx_frame.channel];
            uint8_t has_ts = (feature & GS_CAN_FEATURE_HW_TIMESTAMP) ? 1 : 0;
            uint16_t need_len = is_fd_rx ?
                (has_ts ? GS_USB_FRAME_FD_TS : GS_USB_FRAME_FD_NO_TS) :
                (has_ts ? GS_USB_FRAME_CLASSIC_TS : GS_USB_FRAME_CLASSIC_NO_TS);

            /* Check frame count and space limits */
            if (gs_usb_filling_count >= USB_BULK_TX_MAX_FRAMES) break;
            if (gs_usb_filling_len + need_len > GS_USB_TX_BUF_SIZE) break;

            if (is_fd_rx) {
                /* FD frame: data[64], timestamp at offset 76 */
                frame = (struct gs_host_frame *)(gs_usb_filling_buf + gs_usb_filling_len);
                frame->echo_id = 0xFFFFFFFF;
                frame->can_id = rx_frame.can_id;
                frame->can_dlc = can_fd_len2dlc(rx_frame.can_dlc);
                frame->channel = rx_frame.channel;
                frame->flags = rx_frame.flags;
                frame->reserved = 0;
                memcpy(frame->data, rx_frame.data, rx_frame.can_dlc > 64 ? 64 : rx_frame.can_dlc);
                if (has_ts) {
                    frame->timestamp_us = rx_frame.timestamp_us;
                }
            } else {
                /* Classic frame: data[8], timestamp at offset 20 */
                struct gs_host_frame_classic *cframe =
                    (struct gs_host_frame_classic *)(gs_usb_filling_buf + gs_usb_filling_len);
                cframe->echo_id = 0xFFFFFFFF;
                cframe->can_id = rx_frame.can_id;
                cframe->can_dlc = rx_frame.can_dlc;
                cframe->channel = rx_frame.channel;
                cframe->flags = rx_frame.flags;
                cframe->reserved = 0;
                memcpy(cframe->data, rx_frame.data, rx_frame.can_dlc > 8 ? 8 : rx_frame.can_dlc);
                if (has_ts) {
                    cframe->timestamp_us = rx_frame.timestamp_us;
                }
            }

            gs_usb_filling_len += need_len;
            gs_usb_filling_count++;

            can_stub_consume_rx_frame();
        }

        /* If we filled any frames, loop back to Step 1 to send immediately */
        if (gs_usb_filling_len > 0) {
            continue;
        }

        return;
    }
}

/**
  * @brief Called from class_in_handler when TX completes
  */
void gs_usb_bulk_tx_complete(void)
{
}

/**
  * @brief Called from class_out_handler when RX data arrives
  * @note Supports multi-packet CAN FD frames (up to 76 bytes)
  *       Host-to-device frames do NOT include timestamp, even if
  *       HW_TIMESTAMP feature is enabled. Timestamp is only for
  *       device-to-host frames.
  *       Complete frames are reassembled in the accumulation buffer and pushed
  *       to the TX ring queue; the 128-frame capacity eliminates overwrite
  *       losses when the STB is full.
  */
/* Parse all complete frames from accum_buf and push them into the TX ring.
 * Returns 1 = ring full (bytes stay in accum; the OUT endpoint should pause,
 * and once the main loop drains the ring and resumes, leftover frames are
 * drained before the next packet arrives, so no byte-stream loss).
 * Unparsed trailing bytes are retained after each push (the host may
 * coalesce multiple frames into one USB packet); this fixes the byte-stream
 * desync of the old implementation, which cleared the offset after pushing a
 * frame and thereby lost the remaining bytes in the packet. */
static uint8_t gsusb_accum_drain(void)
{
    while (gs_usb_rx_accum_offset >= GS_USB_FRAME_CLASSIC_NO_TS) {
        struct gs_host_frame *frame = (struct gs_host_frame *)gs_usb_rx_accum_buf;

        if (frame->channel >= GS_USB_ACTIVE_CHANNELS) {
            gs_usb_rx_accum_offset = 0;   /* byte-stream out of sync; discard */
            return 0;
        }

        /* Host-to-device frames do NOT include timestamp */
        uint8_t is_fd = (frame->flags & GS_CAN_FLAG_FD) ? 1 : 0;
        uint16_t expected_size = is_fd ? GS_USB_FRAME_FD_NO_TS : GS_USB_FRAME_CLASSIC_NO_TS;

        if (gs_usb_rx_accum_offset < expected_size) {
            return 0;  /* frame not yet complete; wait for more data */
        }

        __disable_irq();
        if (gs_usb_tx_ring_count < GS_USB_TX_RING_SIZE) {
            /* Queue has room: write at head */
            uint16_t idx = gs_usb_tx_ring_head;
            memcpy(gs_usb_tx_ring[idx], gs_usb_rx_accum_buf, expected_size);
            gs_usb_tx_ring_frame_len[idx] = expected_size;
            gs_usb_tx_ring_head = (gs_usb_tx_ring_head + 1) % GS_USB_TX_RING_SIZE;
            gs_usb_tx_ring_count++;
            __enable_irq();
        } else {
            /* Ring full: keep the bytes in accum (including this complete
             * frame) instead of dropping them; return 1 to pause OUT.
             * The old code defensively dropped frames here (breaking frame
             * conservation); the new code is lossless. */
            __enable_irq();
            return 1;
        }

        /* Remove the parsed frame and keep residual bytes (no frame loss when
         * the host coalesces frames into one packet) */
        uint16_t remain = gs_usb_rx_accum_offset - expected_size;
        memmove(gs_usb_rx_accum_buf, gs_usb_rx_accum_buf + expected_size, remain);
        gs_usb_rx_accum_offset = remain;

        /* Ring full after push: end the drain and return 1 to trigger NAK
         * flow control */
        if (gs_usb_tx_ring_count >= GS_USB_TX_RING_SIZE) {
            return 1;
        }
    }
    return 0;
}

uint8_t gs_usb_bulk_rx_notify(const uint8_t *data, uint16_t len)
{
    /* Append this packet's data (accum capacity 160B >= complete frame 76B +
     * bytes held back when the ring was full last round; covers FD
     * multi-packet reassembly: USB FS 64B packets need several packets to
     * rebuild a 76B/80B frame).
     * Returns: 0 = ring has room (OUT endpoint may keep receiving)
     *          1 = ring full (OUT endpoint should NAK; resume after the ring
     *              is drained) */
    if (gs_usb_rx_accum_offset + len > GSUSB_ACCUM_SIZE) {
        gs_usb_rx_accum_offset = 0;   /* byte-stream out of sync; discard */
        return 0;
    }
    memcpy(gs_usb_rx_accum_buf + gs_usb_rx_accum_offset, data, len);
    gs_usb_rx_accum_offset += len;

    /* Parse all complete frames in accum (possibly several); when the ring is
     * full, keep the bytes and pause OUT. The main loop drains the ring and
     * resumes; after the next packet is appended, leftover frames are pushed
     * before new ones, so no frames are lost */
    return gsusb_accum_drain();
}
