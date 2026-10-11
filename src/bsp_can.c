/*
 * G2FD - three-channel CANFD gs_usb adapter firmware (AT32F456)
 * Copyright (c) 2026 G2FD contributors
 * SPDX-License-Identifier: MIT
 */

#include "at32f45x.h"
#include "at32f45x_board.h"
#include "led.h"
#include "bsp_can.h"
#include "bsp_can_init.h"
#include <string.h>
#include "gs_usb_protocol.h"
#include "can_ring.h"
#include "gsusb_arena.h"    /* Large gs_usb objects are placed in gtl_data_buffer (shared memory region) */

/* ====== Global operation mode ======
 * Open-source slim build: always gs_usb bridge mode (no offline logging mode). */
volatile can_operation_mode_t g_can_operation_mode = CAN_MODE_GS_USB;

/* Stub CAN state per channel */
typedef struct {
    uint8_t  enabled;
    uint8_t  mode;          /* 0=classic, 1=FD */
    uint32_t feature;
    uint32_t rx_cnt;
    uint32_t tx_cnt;
    uint32_t err_cnt;
} can_stub_channel_t;

static can_stub_channel_t g_can_stub[GS_USB_CAN_CHANNEL_MAX];

/* ====== Cumulative statistics counters (per channel) ====== */
volatile uint32_t g_can_hw_rx_total[3] = {0};  /* Total CAN frames received from hardware */
/* g_rx_queue_overflow, g_rx_queue_peak, g_rx_count are defined in can_ring.c */

/* CAN TX (device→bus) statistics per channel */
volatile uint32_t g_can_tx_success[3] = {0};   /* Successfully written to STB */
volatile uint32_t g_can_tx_fail[3] = {0};      /* Failed to write to STB (dropped) */
volatile uint32_t g_can_tx_stb_full[3] = {0};  /* STB was full when trying to send */
volatile uint32_t g_can_tx_echo_full[3] = {0}; /* Echo queue full (retryable) */
volatile uint32_t g_can_tx_complete[3] = {0};  /* CAN TX complete ISR count (frames actually transmitted on bus) */

/* Simulated transmit echo queue - three-pointer design
 * g_tx_write_idx:      write position of can_stub_transmit (on STB write)
 * g_tx_complete_idx:   advanced by the CAN TX ISR (on transmit completion)
 * g_tx_read_idx:       read position of gs_usb_process_bulk_in (echo return)
 * g_tx_count:          total frames (write - read)
 * g_tx_complete_count: completed-but-unreturned frames (complete - read)
 *
 * Echo return condition: g_tx_complete_count > 0 (completed frames not yet returned)
 * Queue-full condition:  g_tx_count >= CAN_STUB_TX_QUEUE_SIZE
 * The STB is a FIFO, so completion order equals write order; sequential advance
 * of complete_idx correctly associates echo_id
 * (CAN_STUB_TX_QUEUE_SIZE and g_tx_queue storage are defined in gsusb_arena.h, reusing gtl_data_buffer) */
#ifndef GSUSB_USE_ARENA
static can_stub_tx_frame_t g_tx_queue[CAN_STUB_TX_QUEUE_SIZE];
#endif
static volatile uint8_t g_tx_write_idx = 0;      /* written by can_stub_transmit */
static volatile uint8_t g_tx_complete_idx = 0;   /* advanced by CAN TX ISR */
static volatile uint8_t g_tx_read_idx = 0;       /* read for echo return */
static volatile uint8_t g_tx_count = 0;          /* total frames (write - read) */
static volatile uint8_t g_tx_complete_count = 0; /* completed count (complete - read) */
/* Per-channel TX complete event counters (ISR increments, main loop consumes).
 * Solves cross-channel interleaving: the ISR only increments the corresponding
 * channel's counter, and the main loop advances complete_idx on peek by matching
 * per channel. This avoids losing TX-complete events when the frame at the queue
 * head belongs to another channel. */
static volatile uint32_t g_tx_complete_pending[3] = {0};

/* External timestamp from USB SOF */
extern uint32_t gs_usb_sof_timestamp_us;

/* External counters from bsp_can_init.c */
extern volatile uint32_t can_rx_cnt;
extern volatile uint32_t can2_rx_cnt;
extern volatile uint32_t can3_rx_cnt;
extern volatile uint32_t can_tx_cnt;
extern volatile uint32_t can_err_cnt;
extern volatile uint32_t can2_err_cnt;
extern volatile uint32_t can3_err_cnt;
extern volatile uint32_t can3_err1_cnt;
extern volatile uint32_t can3_err2_cnt;
extern volatile uint32_t can3_err3_cnt;

/* ====== CAN Error Frame Reporting state (per channel) ====== */
/* [0]=CAN3 (gs_usb ch=0), [1]=CAN2 (gs_usb ch=1), [2]=CAN1 (gs_usb ch=2) */
volatile uint8_t  g_can_berr_reporting[3] = {0, 0, 0};        /* BERR_REPORTING mode flag */
volatile uint32_t g_can_state[3] = {GS_CAN_STATE_ERROR_ACTIVE,  /* gs_can_state enum */
                                     GS_CAN_STATE_ERROR_ACTIVE,
                                     GS_CAN_STATE_ERROR_ACTIVE};
volatile uint32_t g_can_rxerr[3] = {0, 0, 0};                  /* RECNT */
volatile uint32_t g_can_txerr[3] = {0, 0, 0};                  /* TECNT */
volatile uint8_t  g_can_busoff[3] = {0, 0, 0};                 /* 1=node in bus-off */
volatile uint8_t  g_can_busoff_recovery_pending[3] = {0, 0, 0}; /* 1=recovery detected, push RESTARTED */
volatile uint32_t g_can_busoff_tick[3] = {0, 0, 0};            /* HAL_GetTick() at bus-off entry */

#if CAN_ERROR_FRAME_ENABLE
/* Per-channel pending error ring buffer (ISR writes head, main loop reads tail).
 * Each channel has its own buffer, and only that channel's ERR ISR writes to it,
 * so there's no write contention. Main loop is the sole reader. */
static can_pending_error_t s_pending_errors[3][CAN_PENDING_ERROR_BUF_SIZE];
static volatile uint8_t s_pending_error_head[3] = {0, 0, 0};
static volatile uint8_t s_pending_error_tail[3] = {0, 0, 0};
/* Per-channel rate limiting: last push tick for non-critical error frames */
static uint32_t s_last_error_frame_tick[3] = {0, 0, 0};
#endif /* CAN_ERROR_FRAME_ENABLE */

/* ====== CAN Configuration Parameters (global) ====== */
/* [0]=CAN3 (gs_usb ch=0), [1]=CAN2 (gs_usb ch=1), [2]=CAN1 (gs_usb ch=2)
 * Default: 500k arbitration / 2M data */
can_config_params_t g_can_config[3] = {
    {CAN_DEFAULT_BITTIME_DIV, CAN_DEFAULT_AC_BTS1_SIZE, CAN_DEFAULT_AC_BTS2_SIZE, CAN_DEFAULT_AC_RSAW_SIZE,
     CAN_DEFAULT_FD_BTS1_SIZE, CAN_DEFAULT_FD_BTS2_SIZE, CAN_DEFAULT_FD_RSAW_SIZE, CAN_DEFAULT_FD_SSP_OFFSET,
     CAN_MODE_COMMUNICATE},  /* CAN3 (ch=0) */
    {CAN_DEFAULT_BITTIME_DIV, CAN_DEFAULT_AC_BTS1_SIZE, CAN_DEFAULT_AC_BTS2_SIZE, CAN_DEFAULT_AC_RSAW_SIZE,
     CAN_DEFAULT_FD_BTS1_SIZE, CAN_DEFAULT_FD_BTS2_SIZE, CAN_DEFAULT_FD_RSAW_SIZE, CAN_DEFAULT_FD_SSP_OFFSET,
     CAN_MODE_COMMUNICATE},  /* CAN2 (ch=1) */
    {CAN_DEFAULT_BITTIME_DIV, CAN_DEFAULT_AC_BTS1_SIZE, CAN_DEFAULT_AC_BTS2_SIZE, CAN_DEFAULT_AC_RSAW_SIZE,
     CAN_DEFAULT_FD_BTS1_SIZE, CAN_DEFAULT_FD_BTS2_SIZE, CAN_DEFAULT_FD_RSAW_SIZE, CAN_DEFAULT_FD_SSP_OFFSET,
     CAN_MODE_COMMUNICATE},  /* CAN1 (ch=2) */
};

/**
  * @brief  Program the hardware filter for accept-all (filter0, no id/property comparison).
  *         Must be called within the can_software_reset(TRUE) window (required by can_filter_set).
  *         Enabling (can_filter_enable) is done by the caller after reset release.
  *         The open-source slim build has no ID filtering; all three channels always accept everything.
  */
static void can_hw_filter_accept_all(can_type *can_x)
{
    can_filter_config_type f;

    can_filter_default_para_init(&f);
    f.code_para.id      = 0;
    f.code_para.id_type = CAN_ID_STANDARD;
    f.mask_para.id      = 0x1FFFFFFF;
    f.mask_para.id_type = TRUE;                 /* id fully don't-care → accept both standard and extended frames */
    f.mask_para.data_length    = 0xF;
    f.mask_para.frame_type     = TRUE;
    f.mask_para.fd_error_state = TRUE;
    f.mask_para.fd_format      = TRUE;
    f.mask_para.fd_rate_switch = TRUE;
    f.mask_para.recv_frame     = TRUE;
    can_filter_set(can_x, CAN_FILTER_NUM_0, &f);
}

/**
  * @brief  Add a simulated RX frame to the queue (redirected to can_ring)
  */
static void can_stub_push_rx(uint32_t can_id, uint8_t can_dlc, uint8_t channel,
                             uint8_t flags, const uint8_t *data)
{
    can_stub_rx_frame_t frame;
    frame.can_id = can_id;
    frame.can_dlc = can_dlc;
    frame.channel = channel;
    frame.flags = flags;
    frame.reserved = 0;
    memcpy(frame.data, data, can_dlc > 64 ? 64 : can_dlc);
    frame.timestamp_us = gs_usb_sof_timestamp_us;

    can_ring_push_isr(&frame);
}

/**
  * @brief  Push a SocketCAN-style error frame into the RX queue (main-loop context).
  *         Error frames use can_id = CAN_ERR_FLAG | err_type, DLC=8, flags=0.
  *         Only pushes if BERR_REPORTING is enabled for the channel.
  * @param  ch: gs_usb channel (0=CAN3, 1=CAN2, 2=CAN1)
  * @param  err_type: CAN_ERR_* bit(s) without CAN_ERR_FLAG
  * @param  data: 8-byte SocketCAN error payload (may be NULL for zeroed data)
  */
void can_stub_push_error_frame(uint8_t ch, uint32_t err_type, const uint8_t data[8])
{
    uint8_t zero_data[8] = {0};

    if (ch >= 3) return;
    /* Only push error frames when host requested BERR_REPORTING */
    if (!g_can_berr_reporting[ch]) return;

    if (data == NULL) {
        data = zero_data;
    }

    /* Error frame: can_id = CAN_ERR_FLAG | err_type, classic CAN, DLC=8 */
    can_stub_push_rx(CAN_ERR_FLAG | (err_type & CAN_EFF_MASK),
                     8, ch, 0, data);
}

/**
  * @brief  Store an error frame in the per-channel pending ring buffer (ISR-safe).
  *         Does NOT touch PSRAM — writes only to a small SRAM ring buffer.
  *         The main loop drains the buffer via can_process_pending_error_frames().
  *
  *         This deferred-push design avoids PSRAM cache access from the ERR ISR,
  *         eliminating reentrancy risks with the RX ISR and reducing ISR overhead.
  *
  *         Ring buffer concurrency:
  *         - Single writer per channel (that channel's ERR ISR only)
  *         - Single reader (main loop)
  *         - No lock needed: ISR writes head, main loop writes tail, both atomic
  *
  * @param  ch: gs_usb channel (0=CAN3, 1=CAN2, 2=CAN1)
  * @param  err_type: CAN_ERR_* bit(s) without CAN_ERR_FLAG
  * @param  data: 8-byte error payload (may be NULL for zeroed data)
  */
void can_err_isr_store(uint8_t ch, uint32_t err_type, const uint8_t data[8])
{
#if CAN_ERROR_FRAME_ENABLE
    if (ch >= 3) return;
    /* Only store when the host requested BERR_REPORTING */
    if (!g_can_berr_reporting[ch]) return;

    uint8_t head = s_pending_error_head[ch];
    uint8_t next = (head + 1) % CAN_PENDING_ERROR_BUF_SIZE;

    /* If buffer full, drop the NEW entry (not the oldest).
     * This avoids a race condition where both ISR and main loop modify tail.
     * The main loop drains the buffer frequently, so drops are rare.
     * Critical events (BUSOFF) are stored in slot 0 which is always processed
     * first by the main loop after draining. */
    if (next == s_pending_error_tail[ch]) {
        return;  /* buffer full, drop this error */
    }

    s_pending_errors[ch][head].err_type = err_type;
    if (data) {
        memcpy(s_pending_errors[ch][head].data, data, 8);
    } else {
        memset(s_pending_errors[ch][head].data, 0, 8);
    }
    s_pending_error_head[ch] = next;  /* atomic 8-bit write on Cortex-M4 */
#endif /* CAN_ERROR_FRAME_ENABLE */
}

/**
  * @brief  Main-loop function: drain pending error ring buffer and push to RX queue.
  *         Applies rate limiting to prevent flooding during persistent bus errors.
  *         CAN_ERR_BUSOFF and CAN_ERR_RESTARTED bypass rate limiting (critical).
  *         Must be called from the main loop, NOT from ISR.
  */
void can_process_pending_error_frames(void)
{
#if CAN_ERROR_FRAME_ENABLE
    uint8_t ch;
    for (ch = 0; ch < 3; ch++) {
        uint8_t processed = 0;
        while (s_pending_error_tail[ch] != s_pending_error_head[ch] && processed < CAN_PENDING_ERROR_BUF_SIZE) {
            uint8_t tail = s_pending_error_tail[ch];
            uint32_t err_type = s_pending_errors[ch][tail].err_type;

            /* Rate limiting: non-critical error frames are throttled to
             * CAN_ERR_FRAME_MIN_INTERVAL_MS per channel. Critical events
             * (BUSOFF, RESTARTED) are always pushed immediately. */
            int is_critical = (err_type & (CAN_ERR_BUSOFF | CAN_ERR_RESTARTED)) != 0;
            if (!is_critical) {
                uint32_t now = HAL_GetTick();
                if (now - s_last_error_frame_tick[ch] < CAN_ERR_FRAME_MIN_INTERVAL_MS) {
                    /* Rate limited: skip this frame (drop it to avoid backlog).
                     * Advance tail so we don't re-check the same frame. */
                    s_pending_error_tail[ch] = (tail + 1) % CAN_PENDING_ERROR_BUF_SIZE;
                    processed++;
                    continue;
                }
                s_last_error_frame_tick[ch] = now;
            }

            /* Push to the USB RX queue */
            can_stub_push_error_frame(ch, err_type, s_pending_errors[ch][tail].data);
            s_pending_error_tail[ch] = (tail + 1) % CAN_PENDING_ERROR_BUF_SIZE;
            processed++;
        }
    }
#endif /* CAN_ERROR_FRAME_ENABLE */
}

/**
  * @brief  Update per-channel CAN controller state cache from ERR ISR.
  *         Reads TECNT/RECNT from ERR register and maps status flags to
  *         gs_can_state enum. Detects bus-off entry for recovery tracking.
  * @param  ch: gs_usb channel (0=CAN3, 1=CAN2, 2=CAN1)
  * @param  can_x: CAN instance pointer (CAN1/CAN2/CAN3)
  */
void can_update_error_state(uint8_t ch, can_type *can_x)
{
    if (ch >= 3 || can_x == NULL) return;

    /* Read error counters from ERR register */
    g_can_txerr[ch] = can_x->err_bit.tecnt;
    g_can_rxerr[ch] = can_x->err_bit.recnt;

    /* Determine CAN controller state from STS register */
    if (can_busoff_get(can_x) == SET) {
        g_can_state[ch] = GS_CAN_STATE_BUS_OFF;
        /* Record bus-off entry for main-loop recovery detection */
        if (!g_can_busoff[ch]) {
            g_can_busoff[ch] = 1;
            g_can_busoff_tick[ch] = HAL_GetTick();
        }
    } else if (can_x->sts & CAN_EPASS_FLAG) {
        g_can_state[ch] = GS_CAN_STATE_ERROR_PASSIVE;
        g_can_busoff[ch] = 0;
    } else if (can_x->sts & CAN_EWARN_FLAG) {
        g_can_state[ch] = GS_CAN_STATE_ERROR_WARNING;
        g_can_busoff[ch] = 0;
    } else {
        g_can_state[ch] = GS_CAN_STATE_ERROR_ACTIVE;
        g_can_busoff[ch] = 0;
    }
}

/**
  * @brief  Fill gs_device_state struct for GET_STATE response.
  *         Reads live TECNT/RECNT from CAN ERR register and maps status flags
  *         to gs_can_state enum for accurate, real-time state reporting.
  * @param  ch: gs_usb channel (0=CAN3, 1=CAN2, 2=CAN1)
  * @param  state: output struct
  */
void can_get_device_state(uint8_t ch, struct gs_device_state *state)
{
    static can_type * const can_instances[3] = {CAN3, CAN2, CAN1};

    if (ch >= 3 || state == NULL) {
        if (state) {
            state->state = GS_CAN_STATE_ERROR_ACTIVE;
            state->rxerr = 0;
            state->txerr = 0;
        }
        return;
    }

    can_type *can_x = can_instances[ch];
    if (can_x == NULL) {
        state->state = GS_CAN_STATE_ERROR_ACTIVE;
        state->rxerr = 0;
        state->txerr = 0;
        return;
    }

    /* Read live error counters from ERR register */
    state->rxerr = can_x->err_bit.recnt;
    state->txerr = can_x->err_bit.tecnt;

    /* Determine state from STS register (live, not cached) */
    if (can_busoff_get(can_x) == SET) {
        state->state = GS_CAN_STATE_BUS_OFF;
    } else if (can_x->sts & CAN_EPASS_FLAG) {
        state->state = GS_CAN_STATE_ERROR_PASSIVE;
    } else if (can_x->sts & CAN_EWARN_FLAG) {
        state->state = GS_CAN_STATE_ERROR_WARNING;
    } else {
        state->state = GS_CAN_STATE_ERROR_ACTIVE;
    }
}

/**
  * @brief  Main-loop bus-off recovery handler.
  *         - Detects auto-recovery (BUSOFF flag cleared by HW after 128×11
  *           recessive bits) and pushes CAN_ERR_RESTARTED frame.
  *         - If auto-recovery doesn't occur within CAN_BUSOFF_MANUAL_RECOVERY_MS
  *           (e.g., bus permanently stuck), force-resets error counters via
  *           can_busoff_reset() to allow reconnection.
  *         Must be called from the main loop, NOT from ISR.
  */
void can_handle_busoff_recovery(void)
{
#if CAN_ERROR_FRAME_ENABLE
    static can_type * const can_instances[3] = {CAN3, CAN2, CAN1};
    uint8_t ch;

    for (ch = 0; ch < 3; ch++) {
        if (!g_can_busoff[ch]) {
            /* Not in bus-off; check if recovery was just completed */
            if (g_can_busoff_recovery_pending[ch]) {
                g_can_busoff_recovery_pending[ch] = 0;
                /* Push RESTARTED error frame to notify host */
                uint8_t data[8] = {0};
                can_stub_push_error_frame(ch, CAN_ERR_RESTARTED, data);
            }
            continue;
        }

        /* Registers are inaccessible while the channel is powered down (clock gated):
         * clear state and wait for the channel to be re-enabled */
        if (can_ch_is_disabled(ch)) {
            g_can_busoff[ch] = 0;
            g_can_busoff_recovery_pending[ch] = 0;
            continue;
        }

        /* In bus-off: check if HW auto-recovered */
        can_type *can_x = can_instances[ch];
        if (can_busoff_get(can_x) == RESET) {
            /* Auto-recovery detected: BUSOFF flag cleared by hardware.
             * RECNT and TECNT are auto-cleared by HW on recovery. */
            g_can_busoff[ch] = 0;
            g_can_state[ch] = GS_CAN_STATE_ERROR_ACTIVE;
            g_can_rxerr[ch] = 0;
            g_can_txerr[ch] = 0;
            g_can_busoff_recovery_pending[ch] = 1;
            continue;
        }

        /* Still in bus-off: check manual recovery timeout */
        if (HAL_GetTick() - g_can_busoff_tick[ch] >= CAN_BUSOFF_MANUAL_RECOVERY_MS) {
            /* Auto-recovery failed (bus stuck). Force-reset error counters. */
            can_busoff_reset(can_x);
            g_can_busoff[ch] = 0;
            g_can_state[ch] = GS_CAN_STATE_ERROR_ACTIVE;
            g_can_rxerr[ch] = 0;
            g_can_txerr[ch] = 0;
            g_can_busoff_recovery_pending[ch] = 1;
        }
    }
#endif /* CAN_ERROR_FRAME_ENABLE */
}

/**
  * @brief  Push RX frame from ISR (for CAN_RxMsgNotify integration)
  * @param  ch: channel index (0=CAN3, 1=CAN2, 2=CAN1)
  * @param  pRxMsg: pointer to AT32 CAN RX buffer structure
  */
void can_stub_push_rx_from_isr(int ch, can_rxbuf_type *pRxMsg)
{
    can_stub_rx_frame_t frame;
    uint8_t data_len;

    /* Convert AT32 can_rxbuf_type to can_stub_rx_frame_t */
    frame.can_id = pRxMsg->id;
    if (pRxMsg->id_type == CAN_ID_EXTENDED) {
        frame.can_id |= 0x80000000;
    }

    /* Convert AT32 DLC (0-15) to actual data byte count */
    data_len = can_fd_dlc2len(pRxMsg->data_length);
    frame.can_dlc = data_len;  /* Store byte count, not DLC */

    frame.channel = (uint8_t)ch;
    frame.flags = 0;

    /* Set CAN FD flags */
    if (pRxMsg->fd_format == CAN_FORMAT_FD) {
        frame.flags |= GS_CAN_FLAG_FD;
    }
    if (pRxMsg->fd_rate_switch == CAN_BRS_ON) {
        frame.flags |= GS_CAN_FLAG_BRS;
    }

    frame.reserved = 0;
    memcpy(frame.data, pRxMsg->data, data_len > 64 ? 64 : data_len);
    frame.timestamp_us = gs_usb_sof_timestamp_us;

    /* Push to RX queue (call internal function) */
    can_stub_push_rx(frame.can_id, frame.can_dlc, frame.channel, frame.flags, frame.data);

    /* Increment cumulative HW RX counter */
    if (ch >= 0 && ch < 3) {
        g_can_hw_rx_total[ch]++;
    }
}

/**
  * @brief  Pop a simulated RX frame from the queue (redirected to can_ring)
  */
static uint8_t can_stub_pop_rx(can_stub_rx_frame_t *frame)
{
    if (can_ring_peek_usb(frame)) {
        can_ring_consume_usb();
        return 1;
    }
    return 0;
}
/**
  * @brief  Get CAN instance from gs_usb channel index
  * @param  ch: gs_usb channel index (0=CAN3, 1=CAN2, 2=CAN1)
  * @retval CAN instance pointer (CAN1, CAN2, or CAN3)
  */
static can_type* get_can_instance(uint8_t ch)
{
    switch(ch) {
        case 0: return CAN3;  /* CAN3_RX_IRQHandler → ch=0 */
        case 1: return CAN2;  /* CAN2_RX_IRQHandler → ch=1 */
        case 2: return CAN1;  /* CAN1_RX_IRQHandler → ch=2 */
        default: return NULL;
    }
}

/**
  *  @brief  Initialize CAN software state (called from bsp_can_init.c)
  */
void can_sw_init(void)
{
    /* Initialize software state */
    memset(g_can_stub, 0, sizeof(g_can_stub));
    g_tx_write_idx = 0;
    g_tx_complete_idx = 0;
    g_tx_read_idx = 0;
    g_tx_count = 0;
    g_tx_complete_count = 0;
    g_tx_complete_pending[0] = 0;
    g_tx_complete_pending[1] = 0;
    g_tx_complete_pending[2] = 0;

    /* Initialize RX ring queues */
    can_ring_init();
}


/* macro definition about some parameters of can bit time */
#define BITTIME_DIV                      2
#define AC_BTS1_SIZE                     76
#define AC_BTS2_SIZE                     20
#define AC_RSAW_SIZE                     7

#define FD_BTS1_SIZE                     18
#define FD_BTS2_SIZE                     6
#define FD_RSAW_SIZE                     3
#define FD_SSP_OFFSET                    19
/* the calculation process of can_clk is as follows:
   can_clk = can_pclk/BITTIME_DIV = 192M/2 = 96M; 
*/
/* the calculation process of nominal bit rate is as follows: 
   ac_baudrate = can_clk/(AC_BTS1_SIZE + AC_BTS2_SIZE) = 96M/(144 + 48) = 500k;
   ac_samplepoint = AC_BTS1_SIZE/(AC_BTS1_SIZE + AC_BTS2_SIZE) = 144/192 = 75%;
*/
/* the calculation process of data bit rate is as follows: 
   fd_baudrate = can_clk/(FD_BTS1_SIZE + FD_BTS2_SIZE) = 96M/(36 + 12) = 2M;
   fd_samplepoint = FD_BTS1_SIZE/(FD_BTS1_SIZE + FD_BTS2_SIZE) = 36/48 = 75%;
*/
/* Secondary Sample Point(SSP) is suggested to set the FD_SSP_OFFSET equal to FD_BTS1_SIZE+1. */

/**
  *  @brief  can configiguration (CAN1, gs_usb ch=2).
  *  @param  none
  *  @retval the result of can_configuration
  *          this parameter can be one of the following values:
  *          SUCCESS or ERROR
  */
error_status can_configuration_(void)
{
    can_bittime_type can_bittime_struct;
    can_config_params_t *cfg = &g_can_config[2];  /* CAN1 = gs_usb ch=2 */

    /* as specified in can protocol, the maximum allowable oscillator tolerance is 1%. 
    The hick accuracy does not meet the clock requirements in can protocol. to guarantee normal 
    communication, it is recommended to use hext as the system clock source. */
    if(crm_flag_get(CRM_HEXT_STABLE_FLAG) != SET)
    {
    return ERROR;
    }

    /* enable the can clock */
    crm_periph_clock_enable(CRM_CAN1_PERIPH_CLOCK, TRUE);
    crm_can_clock_select(CRM_CAN1, CRM_CAN_CLOCK_SOURCE_PCLK);

    can_software_reset(CAN1, TRUE);

    /* set can bit time from global config */
    can_bittime_default_para_init(&can_bittime_struct);
    can_bittime_struct.bittime_div = cfg->bittime_div;
    can_bittime_struct.ac_bts1_size = cfg->ac_bts1_size;
    can_bittime_struct.ac_bts2_size = cfg->ac_bts2_size;
    can_bittime_struct.ac_rsaw_size = cfg->ac_rsaw_size;
    can_bittime_struct.fd_bts1_size = cfg->fd_bts1_size;
    can_bittime_struct.fd_bts2_size = cfg->fd_bts2_size;
    can_bittime_struct.fd_rsaw_size = cfg->fd_rsaw_size;
    can_bittime_struct.fd_ssp_offset = cfg->fd_ssp_offset;
    can_bittime_set(CAN1, &can_bittime_struct);

    can_fd_iso_mode_enable(CAN1, TRUE);
    can_stb_transmit_mode_set(CAN1, CAN_STB_TRANSMIT_BY_FIFO);

    /* Hardware filter accept-all (programmed within the reset window) */
    can_hw_filter_accept_all(CAN1);

    can_software_reset(CAN1, FALSE);

    /* Enable filter0 after reset release (AE enable requires reset released) */
    can_filter_enable(CAN1, CAN_FILTER_NUM_0, TRUE);

    can_retransmission_limit_set(CAN1, CAN_RE_TRANS_TIMES_UNLIMIT);
    can_rearbitration_limit_set(CAN1, CAN_RE_ARBI_TIMES_UNLIMIT);
    can_rxbuf_warning_set(CAN1, 3);
    can_mode_set(CAN1, (can_mode_type)cfg->mode);

    /* can interrupt config */
    #if CAN_USE_ISR
    nvic_irq_enable(CAN1_RX_IRQn, 1, 0);
    nvic_irq_enable(CAN1_TX_IRQn, 1, 0);
    nvic_irq_enable(CAN1_ERR_IRQn, 2, 0);
    #endif
    can_interrupt_enable(CAN1, CAN_RAFIE_INT|CAN_RFIE_INT|CAN_ROIE_INT|CAN_RIE_INT, TRUE);
#if CAN_ERROR_FRAME_ENABLE
    can_interrupt_enable(CAN1, CAN_BEIE_INT, TRUE);
    /* Error interrupt enables for BERR_REPORTING: bus error (BEIE), error/passive
     * state change (EIE+EPIE). ALIE (arbitration lost) is NOT enabled because
     * arbitration loss is a normal event during simultaneous send/receive and
     * would flood the error frame queue. */
    can_interrupt_enable(CAN1, CAN_EIE_INT, TRUE);
    can_interrupt_enable(CAN1, CAN_EPIE_INT, TRUE);
#endif
    can_interrupt_enable(CAN1, CAN_TSIE_INT, TRUE);

    return SUCCESS;
}


/**
  *  @brief  can configiguration (CAN2, gs_usb ch=1).
  *  @param  none
  *  @retval the result of can_configuration
  *          this parameter can be one of the following values:
  *          SUCCESS or ERROR
  */
error_status can2_configuration_(void)
{
    can_bittime_type can_bittime_struct;
    can_config_params_t *cfg = &g_can_config[1];  /* CAN2 = gs_usb ch=1 */

    /* as specified in can protocol, the maximum allowable oscillator tolerance is 1%. 
    The hick accuracy does not meet the clock requirements in can protocol. to guarantee normal 
    communication, it is recommended to use hext as the system clock source. */
    if(crm_flag_get(CRM_HEXT_STABLE_FLAG) != SET)
    {
    return ERROR;
    }

    /* enable the can clock */
    crm_periph_clock_enable(CRM_CAN2_PERIPH_CLOCK, TRUE);
    crm_can_clock_select(CRM_CAN2, CRM_CAN_CLOCK_SOURCE_PCLK);

    can_software_reset(CAN2, TRUE);

    /* set can bit time from global config */
    can_bittime_default_para_init(&can_bittime_struct);
    can_bittime_struct.bittime_div = cfg->bittime_div;
    can_bittime_struct.ac_bts1_size = cfg->ac_bts1_size;
    can_bittime_struct.ac_bts2_size = cfg->ac_bts2_size;
    can_bittime_struct.ac_rsaw_size = cfg->ac_rsaw_size;
    can_bittime_struct.fd_bts1_size = cfg->fd_bts1_size;
    can_bittime_struct.fd_bts2_size = cfg->fd_bts2_size;
    can_bittime_struct.fd_rsaw_size = cfg->fd_rsaw_size;
    can_bittime_struct.fd_ssp_offset = cfg->fd_ssp_offset;
    can_bittime_set(CAN2, &can_bittime_struct);

    can_fd_iso_mode_enable(CAN2, TRUE);
    can_stb_transmit_mode_set(CAN2, CAN_STB_TRANSMIT_BY_FIFO);

    /* Hardware filter accept-all (programmed within the reset window) */
    can_hw_filter_accept_all(CAN2);

    can_software_reset(CAN2, FALSE);

    /* Enable filter0 after reset release (AE enable requires reset released) */
    can_filter_enable(CAN2, CAN_FILTER_NUM_0, TRUE);

    can_retransmission_limit_set(CAN2, CAN_RE_TRANS_TIMES_UNLIMIT);
    can_rearbitration_limit_set(CAN2, CAN_RE_ARBI_TIMES_UNLIMIT);
    can_rxbuf_warning_set(CAN2, 3);
    can_mode_set(CAN2, (can_mode_type)cfg->mode);

    /* can interrupt config */
    #if CAN_USE_ISR
    /* Keep the CAN2 priority identical to CAN1/CAN3 so that, with all three
     * channels active, CAN2 cannot preempt the CAN1/CAN3 RX ISRs and race on
     * the SRAM ring buffer. With all three channels at the same priority
     * (initial version, commit 72b8101), no lockup was observed. */
    nvic_irq_enable(CAN2_RX_IRQn, 1, 0);
    nvic_irq_enable(CAN2_TX_IRQn, 1, 0);
    nvic_irq_enable(CAN2_ERR_IRQn, 2, 0);
    #endif
    can_interrupt_enable(CAN2, CAN_RAFIE_INT|CAN_RFIE_INT|CAN_ROIE_INT|CAN_RIE_INT, TRUE);
#if CAN_ERROR_FRAME_ENABLE
    can_interrupt_enable(CAN2, CAN_BEIE_INT, TRUE);
    /* Error interrupt enables for BERR_REPORTING (see CAN1 config for details). */
    can_interrupt_enable(CAN2, CAN_EIE_INT, TRUE);
    can_interrupt_enable(CAN2, CAN_EPIE_INT, TRUE);
#endif
    can_interrupt_enable(CAN2, CAN_TSIE_INT, TRUE);

    return SUCCESS;
}


/**
  *  @brief  can configiguration (CAN3, gs_usb ch=0).
  *  @param  none
  *  @retval the result of can_configuration
  *          this parameter can be one of the following values:
  *          SUCCESS or ERROR
  */
error_status can3_configuration_(void)
{
    can_bittime_type can_bittime_struct;
    can_config_params_t *cfg = &g_can_config[0];  /* CAN3 = gs_usb ch=0 */

    /* as specified in can protocol, the maximum allowable oscillator tolerance is 1%. 
    The hick accuracy does not meet the clock requirements in can protocol. to guarantee normal 
    communication, it is recommended to use hext as the system clock source. */
    if(crm_flag_get(CRM_HEXT_STABLE_FLAG) != SET)
    {
    return ERROR;
    }

    /* enable the can clock */
    crm_periph_clock_enable(CRM_CAN3_PERIPH_CLOCK, TRUE);
    crm_can_clock_select(CRM_CAN3, CRM_CAN_CLOCK_SOURCE_PCLK);

    can_software_reset(CAN3, TRUE);

    /* set can bit time from global config */
    can_bittime_default_para_init(&can_bittime_struct);
    can_bittime_struct.bittime_div = cfg->bittime_div;
    can_bittime_struct.ac_bts1_size = cfg->ac_bts1_size;
    can_bittime_struct.ac_bts2_size = cfg->ac_bts2_size;
    can_bittime_struct.ac_rsaw_size = cfg->ac_rsaw_size;
    can_bittime_struct.fd_bts1_size = cfg->fd_bts1_size;
    can_bittime_struct.fd_bts2_size = cfg->fd_bts2_size;
    can_bittime_struct.fd_rsaw_size = cfg->fd_rsaw_size;
    can_bittime_struct.fd_ssp_offset = cfg->fd_ssp_offset;
    can_bittime_set(CAN3, &can_bittime_struct);

    can_fd_iso_mode_enable(CAN3, TRUE);
    can_stb_transmit_mode_set(CAN3, CAN_STB_TRANSMIT_BY_FIFO);

    /* Hardware filter accept-all (programmed within the reset window) */
    can_hw_filter_accept_all(CAN3);

    can_software_reset(CAN3, FALSE);

    /* Enable filter0 after reset release (AE enable requires reset released) */
    can_filter_enable(CAN3, CAN_FILTER_NUM_0, TRUE);

    can_retransmission_limit_set(CAN3, CAN_RE_TRANS_TIMES_UNLIMIT);
    can_rearbitration_limit_set(CAN3, CAN_RE_ARBI_TIMES_UNLIMIT);
    can_rxbuf_warning_set(CAN3, 3);
    can_mode_set(CAN3, (can_mode_type)cfg->mode);

    /* can interrupt config */
    #if CAN_USE_ISR
    nvic_irq_enable(CAN3_RX_IRQn, 1, 0);
    nvic_irq_enable(CAN3_TX_IRQn, 1, 0);
    nvic_irq_enable(CAN3_ERR_IRQn, 2, 0);
    #endif
    can_interrupt_enable(CAN3, CAN_RAFIE_INT|CAN_RFIE_INT|CAN_ROIE_INT|CAN_RIE_INT, TRUE);
#if CAN_ERROR_FRAME_ENABLE
    can_interrupt_enable(CAN3, CAN_BEIE_INT, TRUE);
    /* Error interrupt enables for BERR_REPORTING (see CAN1 config for details). */
    can_interrupt_enable(CAN3, CAN_EIE_INT, TRUE);
    can_interrupt_enable(CAN3, CAN_EPIE_INT, TRUE);
#endif
    can_interrupt_enable(CAN3, CAN_TSIE_INT, TRUE);

    return SUCCESS;
}



/**
  *  @brief  Set arbitration bit timing for a specific CAN channel
  *         Updates global config and reconfigures CAN hardware.
  *         Also updates FD data phase parameters if provided (non-zero).
  *         AT32 CAN uses a SHARED prescaler for arbitration and data phases.
  */
void can_set_bittime(uint8_t ch, can_bittime_type *bittime)
{
    if (ch >= GS_USB_CAN_CHANNEL_MAX) return;

    /* Update global config - arbitration parameters */
    g_can_config[ch].bittime_div = bittime->bittime_div;
    g_can_config[ch].ac_bts1_size = bittime->ac_bts1_size;
    g_can_config[ch].ac_bts2_size = bittime->ac_bts2_size;
    g_can_config[ch].ac_rsaw_size = bittime->ac_rsaw_size;

    /* Update FD data phase parameters if provided (non-zero) */
    if (bittime->fd_bts1_size > 0 && bittime->fd_bts2_size > 0) {
        g_can_config[ch].fd_bts1_size = bittime->fd_bts1_size;
        g_can_config[ch].fd_bts2_size = bittime->fd_bts2_size;
        g_can_config[ch].fd_rsaw_size = bittime->fd_rsaw_size;
        g_can_config[ch].fd_ssp_offset = bittime->fd_ssp_offset;
    }

    /* Reconfigure CAN hardware */
    switch(ch) {
        case 0: can3_configuration_(); break;  /* CAN3 */
        case 1: can2_configuration_(); break;  /* CAN2 */
        case 2: can_configuration_(); break;   /* CAN1 */
    }
}

/**
  *  @brief  Set CAN FD data phase bit timing for a specific CAN channel
  *         Updates global config and reconfigures CAN hardware
  */
void can_set_fd_bittime(uint8_t ch, can_bittime_type *bittime)
{
    if (ch >= GS_USB_CAN_CHANNEL_MAX) return;

    /* Update global config (data phase parameters) */
    g_can_config[ch].fd_bts1_size = bittime->fd_bts1_size;
    g_can_config[ch].fd_bts2_size = bittime->fd_bts2_size;
    g_can_config[ch].fd_rsaw_size = bittime->fd_rsaw_size;
    g_can_config[ch].fd_ssp_offset = bittime->fd_ssp_offset;

    /* Also update arbitration parameters from input */
    g_can_config[ch].bittime_div = bittime->bittime_div;
    g_can_config[ch].ac_bts1_size = bittime->ac_bts1_size;
    g_can_config[ch].ac_bts2_size = bittime->ac_bts2_size;
    g_can_config[ch].ac_rsaw_size = bittime->ac_rsaw_size;

    g_can_stub[ch].mode = 1; /* FD mode */

    /* Reconfigure CAN hardware */
    switch(ch) {
        case 0: can3_configuration_(); break;  /* CAN3 */
        case 1: can2_configuration_(); break;  /* CAN2 */
        case 2: can_configuration_(); break;   /* CAN1 */
    }
}

/**
  *  @brief  Enable a specific CAN channel
  *         Updates mode and reconfigures CAN hardware
  */
void can_enable(uint8_t ch)
{
    if (ch >= GS_USB_CAN_CHANNEL_MAX) return;

    g_can_stub[ch].enabled = 1;
    g_can_config[ch].mode = CAN_MODE_COMMUNICATE;

    /* Reconfigure CAN hardware */
    switch(ch) {
        case 0: can3_configuration_(); break;  /* CAN3 */
        case 1: can2_configuration_(); break;  /* CAN2 */
        case 2: can_configuration_(); break;   /* CAN1 */
    }
}

/**
  *  @brief  Disable a specific CAN channel
  *         Updates mode and reconfigures CAN hardware
  */
void can_disable(uint8_t ch)
{
    if (ch >= GS_USB_CAN_CHANNEL_MAX) return;

    g_can_stub[ch].enabled = 0;

    /* Reconfigure with software reset (mode handled in config) */
    switch(ch) {
        case 0: can3_configuration_(); break;  /* CAN3 */
        case 1: can2_configuration_(); break;  /* CAN2 */
        case 2: can_configuration_(); break;   /* CAN1 */
    }
}

/**
  *  @brief  Set CAN FD data phase bit timing for all CAN channels
  */
void can_set_fd_bittime_all(can_bittime_type *bittime)
{
    for (uint8_t ch = 0; ch < GS_USB_CAN_CHANNEL_MAX; ch++) {
        can_set_fd_bittime(ch, bittime);
    }
}

/**
  *  @brief  Transmit CAN frame - real hardware implementation
  *  @retval 1=success (written to STB), 0=fail (error, frame should be discarded),
  *         2=STB full (retryable, caller should NOT consume the frame)
  */
uint8_t can_stub_transmit(uint8_t ch, uint32_t ech0_id, uint32_t can_id, uint8_t can_dlc,
                          uint8_t flags, const uint8_t *data)
{
    can_type *can_x;
    can_txbuf_type tx_buf;
    uint8_t data_len;
    can_stub_tx_frame_t *frame;
    can_stb_status_type stb_sts;
    error_status write_ret;

    if (ch >= GS_USB_CAN_CHANNEL_MAX) {
        return 0;
    }
    if (!g_can_stub[ch].enabled) {
        return 0;
    }
    /* Registers are inaccessible while the channel is powered down (clock gated):
     * drop the TX frame */
    if (can_ch_is_disabled(ch)) {
        g_can_tx_fail[ch]++;
        return 0;
    }
    can_x = get_can_instance(ch);
    if (can_x == NULL) {
        return 0;
    }

    data_len = can_fd_dlc2len(can_dlc);

    /* Check STB status before writing */
    stb_sts = can_stb_status_get(can_x);
    if (stb_sts == CAN_STB_STATUS_FULL) {
        g_can_tx_stb_full[ch]++;
        /* STB full: do not consume the frame; return 2 so the caller retries */
        return 2;
    }

    /* Check echo queue space AFTER STB check (STB not full but echo queue may be full).
     * Return 2 (retry) instead of 0 (drop) to prevent TX frame loss.
     * The frame stays in gs_usb_tx_ring and retries when echo queue drains. */
    if (g_tx_count >= CAN_STUB_TX_QUEUE_SIZE) {
        g_can_tx_echo_full[ch]++;
        return 2;
    }

    /* Prepare TX buffer structure */
    memset(&tx_buf, 0, sizeof(tx_buf));
    tx_buf.id = can_id & 0x1FFFFFFF;
    tx_buf.id_type = (can_id & 0x80000000) ? CAN_ID_EXTENDED : CAN_ID_STANDARD;
    tx_buf.frame_type = CAN_FRAME_DATA;
    tx_buf.fd_format = (flags & GS_CAN_FLAG_FD) ? CAN_FORMAT_FD : CAN_FORMAT_CLASSIC;
    tx_buf.fd_rate_switch = (flags & GS_CAN_FLAG_BRS) ? CAN_BRS_ON : CAN_BRS_OFF;
    /* AT32 can_data_length_type uses same encoding as gs_usb DLC (0-15) */
    tx_buf.data_length = (can_data_length_type)can_dlc;
    memcpy(tx_buf.data, data, data_len > 64 ? 64 : data_len);

    /* Write to STB buffer */
    write_ret = can_txbuf_write(can_x, CAN_TXBUF_STB, &tx_buf);
    if (write_ret != SUCCESS) {
        g_can_tx_fail[ch]++;
        return 0;
    }

    /* Trigger STB transmission (all pending frames) */
    can_txbuf_transmit(can_x, CAN_TRANSMIT_STB_ALL);

    g_can_tx_success[ch]++;

    /* Store echo frame for TX completion callback.
     * can_dlc field stores actual byte count (not DLC encoding).
     *
     * ★ completed=1 is set immediately, without relying on the CAN TX ISR (TSIF) ★
     * Reason: TSIF on the AT32 FDCAN is a flag (not a counter). Under high load,
     * several frames may complete within one ISR service interval yet only a
     * single TSIF interrupt fires, so g_tx_complete_pending ends up below the
     * actual completion count (in testing, 130647 transmitted frames triggered
     * only 130520 TSIF interrupts, losing 127 echoes).
     *
     * Safety analysis:
     * - can_txbuf_write returning SUCCESS means the frame was written into the
     *   STB (Secondary Transmit Buffer)
     * - The STB is a FIFO, so the frame will eventually be sent on the CAN bus
     *   (unless a bus error occurs, but fail=0 means no error)
     * - If the CAN bus has an error, can_txbuf_write returns ERROR and the frame
     *   is never pushed into the echo queue
     * - The echo timestamp is taken at can_stub_transmit time (SOF timestamp),
     *   less than 1 ms away from the actual transmit time
     *
     * The CAN TX ISR (can_stub_notify_tx_complete) is kept for diagnostics
     * (counting TSIF firings) but is no longer used to advance the echo queue. */
    frame = &g_tx_queue[g_tx_write_idx];
    frame->ech0_id = ech0_id;
    frame->can_id = can_id;
    frame->can_dlc = data_len;  /* Store byte count */
    frame->channel = ch;
    frame->flags = flags;
    frame->reserved = 0;
    memcpy(frame->data, data, data_len > 64 ? 64 : data_len);
    frame->timestamp_us = gs_usb_sof_timestamp_us;
    frame->pending = 1;
    frame->completed = 1;  /* mark as completed immediately, not waiting for the TSIF interrupt */
    g_tx_complete_count++;  /* count as completed immediately so peek_tx_echo can return right away */

    g_tx_write_idx = (g_tx_write_idx + 1) % CAN_STUB_TX_QUEUE_SIZE;
    g_tx_count++;

    g_can_stub[ch].tx_cnt++;
    can_tx_cnt++;

    return 1;
}

/**
  *  @brief  Notify TX complete - called from CAN TX ISR.
  *          The ISR only increments the pending counter of the matching channel;
  *          it never touches the echo queue pointers directly. The main loop calls
  *          can_stub_advance_tx_complete() on peek to consume the pending counter.
  *  @param  ch: channel index (0=CAN3, 1=CAN2, 2=CAN1)
  */
void can_stub_notify_tx_complete(uint8_t ch)
{
    if (ch < 3) {
        g_tx_complete_pending[ch]++;
    }
}

/**
  *  @brief  Advance completed flags by consuming per-channel TX complete events.
  *          Called from main loop (can_stub_peek_tx_echo) before peeking.
  *          Walks the queue and marks every completed frame (out of FIFO order),
  *          unblocking the FIFO head. The host validates echoes via echo_id + can_id
  *          checks, so echo return order does not matter.
  */
static void can_stub_advance_tx_complete(void)
{
    /* Walk the queue and mark every pending, not-yet-completed frame as completed */
    uint8_t check_idx = g_tx_read_idx;
    uint8_t available = g_tx_count;

    while (available > 0) {
        can_stub_tx_frame_t *frame = &g_tx_queue[check_idx];
        if (frame->pending && !frame->completed) {
            uint8_t ch = frame->channel;
            if (ch < 3 && g_tx_complete_pending[ch] > 0) {
                g_tx_complete_pending[ch]--;
                frame->completed = 1;
                g_tx_complete_count++;
            }
        }
        check_idx = (check_idx + 1) % CAN_STUB_TX_QUEUE_SIZE;
        available--;
    }
}

/**
  *  @brief  Get pending TX echo frame (legacy interface, peek+consume preferred)
  *          Only frames whose CAN TX has completed are returned.
  */
uint8_t can_stub_get_tx_echo(can_stub_tx_frame_t *frame)
{
    can_stub_advance_tx_complete();  /* consume TX-complete events accumulated by the ISR */
    if (g_tx_complete_count == 0) return 0;

    /* Walk the queue and return the first completed, unconsumed frame (out of FIFO order) */
    uint8_t check_idx = g_tx_read_idx;
    uint8_t available = g_tx_count;

    while (available > 0) {
        can_stub_tx_frame_t *src = &g_tx_queue[check_idx];
        if (src->pending && src->completed) {
            memcpy(frame, src, sizeof(can_stub_tx_frame_t));
            src->pending = 0;
            src->completed = 0;
            g_tx_count--;
            g_tx_complete_count--;

            /* If the consumed frame sits at read_idx, advance read_idx past consumed frames */
            if (check_idx == g_tx_read_idx) {
                g_tx_read_idx = (g_tx_read_idx + 1) % CAN_STUB_TX_QUEUE_SIZE;
                while (g_tx_count > 0 && !g_tx_queue[g_tx_read_idx].pending) {
                    g_tx_read_idx = (g_tx_read_idx + 1) % CAN_STUB_TX_QUEUE_SIZE;
                }
            }
            return 1;
        }
        check_idx = (check_idx + 1) % CAN_STUB_TX_QUEUE_SIZE;
        available--;
    }
    return 0;
}

/**
  *  @brief  Peek at pending TX echo frame without dequeuing.
  *          Returns an echo only while completed-but-unreturned frames exist.
  */
uint8_t can_stub_peek_tx_echo(can_stub_tx_frame_t *frame)
{
    can_stub_advance_tx_complete();  /* consume TX-complete events accumulated by the ISR */
    if (g_tx_complete_count == 0) return 0;

    /* Walk the queue and return the first completed, unconsumed frame (out of FIFO order) */
    uint8_t check_idx = g_tx_read_idx;
    uint8_t available = g_tx_count;

    while (available > 0) {
        can_stub_tx_frame_t *src = &g_tx_queue[check_idx];
        if (src->pending && src->completed) {
            memcpy(frame, src, sizeof(can_stub_tx_frame_t));
            return 1;
        }
        check_idx = (check_idx + 1) % CAN_STUB_TX_QUEUE_SIZE;
        available--;
    }
    return 0;
}

/**
  *  @brief  Consume the peeked TX echo frame (advance read_idx)
  *          Consumes the frame returned by peek (the first completed, unconsumed frame).
  */
void can_stub_consume_tx_echo(void)
{
    if (g_tx_complete_count == 0) return;

    /* Walk the queue and consume the first completed, unconsumed frame */
    uint8_t check_idx = g_tx_read_idx;
    uint8_t available = g_tx_count;

    while (available > 0) {
        can_stub_tx_frame_t *src = &g_tx_queue[check_idx];
        if (src->pending && src->completed) {
            src->pending = 0;
            src->completed = 0;
            g_tx_count--;
            g_tx_complete_count--;

            /* If the consumed frame sits at read_idx, advance read_idx past consumed frames */
            if (check_idx == g_tx_read_idx) {
                g_tx_read_idx = (g_tx_read_idx + 1) % CAN_STUB_TX_QUEUE_SIZE;
                while (g_tx_count > 0 && !g_tx_queue[g_tx_read_idx].pending) {
                    g_tx_read_idx = (g_tx_read_idx + 1) % CAN_STUB_TX_QUEUE_SIZE;
                }
            }
            return;
        }
        check_idx = (check_idx + 1) % CAN_STUB_TX_QUEUE_SIZE;
        available--;
    }
}

/**
  *  @brief  Get TX echo queue count (only TX-completed frames ready for return)
  */
uint8_t can_stub_get_tx_echo_count(void)
{
    return (uint8_t)g_tx_complete_count;
}

/**
  * @brief  Get pending RX frame
  */
uint8_t can_stub_get_rx_frame(can_stub_rx_frame_t *frame)
{
    return can_stub_pop_rx(frame);
}

/**
  * @brief  Peek at pending RX frame without dequeuing (redirected to can_ring)
  */
uint8_t can_stub_peek_rx_frame(can_stub_rx_frame_t *frame)
{
    return can_ring_peek_usb(frame);
}

/**
  * @brief  Consume the peeked RX frame (advance tail) (redirected to can_ring)
  */
void can_stub_consume_rx_frame(void)
{
    can_ring_consume_usb();
}

/**
  *  @brief  Inject a simulated CAN FD RX frame (for testing)
  */
void can_stub_inject_rx_frame(uint8_t ch, uint32_t can_id, uint8_t can_dlc,
                              uint8_t is_fd, uint8_t is_brs, const uint8_t *data)
{
    uint8_t flags = 0;
    if (is_fd) flags |= GS_CAN_FLAG_FD;
    if (is_brs) flags |= GS_CAN_FLAG_BRS;

    can_stub_push_rx(can_id, can_dlc, ch, flags, data);
    g_can_stub[ch].rx_cnt++;
    if (ch == 0) can_rx_cnt++;
    else if (ch == 1) can2_rx_cnt++;
    else can3_rx_cnt++;
}

/**
  *  @brief  Auto-generate simulated RX frames for testing
  */
void can_stub_auto_generate_rx(void)
{
    static uint32_t counter = 0;

    for (uint8_t ch = 0; ch < GS_USB_CAN_CHANNEL_MAX; ch++) {
        if (!g_can_stub[ch].enabled) continue;

        uint8_t data[64];
        memset(data, 0, sizeof(data));
        data[0] = (counter >> 0) & 0xFF;
        data[1] = (counter >> 8) & 0xFF;
        data[2] = (counter >> 16) & 0xFF;
        data[3] = (counter >> 24) & 0xFF;
        data[4] = ch;
        data[5] = 0xFD; /* marker for FD frame */

        /* Alternate between classic CAN and CAN FD */
        if (counter & 1) {
            /* CAN FD frame, 8 bytes, BRS enabled */
            can_stub_inject_rx_frame(ch, 0x12345678 | (ch << 16), 8, 1, 1, data);
        } else {
            /* Classic CAN frame, 8 bytes */
            can_stub_inject_rx_frame(ch, 0x100 | (ch << 8), 8, 0, 0, data);
        }

        counter++;
    }
}

/**
  *  @brief  Get channel RX count
  */
uint32_t can_stub_get_rx_cnt(uint8_t ch)
{
    if (ch >= GS_USB_CAN_CHANNEL_MAX) return 0;
    return g_can_stub[ch].rx_cnt;
}

/**
  *  @brief  Get channel TX count
  */
uint32_t can_stub_get_tx_cnt(uint8_t ch)
{
    if (ch >= GS_USB_CAN_CHANNEL_MAX) return 0;
    return g_can_stub[ch].tx_cnt;
}

/**
  *  @brief  Check if channel is enabled
  */
uint8_t can_stub_is_enabled(uint8_t ch)
{
    if (ch >= GS_USB_CAN_CHANNEL_MAX) return 0;
    return g_can_stub[ch].enabled;
}

/* ====== Legacy interface compatibility ====== */
#if 0
void CAN1_RX_IRQHandler(void) { }
void CAN1_ERR_IRQHandler(void) { }
void CAN2_RX_IRQHandler(void) { }
void CAN2_ERR_IRQHandler(void) { }
void CAN3_RX_IRQHandler(void) { }
void CAN3_ERR_IRQHandler(void) { }
void can_transmit_data(void)
{
    /* Legacy test function - do nothing in stub mode */
}
#endif

