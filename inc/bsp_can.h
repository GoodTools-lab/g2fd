/*
 * G2FD - three-channel CANFD gs_usb adapter firmware (AT32F456)
 * Copyright (c) 2026 G2FD contributors
 * SPDX-License-Identifier: MIT
 */

#ifndef __BSP_CAN_H__
#define __BSP_CAN_H__

#include "at32f45x.h"

#define CAN_MAX_DATA_SIZE       (64u)

#define CAN_USE_ISR              1

/* ====== Error frame handling master switch (for debugging/troubleshooting) ======
 * 1 = enabled: ERR interrupts enabled (BEIE/EIE/EPIE), error frame reporting
 *              (USB RX queue / GRL offline log), bus-off detection and recovery
 * 0 = fully disabled: restores the legacy behavior from before error frame
 *              handling was introduced, for A/B testing whether error frame
 *              handling causes issues (e.g., sporadic TsMaster ACK errors / RX frame loss).
 * Effects when disabled:
 *   - CANx_ERR_IRQHandler never fires (no error interrupt sources), no ERR ISR overhead
 *   - No error frames (CAN_ERR_FLAG frames) are generated; the GRL log records no errors either
 *   - GET_STATE error counters/state cache are no longer refreshed (initial values persist)
 *   - bus-off relies solely on hardware auto-recovery (128×11 recessive bits), with no
 *     software manual-recovery fallback */
#ifndef CAN_ERROR_FRAME_ENABLE
#define CAN_ERROR_FRAME_ENABLE   0
#endif

/* ====== Operation Mode ======
 * The open-source slim build keeps only gs_usb bridge mode (no offline logging mode).
 * The variable is retained for diagnostic logs and debugger visibility; it is always CAN_MODE_GS_USB. */
typedef enum {
    CAN_MODE_GS_USB = 1       /* gs_usb bridge mode (only mode in this build) */
} can_operation_mode_t;

extern volatile can_operation_mode_t g_can_operation_mode;

void can_init(void);
void can_sw_init(void);

/* ====== Per-channel hardware configuration (read bittiming from g_can_config) ======
 * The '_'-suffixed variants read bit timing from the global g_can_config[], for use by
 * can_init() and runtime reconfiguration. The non-suffixed variants (can_configuration
 * etc., defined in bsp_can_init.c) use compile-time hardcoded macros. */
error_status can_configuration_(void);    /* CAN1 = gs_usb ch=2 */
error_status can2_configuration_(void);   /* CAN2 = gs_usb ch=1 */
error_status can3_configuration_(void);   /* CAN3 = gs_usb ch=0 */

void CAN1_RX_IRQHandler(void);
void CAN1_ERR_IRQHandler(void);
void can_transmit_data(void);

void CAN2_RX_IRQHandler(void);
void CAN2_ERR_IRQHandler(void);

void CAN3_RX_IRQHandler(void);
void CAN3_ERR_IRQHandler(void);

void CAN1_TX_IRQHandler(void);
void CAN2_TX_IRQHandler(void);
void CAN3_TX_IRQHandler(void);

/* ====== CAN Configuration Parameters ====== */
/* CAN channel configuration parameters */
typedef struct {
    uint8_t  bittime_div;      /* BRP (Prescaler) */
    uint16_t ac_bts1_size;     /* Arbitration phase TSEG1 */
    uint8_t  ac_bts2_size;     /* Arbitration phase TSEG2 */
    uint8_t  ac_rsaw_size;     /* Arbitration phase SJW */
    uint16_t fd_bts1_size;     /* Data phase TSEG1 */
    uint8_t  fd_bts2_size;     /* Data phase TSEG2 */
    uint8_t  fd_rsaw_size;     /* Data phase SJW */
    uint8_t  fd_ssp_offset;    /* Secondary Sample Point offset */
    uint8_t  mode;             /* CAN mode (CAN_MODE_COMMUNICATE, etc.) */
} can_config_params_t;

/* Global configuration for each CAN channel (extern)
 * [0]=CAN3 (gs_usb ch=0), [1]=CAN2 (gs_usb ch=1), [2]=CAN1 (gs_usb ch=2) */
extern can_config_params_t g_can_config[3];

/* Default values (500k arbitration, 2M data) */
#define CAN_DEFAULT_BITTIME_DIV      2
#define CAN_DEFAULT_AC_BTS1_SIZE     76
#define CAN_DEFAULT_AC_BTS2_SIZE     20
#define CAN_DEFAULT_AC_RSAW_SIZE     7
#define CAN_DEFAULT_FD_BTS1_SIZE     18
#define CAN_DEFAULT_FD_BTS2_SIZE     6
#define CAN_DEFAULT_FD_RSAW_SIZE     3
#define CAN_DEFAULT_FD_SSP_OFFSET    19

/* Active gs_usb channels (must match GS_USB_ACTIVE_CHANNELS in gs_usb_protocol.h) */
#define GS_USB_CAN_CHANNEL_MAX    3
void can_set_bittime(uint8_t ch, can_bittime_type *bittime);
void can_set_fd_bittime(uint8_t ch, can_bittime_type *bittime);
void can_enable(uint8_t ch);
void can_disable(uint8_t ch);

/* gs_usb protocol support - global operations (apply to all channels) */
void can_set_bittime_all(can_bittime_type *bittime);
void can_set_fd_bittime_all(can_bittime_type *bittime);
void can_enable_all(void);
void can_disable_all(void);

/* Counters for monitoring */
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
/* RX FIFO overflow counters (frame-loss attribution diagnostics, see bsp_can_init.c) */
extern volatile uint32_t can_rx_ovf_cnt;
extern volatile uint32_t can2_rx_ovf_cnt;

/* ====== CAN Error Frame Reporting (BERR_REPORTING) ====== */
/* Per-channel BERR_REPORTING enable flag (set by gs_usb_can_enable when host
 * requests GS_CAN_MODE_BERR_REPORTING). ISR checks this before pushing error
 * frames to avoid flooding the RX queue when the host doesn't want them. */
extern volatile uint8_t g_can_berr_reporting[3];

/* Per-channel CAN controller state cache (updated by ERR ISR, read by
 * GS_USB_BREQ_GET_STATE handler). Indices: 0=CAN3, 1=CAN2, 2=CAN1. */
extern volatile uint32_t g_can_state[3];     /* gs_can_state enum value */
extern volatile uint32_t g_can_rxerr[3];     /* receive error counter (RECNT) */
extern volatile uint32_t g_can_txerr[3];     /* transmit error counter (TECNT) */

/* Per-channel bus-off tracking (set by ERR ISR on BUSOFF entry, cleared by
 * main loop recovery handler after auto-recovery detection). */
extern volatile uint8_t g_can_busoff[3];         /* 1=node in bus-off state */
extern volatile uint8_t g_can_busoff_recovery_pending[3]; /* 1=recovery detected, need RESTARTED frame */
extern volatile uint32_t g_can_busoff_tick[3];   /* HAL_GetTick() at bus-off entry */

/* Cumulative per-channel CAN HW RX / USB TX counters for diagnostics */
extern volatile uint32_t g_can_hw_rx_total[3];
extern volatile uint16_t g_rx_count;           /* Compat: ISR + USB queue backlog */

/* CAN TX (device→bus) statistics per channel */
extern volatile uint32_t g_can_tx_success[3];  /* Successfully written to STB */
extern volatile uint32_t g_can_tx_fail[3];     /* Failed to write to STB (dropped) */
extern volatile uint32_t g_can_tx_stb_full[3]; /* STB was full when trying to send */
extern volatile uint32_t g_can_tx_echo_full[3]; /* Echo queue full (retryable) */
extern volatile uint32_t g_can_tx_complete[3];  /* CAN TX complete ISR count (frames actually transmitted on bus) */
extern volatile uint32_t g_rx_queue_overflow;  /* RX frames lost due to queue full */
extern volatile uint16_t g_rx_queue_peak;      /* Peak total backlog (high-water mark) */

/* Total RX capacity: 2 SRAM queues (128 each, defined in can_ring.h) */
#define CAN_STUB_RX_QUEUE_CAPACITY  (2 * CAN_RING_QUEUE_SIZE)  /* 256 frames */

/* ====== Stub CAN driver interfaces ====== */

/* Simulated RX frame structure (matches gs_host_frame layout) */
typedef struct {
    uint32_t can_id;
    uint8_t  can_dlc;
    uint8_t  channel;
    uint8_t  flags;           /* GS_CAN_FLAG_FD | GS_CAN_FLAG_BRS | GS_CAN_FLAG_ESI */
    uint8_t  reserved;
    uint8_t  data[64];
    uint32_t timestamp_us;
} can_stub_rx_frame_t;

/* Simulated TX echo frame structure */
typedef struct {
    uint32_t ech0_id;
    uint32_t can_id;
    uint8_t  can_dlc;
    uint8_t  channel;
    uint8_t  flags;
    uint8_t  reserved;
    uint8_t  data[64];
    uint32_t timestamp_us;
    uint8_t  pending;
    uint8_t  completed;   /* 1=CAN TX completed (advanced by ISR), 0=not yet completed. Unblocks the FIFO head: echoes may be returned out of FIFO order */
} can_stub_tx_frame_t;

/* Stub CAN API */
/* can_stub_transmit return value: 1=success, 0=error (frame should be dropped),
 * 2=STB full or echo queue full (retryable; the caller must not consume the frame) */
uint8_t can_stub_transmit(uint8_t ch, uint32_t ech0_id, uint32_t can_id, uint8_t can_dlc, uint8_t flags, const uint8_t *data);
uint8_t can_stub_get_tx_echo(can_stub_tx_frame_t *frame);
uint8_t can_stub_peek_tx_echo(can_stub_tx_frame_t *frame);
void can_stub_consume_tx_echo(void);
uint8_t can_stub_get_tx_echo_count(void);
void can_stub_notify_tx_complete(uint8_t ch);  /* Called from CAN TX ISR to advance complete_idx */
uint8_t can_stub_get_rx_frame(can_stub_rx_frame_t *frame);
uint8_t can_stub_peek_rx_frame(can_stub_rx_frame_t *frame);
void can_stub_consume_rx_frame(void);
void can_stub_push_rx_from_isr(int ch, can_rxbuf_type *pRxMsg);
void can_stub_inject_rx_frame(uint8_t ch, uint32_t can_id, uint8_t can_dlc,
                              uint8_t is_fd, uint8_t is_brs, const uint8_t *data);
void can_stub_auto_generate_rx(void);
uint32_t can_stub_get_rx_cnt(uint8_t ch);
uint32_t can_stub_get_tx_cnt(uint8_t ch);
uint8_t can_stub_is_enabled(uint8_t ch);

/* ====== CAN Error Frame API ====== */
/* Forward declaration (full definition in gs_usb_protocol.h) */
struct gs_device_state;

/* Manual bus-off recovery timeout: if auto-recovery doesn't occur within this
 * period, force-reset error counters via BUSOFF=1. AT32 auto-recovery needs
 * 128×11 recessive bits on a quiet bus; a permanently stuck bus never recovers
 * automatically, so this provides a fallback. */
#define CAN_BUSOFF_MANUAL_RECOVERY_MS  1000

/* Pending error ring buffer size per channel. Small to keep ISR overhead low.
 * If buffer overflows (burst errors), oldest entries are dropped. */
#define CAN_PENDING_ERROR_BUF_SIZE  4

/* Minimum interval between non-critical error frames pushed to the RX queue
 * (per channel). Prevents persistent bus errors (e.g., shorted CANH/CANL)
 * from flooding the RX queue and starving normal data frames.
 * CAN_ERR_BUSOFF and CAN_ERR_RESTARTED are exempt (always pushed immediately). */
#define CAN_ERR_FRAME_MIN_INTERVAL_MS  50

/* Per-channel pending error entry (stored by ISR, consumed by main loop). */
typedef struct {
    uint32_t err_type;   /* CAN_ERR_* bits (without CAN_ERR_FLAG) */
    uint8_t  data[8];    /* SocketCAN error payload */
} can_pending_error_t;

/* Store an error frame in the per-channel pending ring buffer (ISR-safe).
 * Called from CANx_ERR_IRQHandler. Writes only to a small SRAM ring buffer.
 * The main loop later calls can_process_pending_error_frames()
 * to push them to the RX queue.
 *   ch: gs_usb channel (0=CAN3, 1=CAN2, 2=CAN1)
 *   err_type: CAN_ERR_* bit(s) OR'd together (without CAN_ERR_FLAG)
 *   data: 8-byte error payload (may be NULL for zeroed data)
 * Only stored when g_can_berr_reporting[ch] is set (host requested BERR_REPORTING) */
void can_err_isr_store(uint8_t ch, uint32_t err_type, const uint8_t data[8]);

/* Main-loop function: drain pending error ring buffer and push to RX queue.
 * Applies rate limiting (CAN_ERR_FRAME_MIN_INTERVAL_MS) to prevent flooding
 * during persistent bus errors. CAN_ERR_BUSOFF and CAN_ERR_RESTARTED are
 * always pushed immediately (critical events).
 * Must be called from the main loop, NOT from ISR.
 * Pushed to the USB RX queue via can_stub_push_error_frame() */
void can_process_pending_error_frames(void);

/* Push a SocketCAN-style error frame into the RX queue (main-loop context).
 *   ch: gs_usb channel (0=CAN3, 1=CAN2, 2=CAN1)
 *   err_type: CAN_ERR_* bit(s) OR'd together (without CAN_ERR_FLAG)
 *   data: 8-byte error payload (matches SocketCAN can_frame.data layout)
 * Only pushes if g_can_berr_reporting[ch] is set (host requested BERR_REPORTING). */
void can_stub_push_error_frame(uint8_t ch, uint32_t err_type, const uint8_t data[8]);

/* Update per-channel CAN controller state cache from ERR ISR.
 * Reads TECNT/RECNT from CAN ERR register and maps EWARN/EPASS/BUSOFF to
 * gs_can_state enum. Called from CANx_ERR_IRQHandler. */
void can_update_error_state(uint8_t ch, can_type *can_x);

/* Fill gs_device_state struct for GET_STATE response.
 *   ch: gs_usb channel (0=CAN3, 1=CAN2, 2=CAN1)
 *   state: output struct to populate */
void can_get_device_state(uint8_t ch, struct gs_device_state *state);

/* Main-loop bus-off recovery handler. Detects auto-recovery (BUSOFF flag
 * cleared by hardware after 128×11 recessive bits) and pushes CAN_ERR_RESTARTED
 * frame. Also performs manual recovery (BUSOFF=1 reset) if auto-recovery
 * doesn't occur within CAN_BUSOFF_MANUAL_RECOVERY_MS. */
void can_handle_busoff_recovery(void);

#endif
