/*
 * G2FD - three-channel CANFD gs_usb adapter firmware (AT32F456)
 * Copyright (c) 2026 G2FD contributors
 * SPDX-License-Identifier: MIT
 */

#ifndef __CAN_RING_H__
#define __CAN_RING_H__

#include "bsp_can.h"

/* ====== SRAM-only CAN RX frame queues (open-source lean edition) ======
 *
 * Data path (no PSRAM, no DMA):
 *   CAN RX ISR → g_isr_queue (SRAM) ──[main-loop direct copy]──→ g_usb_queue (SRAM) → USB
 *
 * Under heavy load, frames are dropped as soon as a queue fills up: overflow
 * of g_isr_queue is counted in g_rx_queue_overflow; g_usb_queue in theory
 * never fills up (the direct copy only runs when a free slot exists).
 *
 * Storage: g_isr_queue/g_usb_queue are placed inside gtl_data_buffer
 * (see gsusb_arena.h); gtl_data_buffer itself is defined in can_ring.c. */

/* Depth of each SRAM queue (in frames) */
#define CAN_RING_QUEUE_SIZE     128

/* ====== Statistics ====== */
typedef struct {
    volatile uint32_t isr_queue_peak;       /* Historical high watermark of g_isr_queue */
    volatile uint32_t usb_queue_peak;       /* Historical high watermark of g_usb_queue */
    volatile uint32_t direct_move_count;    /* Frames moved on the fast-path direct copy */
} can_ring_stats_t;

/* ====== API ====== */

/* Initialize the queues (call after power-on / software reset) */
void can_ring_init(void);

/* ISR context: push a frame into g_isr_queue (zero blocking, PRIMASK
 * critical section, drop + count when the queue is full) */
void can_ring_push_isr(can_stub_rx_frame_t *frame);

/* Background processing (called at high rate from the main loop, before
 * gs_usb_process_bulk_in): moves frames from g_isr_queue to g_usb_queue */
void can_ring_process(void);

/* USB context: peek a frame from g_usb_queue (without consuming it) */
uint8_t can_ring_peek_usb(can_stub_rx_frame_t *frame);

/* USB context: consume the frame in g_usb_queue (advance tail) */
void can_ring_consume_usb(void);

/* ====== Statistics access ====== */
uint16_t can_ring_get_isr_count(void);
uint16_t can_ring_get_usb_count(void);
uint32_t can_ring_get_total_backlog(void);
const can_ring_stats_t* can_ring_get_stats(void);

/* ====== Compatibility variables (extern in bsp_can.h, defined in can_ring.c) ======
 * g_rx_count: kept for legacy code, = g_isr_count + g_usb_count */
extern volatile uint16_t g_rx_count;
extern volatile uint32_t g_rx_queue_overflow;
extern volatile uint16_t g_rx_queue_peak;

#endif /* __CAN_RING_H__ */
