/*
 * G2FD - three-channel CANFD gs_usb adapter firmware (AT32F456)
 * Copyright (c) 2026 G2FD contributors
 * SPDX-License-Identifier: MIT
 */

/**
  **************************************************************************
  * @file     can_ring.c
  * @brief    SRAM-only CAN RX frame queues — open-source lean gs_usb data path
  *
  * Data path (no PSRAM / no DMA / no offline recording):
  *   CAN RX ISR → g_isr_queue (SRAM) ──[main-loop direct copy]──→ g_usb_queue (SRAM) → USB
  *
  * Behavior under heavy load: frames are dropped as soon as a queue fills up,
  * counted in g_rx_queue_overflow (visible to the host).
  * Storage: both queues are placed inside gtl_data_buffer (see gsusb_arena.h).
  **************************************************************************
  */

#include "can_ring.h"
#include "gsusb_arena.h"   /* queue storage placed in gtl_data_buffer */
#include <string.h>

/* ====== Shared storage (formerly the recording-mode GTL ring buffer, now used by the RX queues only) ====== */
uint8_t __attribute__((aligned(4))) gtl_data_buffer[GTL_BUF_SIZE];

/* ====== The two SRAM queues (placement see gsusb_arena.h) ====== */
static volatile uint16_t g_isr_head = 0;
static volatile uint16_t g_isr_tail = 0;
static volatile uint16_t g_isr_count = 0;

/* g_usb_queue: head written by the main loop, tail read by the USB path */
static volatile uint16_t g_usb_head = 0;
static volatile uint16_t g_usb_tail = 0;
static volatile uint16_t g_usb_count = 0;

/* ====== Statistics ====== */
static can_ring_stats_t g_stats;

/* ====== Compatibility variables (referenced via extern in bsp_can.h) ====== */
volatile uint16_t g_rx_count = 0;
volatile uint32_t g_rx_queue_overflow = 0;
volatile uint16_t g_rx_queue_peak = 0;


/* ====== Initialization ====== */
void can_ring_init(void)
{
    g_isr_head = 0;
    g_isr_tail = 0;
    g_isr_count = 0;

    g_usb_head = 0;
    g_usb_tail = 0;
    g_usb_count = 0;

    memset((void *)&g_stats, 0, sizeof(g_stats));

    g_rx_count = 0;
    g_rx_queue_overflow = 0;
    g_rx_queue_peak = 0;
}


/* ====== ISR push (zero blocking) ====== */
void can_ring_push_isr(can_stub_rx_frame_t *frame)
{
    if (g_isr_count >= CAN_RING_QUEUE_SIZE) {
        /* Queue full, drop the frame */
        g_rx_queue_overflow++;
        return;
    }

    /* Disable interrupts to protect head/count (prevents races against the
     * main loop's tail operations).
     * Must use PRIMASK save/restore rather than an unconditional
     * __enable_irq(). Reason: this function may be invoked serially by the
     * same-priority CAN RX ISR, or be called nested; an unconditional
     * re-enable in the inner section would open the outer critical section
     * too early. */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    g_isr_queue[g_isr_head] = *frame;
    g_isr_head = (g_isr_head + 1) % CAN_RING_QUEUE_SIZE;
    g_isr_count++;
    __set_PRIMASK(primask);

    if (g_isr_count > g_stats.isr_queue_peak) {
        g_stats.isr_queue_peak = g_isr_count;
    }
}


/* ====== Background processing (called at high rate from the main loop) ====== */
void can_ring_process(void)
{
    /* Fast path — move frames from g_isr_queue to g_usb_queue */
    while (g_isr_count > 0 && g_usb_count < CAN_RING_QUEUE_SIZE) {
        g_usb_queue[g_usb_head] = g_isr_queue[g_isr_tail];
        g_usb_head = (g_usb_head + 1) % CAN_RING_QUEUE_SIZE;
        g_usb_count++;

        if (g_usb_count > g_stats.usb_queue_peak) {
            g_stats.usb_queue_peak = g_usb_count;
        }

        __disable_irq();
        g_isr_tail = (g_isr_tail + 1) % CAN_RING_QUEUE_SIZE;
        g_isr_count--;
        __enable_irq();

        g_stats.direct_move_count++;
    }

    /* Update the compatibility variables */
    g_rx_count = g_isr_count + g_usb_count;
    if (g_rx_count > g_rx_queue_peak) {
        g_rx_queue_peak = g_rx_count;
    }
}


/* ====== USB call: peek a frame (no consume) ====== */
uint8_t can_ring_peek_usb(can_stub_rx_frame_t *frame)
{
    if (g_usb_count == 0) {
        return 0;
    }
    *frame = g_usb_queue[g_usb_tail];
    return 1;
}


/* ====== USB call: consume a frame (advance tail) ====== */
void can_ring_consume_usb(void)
{
    if (g_usb_count > 0) {
        g_usb_tail = (g_usb_tail + 1) % CAN_RING_QUEUE_SIZE;
        g_usb_count--;
    }
}


/* ====== Statistics access ====== */
uint16_t can_ring_get_isr_count(void)
{
    return g_isr_count;
}

uint16_t can_ring_get_usb_count(void)
{
    return g_usb_count;
}

uint32_t can_ring_get_total_backlog(void)
{
    return (uint32_t)g_isr_count + g_usb_count;
}

const can_ring_stats_t* can_ring_get_stats(void)
{
    return &g_stats;
}
