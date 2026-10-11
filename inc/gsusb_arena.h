/*
 * G2FD - three-channel CANFD gs_usb adapter firmware (AT32F456)
 * Copyright (c) 2026 G2FD contributors
 * SPDX-License-Identifier: MIT
 */

#ifndef __GSUSB_ARENA_H__
#define __GSUSB_ARENA_H__

/* ====== gs_usb large buffers share the gtl_data_buffer address space (arena) ======
 *
 * gtl_data_buffer (64KB) is defined in can_ring.c and is addressed in common
 * by the major gs_usb arrays (originally the GTL ring buffer of the offline
 * recording mode; after the recording subsystem was removed, only the RX/USB
 * queues still use it).
 * This header maps each large array to a fixed offset inside gtl_data_buffer
 * via "pointer-to-array macros"; the access syntax and sizeof semantics are
 * identical to the original static arrays, so each .c file merely removes its
 * array definition and includes this header.
 *
 * Fallback: for test builds, each .c keeps its #ifndef GSUSB_USE_ARENA static
 * array branch (shrink GTL_BUF_SIZE until the arena no longer fits, so the
 * conditional compilation takes the static-array path).
 */

#include "bsp_can.h"
#include "can_ring.h"
#include "gs_usb_protocol.h"

extern uint8_t gtl_data_buffer[];

/* Shared storage size (originally defined in CAN_RecordProcess.h, moved here
 * after the recording subsystem was removed) */
#define GTL_BUF_SIZE            0x10000U    /* 64KB */

/* gs_usb-mode only: queue depth definitions moved up from bsp_can.c /
 * gs_usb_protocol.c */
#define CAN_STUB_TX_QUEUE_SIZE      128     /* Depth of g_tx_queue (echo wait queue) */
#define GS_USB_TX_RING_SIZE         128     /* Host->CAN TX ring queue depth */
#define GS_USB_TX_RING_SLOT_SIZE    (GS_USB_FRAME_FD_TS)  /* 80B per slot, fits FD+TS */
#define USB_BULK_TX_FD_COUNT        28      /* FD frames carried per ping-pong buffer */
#define USB_BULK_TX_MAX_FRAMES      (GS_USB_TX_BUF_SIZE / GS_USB_FRAME_CLASSIC_TS)

/* ====== Arena offset layout (all 4-byte aligned, ~47.9KB / 64KB used) ======
 * Note: offsets keep the historical layout (the former PSRAM assembly buffer
 * region 0x4C00-0x5400 is deliberately left vacant to avoid arithmetic errors
 * from a re-layout); remaining ~17.4KB is reserved */
#define GSUSB_OFF_ISR_Q             0x0000  /* g_isr_queue            128*76 = 9728 */
#define GSUSB_OFF_USB_Q             0x2600  /* g_usb_queue            128*76 = 9728 */
#define GSUSB_OFF_PSRAM_WBUF        0x4C00  /* (vacant) former PSRAM assembly buffer, 1024 */
#define GSUSB_OFF_PSRAM_RBUF        0x5000  /* (vacant) former PSRAM read buffer, 1024 */
#define GSUSB_OFF_TXQ               0x5400  /* g_tx_queue             128*84 = 10752 */
#define GSUSB_OFF_TX_RING           0x7EC0  /* gs_usb_tx_ring         128*80 = 10240 */
#define GSUSB_OFF_TX_RING_LEN       0xA6C0  /* gs_usb_tx_ring_frame_len 128*2 = 256 */
#define GSUSB_OFF_BULK_TX_A         0xA7C0  /* gs_usb_bulk_tx_buf_a   2240 */
#define GSUSB_OFF_BULK_TX_B         0xB080  /* gs_usb_bulk_tx_buf_b   2240 */
#define GSUSB_OFF_RXACCUM80         0xBA00  /* (deprecated) former 80B accumulation buffer, now reserved */
#define GSUSB_OFF_ACCUM             0xBAC0  /* gs_usb_rx_accum_buf    160 (USB packet accumulation) */
#define GSUSB_ARENA_END             0xBB60  /* Total footprint after alignment 47968B, ~17.4KB reserved */

/* USB OUT packet accumulation buffer capacity: must fit a full 76B frame +
 * bytes retained from the previous round while the ring was full + a 64B packet */
#define GSUSB_ACCUM_SIZE            160

#if (GTL_BUF_SIZE >= GSUSB_ARENA_END)

#define GSUSB_USE_ARENA 1

/* Pointer-to-array macros: preserve all array semantics (sizeof, decay,
 * subscripting, ...) */
#define g_isr_queue \
    (*(can_stub_rx_frame_t (*)[CAN_RING_QUEUE_SIZE])&gtl_data_buffer[GSUSB_OFF_ISR_Q])
#define g_usb_queue \
    (*(can_stub_rx_frame_t (*)[CAN_RING_QUEUE_SIZE])&gtl_data_buffer[GSUSB_OFF_USB_Q])
#define g_tx_queue \
    (*(can_stub_tx_frame_t (*)[CAN_STUB_TX_QUEUE_SIZE])&gtl_data_buffer[GSUSB_OFF_TXQ])
#define gs_usb_tx_ring \
    (*(uint8_t (*)[GS_USB_TX_RING_SIZE][GS_USB_TX_RING_SLOT_SIZE])&gtl_data_buffer[GSUSB_OFF_TX_RING])
#define gs_usb_tx_ring_frame_len \
    (*(volatile uint16_t (*)[GS_USB_TX_RING_SIZE])&gtl_data_buffer[GSUSB_OFF_TX_RING_LEN])
#define gs_usb_bulk_tx_buf_a \
    (*(uint8_t (*)[GS_USB_TX_BUF_SIZE])&gtl_data_buffer[GSUSB_OFF_BULK_TX_A])
#define gs_usb_bulk_tx_buf_b \
    (*(uint8_t (*)[GS_USB_TX_BUF_SIZE])&gtl_data_buffer[GSUSB_OFF_BULK_TX_B])
#define gs_usb_rx_accum_buf \
    (*(uint8_t (*)[GSUSB_ACCUM_SIZE])&gtl_data_buffer[GSUSB_OFF_ACCUM])

/* Compile-time assertions: arena footprint stays within bounds + key offsets
 * are 4-byte aligned */
typedef char gsusb_arena_size_check[(GSUSB_ARENA_END <= GTL_BUF_SIZE) ? 1 : -1];
typedef char gsusb_arena_align_check[
    ((GSUSB_OFF_ISR_Q | GSUSB_OFF_USB_Q | GSUSB_OFF_TXQ |
      GSUSB_OFF_TX_RING | GSUSB_OFF_TX_RING_LEN | GSUSB_OFF_BULK_TX_A |
      GSUSB_OFF_BULK_TX_B | GSUSB_OFF_RXACCUM80 |
      GSUSB_OFF_ACCUM) % 4) == 0 ? 1 : -1];

#endif /* GTL_BUF_SIZE >= GSUSB_ARENA_END */

#endif /* __GSUSB_ARENA_H__ */
