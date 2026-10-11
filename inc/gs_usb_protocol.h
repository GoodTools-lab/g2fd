/*
 * G2FD - three-channel CANFD gs_usb adapter firmware (AT32F456)
 * Copyright (c) 2026 G2FD contributors
 * SPDX-License-Identifier: MIT
 */

/**
  **************************************************************************
  * @file     gs_usb_protocol.h
  * @brief    gs_usb (candleLight) protocol definitions and handler
  **************************************************************************
  */

#ifndef __GS_USB_PROTOCOL_H__
#define __GS_USB_PROTOCOL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "usb_std.h"
#include "usbd_core.h"

/**
  * @brief gs_usb endpoint addresses (match candleLight)
  */
#define GSUSB_ENDPOINT_IN           0x81
#define GSUSB_ENDPOINT_OUT          0x02

/**
  * @brief gs_usb feature flags
  */
#define GS_CAN_FEATURE_LISTEN_ONLY              (1<<0)
#define GS_CAN_FEATURE_LOOP_BACK                (1<<1)
#define GS_CAN_FEATURE_TRIPLE_SAMPLE            (1<<2)
#define GS_CAN_FEATURE_ONE_SHOT                 (1<<3)
#define GS_CAN_FEATURE_HW_TIMESTAMP             (1<<4)
#define GS_CAN_FEATURE_IDENTIFY                 (1<<5)
#define GS_CAN_FEATURE_USER_ID                  (1<<6)
#define GS_CAN_FEATURE_PAD_PKTS_TO_MAX_PKT_SIZE (1<<7)
#define GS_CAN_FEATURE_FD                       (1<<8)
#define GS_CAN_FEATURE_REQ_USB_QUIRK_LPC546XX   (1<<9)
#define GS_CAN_FEATURE_BT_CONST_EXT             (1<<10)
#define GS_CAN_FEATURE_TERMINATION              (1<<11)
#define GS_CAN_FEATURE_BERR_REPORTING           (1<<12)
#define GS_CAN_FEATURE_GET_STATE                (1<<13)
#define GS_CAN_FEATURE_FILTER                   (1<<16)

/**
  * @brief gs_usb request codes (matching candleLight gs_usb.h)
  */
enum gs_usb_breq {
    GS_USB_BREQ_HOST_FORMAT = 0,
    GS_USB_BREQ_BITTIMING,
    GS_USB_BREQ_MODE,
    GS_USB_BREQ_BERR,           /* NOT handled by candleLight firmware */
    GS_USB_BREQ_BT_CONST,
    GS_USB_BREQ_DEVICE_CONFIG,
    GS_USB_BREQ_TIMESTAMP,
    GS_USB_BREQ_IDENTIFY,
    GS_USB_BREQ_GET_USER_ID,    /* not implemented */
    GS_USB_BREQ_SET_USER_ID,    /* not implemented */
    GS_USB_BREQ_DATA_BITTIMING,
    GS_USB_BREQ_BT_CONST_EXT,
    GS_USB_BREQ_SET_TERMINATION,
    GS_USB_BREQ_GET_TERMINATION,
    GS_USB_BREQ_GET_STATE,
    GS_USB_BREQ_SET_FILTER,
    GS_USB_BREQ_GET_FILTER,
    __GS_USB_BREQ_PLACEHOLDER_17,
    __GS_USB_BREQ_PLACEHOLDER_18,
    __GS_USB_BREQ_PLACEHOLDER_19,
    GS_USB_BREQ_ELM_GET_BOARDINFO = 20,
    GS_USB_BREQ_ELM_SET_FILTER,
    GS_USB_BREQ_ELM_GET_LASTERROR,
    GS_USB_BREQ_ELM_SET_BUSLOADREPORT,
    GS_USB_BREQ_ELM_SET_PINSTATUS,
    GS_USB_BREQ_ELM_GET_PINSTATUS,
    GS_USB_BREQ_ELM_READ_FLASH = 26,
    GS_USB_BREQ_ELM_WRITE_FLASH,
    GS_USB_BREQ_MAX = 28,       /* GS_USB_BREQ_ELM_WRITE_FLASH + 1 */
};

/**
  * @brief gs_usb CAN mode
  */
enum gs_can_mode {
    GS_CAN_MODE_RESET = 0,
    GS_CAN_MODE_START
};

/* Mode flags (OR'd into gs_device_mode.flags by host) */
#define GS_CAN_MODE_NORMAL              0
#define GS_CAN_MODE_LISTEN_ONLY         (1 << 0)
#define GS_CAN_MODE_LOOP_BACK           (1 << 1)
#define GS_CAN_MODE_TRIPLE_SAMPLE       (1 << 2)
#define GS_CAN_MODE_ONE_SHOT            (1 << 3)
#define GS_CAN_MODE_HW_TIMESTAMP        (1 << 4)
#define GS_CAN_MODE_PAD_PKTS_TO_MAX_PKT_SIZE (1 << 7)
#define GS_CAN_MODE_FD                  (1 << 8)
#define GS_CAN_MODE_BERR_REPORTING      (1 << 12)

enum gs_can_state {
    GS_CAN_STATE_ERROR_ACTIVE = 0,
    GS_CAN_STATE_ERROR_WARNING,
    GS_CAN_STATE_ERROR_PASSIVE,
    GS_CAN_STATE_BUS_OFF,
    GS_CAN_STATE_STOPPED,
    GS_CAN_STATE_SLEEPING
};

/**
  * @brief SocketCAN CAN ID flags (match Linux kernel can.h)
  *   Error frames use can_id = CAN_ERR_FLAG | <error type bits>
  */
#define CAN_EFF_FLAG    0x80000000U
#define CAN_RTR_FLAG    0x40000000U
#define CAN_ERR_FLAG    0x20000000U
#define CAN_EFF_MASK    0x1FFFFFFFU

/**
  * @brief SocketCAN error type codes (OR'd with CAN_ERR_FLAG in can_id)
  *   Match Linux kernel can/error.h and CANgaroo GsUsbProtocol.h
  */
#define CAN_ERR_LOSTARB     0x00000002U  /* arbitration lost (data[0]=location) */
#define CAN_ERR_CRTL        0x00000004U  /* controller state change (data[1]) */
#define CAN_ERR_PROT        0x00000008U  /* protocol error (data[2]=type, data[3]=loc) */
#define CAN_ERR_ACK         0x00000020U  /* ACK error */
#define CAN_ERR_BUSOFF      0x00000040U  /* bus-off entered */
#define CAN_ERR_BUSERROR    0x00000080U  /* bus error (generic) */
#define CAN_ERR_RESTARTED   0x00000100U  /* bus-off recovery completed */

/* CAN_ERR_CRTL sub-bits (data[1]) */
#define CAN_ERR_CRTL_RX_WARNING  0x01
#define CAN_ERR_CRTL_TX_WARNING  0x02
#define CAN_ERR_CRTL_RX_PASSIVE  0x04
#define CAN_ERR_CRTL_TX_PASSIVE  0x08

/* CAN_ERR_PROT type sub-bits (data[2]) */
#define CAN_ERR_PROT_BIT     0x01
#define CAN_ERR_PROT_FORM    0x02
#define CAN_ERR_PROT_STUFF   0x04
/* CAN_ERR_PROT location sub-bits (data[3]) */
#define CAN_ERR_PROT_LOC_CRC 0x18

enum gs_can_termination_state {
    GS_CAN_TERMINATION_STATE_OFF = 0,
    GS_CAN_TERMINATION_STATE_ON,
    GS_CAN_TERMINATION_UNSUPPORTED = 0x7FFFFFFF,
};

/**
 * @brief Active CAN channels exposed via gs_usb (icount + 1)
 */
#define GS_USB_ACTIVE_CHANNELS    3

/**
 * @brief Max TX URBs (slot count) - must match upper computer GS_MAX_TX_URBS
 * Upper computer (cangaroo) uses 40 slots for TX echo tracking (echo_id 0-39)
 */
#define GS_MAX_TX_URBS            40

/**
  * @brief Device capability flags (candleLight BOARD_budgetcan FDCAN)
  *   BERR_REPORTING: device pushes SocketCAN-style error frames on CAN bus errors
  *   GET_STATE: device supports GS_USB_BREQ_GET_STATE to query CAN controller state
  */
#define GS_USB_DEV_FEATURES       (GS_CAN_FEATURE_LISTEN_ONLY | \
                                     GS_CAN_FEATURE_LOOP_BACK | \
                                     GS_CAN_FEATURE_ONE_SHOT | \
                                     GS_CAN_FEATURE_HW_TIMESTAMP | \
                                     GS_CAN_FEATURE_IDENTIFY | \
                                     GS_CAN_FEATURE_FD | \
                                     GS_CAN_FEATURE_BT_CONST_EXT | \
                                     GS_CAN_FEATURE_BERR_REPORTING | \
                                     GS_CAN_FEATURE_GET_STATE)

/**
  * @brief gs_usb data structures (match candleLight gs_usb.h layout)
  */
struct gs_host_config {
    uint32_t byte_order;
} __attribute__((packed, aligned(4)));

struct gs_device_config {
    uint8_t reserved1;
    uint8_t reserved2;
    uint8_t reserved3;
    uint8_t icount;
    uint32_t sw_version;
    uint32_t hw_version;
} __attribute__((packed, aligned(4)));

struct gs_device_mode {
    uint32_t mode;
    uint32_t flags;
} __attribute__((packed, aligned(4)));

/**
  * @brief CAN FD DLC to data length conversion (array lookup)
  * DLC 0-8: direct mapping, DLC 9-15: 12,16,20,24,32,48,64
  */
static inline uint8_t can_fd_dlc2len(uint8_t dlc)
{
    static const uint8_t dlc2len[16] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};
    return dlc2len[dlc & 0x0F];
}

/**
  * @brief CAN FD data length to DLC conversion (array lookup)
  * Input: actual byte count (0-64), Output: DLC encoding (0-15)
  */
static inline uint8_t can_fd_len2dlc(uint8_t len)
{
    static const uint8_t len2dlc[65] = {
        0, 1, 2, 3, 4, 5, 6, 7, 8,       /* 0-8 → DLC 0-8 */
        9, 9, 9, 9,                       /* 9-12 → DLC 9 */
        10, 10, 10, 10,                   /* 13-16 → DLC 10 */
        11, 11, 11, 11,                   /* 17-20 → DLC 11 */
        12, 12, 12, 12,                   /* 21-24 → DLC 12 */
        13, 13, 13, 13, 13, 13, 13, 13,   /* 25-32 → DLC 13 */
        14, 14, 14, 14, 14, 14, 14, 14,
        14, 14, 14, 14, 14, 14, 14, 14,   /* 33-48 → DLC 14 */
        15, 15, 15, 15, 15, 15, 15, 15,
        15, 15, 15, 15, 15, 15, 15, 15    /* 49-64 → DLC 15 */
    };
    return (len <= 64) ? len2dlc[len] : 15;
}

struct gs_device_state {
    uint32_t state;
    uint32_t rxerr;
    uint32_t txerr;
} __attribute__((packed, aligned(4)));

struct gs_device_bittiming {
    uint32_t prop_seg;
    uint32_t phase_seg1;
    uint32_t phase_seg2;
    uint32_t sjw;
    uint32_t brp;
} __attribute__((packed, aligned(4)));

struct can_bittiming_const {
    uint32_t tseg1_min;
    uint32_t tseg1_max;
    uint32_t tseg2_min;
    uint32_t tseg2_max;
    uint32_t sjw_max;
    uint32_t brp_min;
    uint32_t brp_max;
    uint32_t brp_inc;
} __attribute__((packed, aligned(4)));

struct gs_device_bt_const {
    uint32_t feature;
    uint32_t fclk_can;
    struct can_bittiming_const btc;
} __attribute__((packed, aligned(4)));

struct gs_device_bt_const_extended {
    uint32_t feature;
    uint32_t fclk_can;
    struct can_bittiming_const btc;
    struct can_bittiming_const dbtc;
} __attribute__((packed, aligned(4)));

struct gs_device_berr {
    uint32_t berr;
} __attribute__((packed, aligned(4)));

struct gs_identify_mode {
    uint32_t mode;
} __attribute__((packed, aligned(4)));

/**
  * @brief EP0 buffer union (match candleLight ep0 union design)
  * Used to receive data from host for OUT control requests
  */
union gs_usb_ep0_buf {
    uint32_t identify;
    struct gs_host_config host_config;
    struct gs_device_bittiming bittiming;
    struct gs_device_bittiming data_bittiming;
    struct gs_device_mode mode;
    struct gs_device_berr berr;
    struct gs_identify_mode identify_mode;
    uint8_t raw[64];  /* >= largest gs_usb EP0 payload */
};

/**
  * @brief gs_usb handler state
  */
typedef struct {
    uint32_t sof_timestamp_us;
    uint8_t current_channel;   /* channel index from wValue for per-channel requests */
    uint8_t can_enabled[GS_USB_ACTIVE_CHANNELS];
    uint32_t feature[GS_USB_ACTIVE_CHANNELS];
    struct gs_device_bittiming current_timing[GS_USB_ACTIVE_CHANNELS];
    struct gs_device_bittiming current_data_timing[GS_USB_ACTIVE_CHANNELS];
} gs_usb_state_t;

/**
  * @brief gs_usb CAN frame flags (match candleLight gs_usb.h)
  */
#define GS_CAN_FLAG_OVERFLOW            (1<<0)
#define GS_CAN_FLAG_FD                  (1<<1) /* is a CAN-FD frame */
#define GS_CAN_FLAG_BRS                 (1<<2) /* bit rate switch (for CAN-FD frames) */
#define GS_CAN_FLAG_ESI                 (1<<3) /* error state indicator (for CAN-FD frames) */

/**
  * @brief gs_usb host frame structure (match candleLight gs_host_frame layout)
  * Classic CAN without timestamp: 20 bytes (16 header + 4 data)
  * Classic CAN with timestamp: 24 bytes (16 header + 8 data)
  * CAN FD without timestamp: 76 bytes (16 header + 64 data)
  * CAN FD with timestamp: 80 bytes (16 header + 64 data + 4 timestamp)
  */
struct gs_host_frame {
    uint32_t echo_id;
    uint32_t can_id;
    uint8_t  can_dlc;
    uint8_t  channel;
    uint8_t  flags;
    uint8_t  reserved;
    uint8_t  data[64];
    uint32_t timestamp_us;
} __attribute__((packed, aligned(4)));

/**
  * @brief Classic CAN frame layout (with timestamp: 24 bytes, without: 20 bytes)
  * NOTE: timestamp is at offset 20 (after data[8]), different from gs_host_frame!
  */
struct gs_host_frame_classic {
    uint32_t echo_id;
    uint32_t can_id;
    uint8_t  can_dlc;
    uint8_t  channel;
    uint8_t  flags;
    uint8_t  reserved;
    uint8_t  data[8];
    uint32_t timestamp_us;
} __attribute__((packed, aligned(4)));

/* Frame size constants */
#define GS_USB_FRAME_CLASSIC_NO_TS  20
#define GS_USB_FRAME_CLASSIC_TS     24
#define GS_USB_FRAME_FD_NO_TS       76
#define GS_USB_FRAME_FD_TS          80

/* ====== gs_usb mode queue geometry (layout depends on gsusb_arena.h, do not move back into the .c) ======
 * GS_USB_TX_BUF_SIZE = FD frame capacity: each ping-pong buffer holds 28 x 80-byte frames (2240B),
 * 2240B = 35 x 64B, covering 2 USB frames (2ms, up to 38 bulk transactions) */
#define GS_USB_TX_BUF_SIZE          (GS_USB_FRAME_FD_TS * USB_BULK_TX_FD_COUNT)

/**
  * @brief gs_usb handler functions
  */
void gs_usb_init(void);
uint8_t gs_usb_handle_setup(void *udev, usb_setup_type *setup);
uint8_t gs_usb_handle_ep0_rx(void *udev);
uint8_t gs_usb_is_pending_request(void);

/**
  * @brief Bulk endpoint frame processing (USB <-> CAN bridge)
  * gs_usb_bulk_rx_notify returns 0 if TX ring has space (OUT endpoint can
  * be re-armed), or 1 if TX ring is full (OUT endpoint should NAK).
  */
void gs_usb_process_bulk_out(void *udev);
void gs_usb_process_bulk_in(void *udev);
uint8_t gs_usb_bulk_rx_notify(const uint8_t *data, uint16_t len);

/**
  * @brief CAN bittiming application callback (implemented in bsp_can.c)
  */
void gs_usb_apply_bittiming(uint8_t ch, const struct gs_device_bittiming *timing);
void gs_usb_apply_data_bittiming(uint8_t ch, const struct gs_device_bittiming *timing);
void gs_usb_can_enable(uint8_t ch, uint32_t feature);
void gs_usb_can_disable(uint8_t ch);
void gs_usb_update_sof_timestamp(uint32_t timestamp_us);

#ifdef __cplusplus
}
#endif

#endif /* __GS_USB_PROTOCOL_H__ */
