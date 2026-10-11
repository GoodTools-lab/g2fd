/**
  **************************************************************************
  * @file     winusb_class.h
  * @brief    usb winusb class file
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

 /* define to prevent recursive inclusion -------------------------------------*/
#ifndef __WINUSB_CLASS_H
#define __WINUSB_CLASS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "usb_std.h"
#include "usbd_core.h"

/** @addtogroup AT32F45x_middlewares_usbd_class
  * @{
  */

/** @addtogroup USB_winusb_class
  * @{
  */

/** @defgroup USB_winusb_class_definition
  * @{
  */

/**
  * @brief usb use endpoint define
  * Use gs_usb (candleLight) endpoint addresses for compatibility
  */
#define USBD_WINUSB_BULK_IN_EPT             0x81
#define USBD_WINUSB_BULK_OUT_EPT            0x02

/**
  * @brief usb in and out max packet size define
  */

#define USBD_FS_WINUSB_MAXPACKET_SIZE        0x40
#define USBD_WINUSB_IN_MAXPACKET_SIZE        USBD_FS_WINUSB_MAXPACKET_SIZE
#define USBD_WINUSB_OUT_MAXPACKET_SIZE       USBD_FS_WINUSB_MAXPACKET_SIZE

#define WINUSB_BMS_VENDOR_CODE            0x21  /* changed from 0x20 to force Windows MS OS descriptor cache invalidation */

/* ====== USB 2.1 / BOS / MS OS 2.0 descriptors ======
 * Windows 10+ caches MS OS 1.0 string descriptor (index 0xEE) by VID/PID/serial.
 * Once it sees a device without MS OS 1.0 string, it never re-queries, even if
 * the device firmware is updated to add the descriptor. The reliable bypass is
 * MS OS 2.0 via BOS: declare bcdUSB=0x0210, expose a BOS descriptor (standard
 * type 0x0F) with a Microsoft OS 2.0 platform capability, and Windows MUST
 * query BOS on every enumeration (per USB 2.0 spec — no caching for BOS). */

#define USB_DESCIPTOR_TYPE_BOS             0x0F  /* BOS descriptor type */
#define USB_DESCIPTOR_TYPE_DEVICE_CAP      0x10  /* Device Capability descriptor type */

/* MS OS 2.0 vendor request wIndex values (Microsoft OS 2.0 spec) */
#define MSOS20_DESC_SET_INDEX              0x0007  /* Get MS OS 2.0 Descriptor Set */
#define MSOS20_CFG_SUBSET_INDEX            0x0008  /* Get Configuration Subset (alt enum) */
#define MSOS20_FUNC_SUBSET_INDEX           0x0009  /* Get Function Subset (alt enum) */

/* MS OS 2.0 Descriptor Set total length:
 *   Set Header (10) + Config Subset Header (8) +
 *   Func0 Subset (8 + CompatID 20 + RegProp 132 = 160) = 178 bytes
 * NOTE: MS OS 2.0 Compatible ID descriptor is 20 bytes (wLength=0x14),
 * NOT 24 bytes. Layout per Microsoft spec:
 *   wLength(2) + wDescriptorType(2) + CompatibleID(8) + SubCompatibleID(8) = 20 */
#define WINUSB20_DESC_SET_SIZE             0x00B2
#define WINUSB20_CFG_SUBSET_SIZE           0x00A8  /* 8 + 160 = 168 */
#define WINUSB20_FUNC_SUBSET_SIZE          0x00A0  /* 8 + 20 + 132 = 160 */

/**
  * @}
  */

/** @defgroup USB_winusb_class_exported_types
  * @{
  */

/**
  * @brief usb winusb class struct
  */
typedef struct
{
  uint32_t alt_setting;
  uint8_t *g_rx_buff;
  uint16_t g_len, g_rxlen;
  __IO uint8_t g_tx_completed, g_rx_completed;
  uint32_t maxpacket;
}winusb_struct_type;


/**
  * @}
  */

/** @defgroup USB_winusb_class_exported_functions
  * @{
  */
extern usbd_class_handler winusb_class_handler;
error_status usb_winusb_send_data(void *udev, uint8_t *send_data, uint16_t len);

/* Standalone Bulk IN TX completion flag - isolated from winusb_struct_type
 * to prevent struct corruption from affecting Bulk IN state.
 * g_tx_completed in the struct is no longer used for Bulk IN. */
extern volatile uint8_t g_bulk_in_tx_completed;

/* ====== Bulk OUT endpoint NAK flow control ======
 * When TX ring is full, the OUT endpoint is paused (not re-armed).
 * USB hardware automatically NAKs host OUT requests, causing the host
 * controller to retry in the next USB frame (1ms for FS).
 * This provides backpressure without dropping frames.
 * When ring has space again, usb_winusb_resume_out() re-arms the endpoint. */
extern volatile uint8_t g_usb_out_paused;   /* 1 = OUT endpoint paused (NAK) */
void usb_winusb_resume_out(void *udev);      /* Re-arm OUT endpoint after pause */

/* ====== USB link alive detection (SOF based) ======
 * USB_VBUS_IGNORE is defined, so VBUS session-end detection is compiled out
 * and class_clear_handler is NEVER called on cable unplug. Detect detach by
 * SOF silence instead: full-speed host emits SOF every 1ms; if it stops,
 * the cable is unplugged or the host suspended. Main loop polls
 * usb_link_alive() and falls back to recording mode (applies CAN filter). */
uint8_t usb_link_alive(void);                /* 1 = SOF seen within timeout */
void usb_link_traffic_seen(void);            /* ISR context: bulk TX/RX refreshes the alive timestamp (complements SOF) */
void winusb_bulk_in_watchdog(void *udev);      /* Bulk IN completion-event watchdog (called every main loop pass, with hardware classification) */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
#ifdef __cplusplus
}
#endif

#endif




