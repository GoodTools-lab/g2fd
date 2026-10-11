/**
  **************************************************************************
  * @file     winusb_desc.h
  * @brief    usb winusb descriptor header file
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
#ifndef __WINUSB_DESC_H
#define __WINUSB_DESC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "winusb_class.h"
#include "usbd_core.h"

/** @addtogroup AT32F45x_middlewares_usbd_class
  * @{
  */

/** @addtogroup USB_winusb_desc
  * @{
  */

/** @defgroup USB_winusb_desc_definition
  * @{
  */
/**
  * @brief usb vendor id and product id define
  * Own PID assigned under the pid.codes community VID 0x1209 (open-source
  * hardware registry, https://pid.codes/1209/6868/). Windows needs no driver
  * (WinUSB via MS OS 2.0 descriptors); on Linux the gs_usb driver needs the
  * ID added to gs_usb_table (kernel patch) or a temporary
  * `modprobe gs_usb new_id` until the patch lands.
  */
#define USBD_WINUSB_VENDOR_ID               0x1209 /* pid.codes community VID */
#define USBD_WINUSB_PRODUCT_ID              0x6868 /* G2FD, assigned via pid.codes.
                                                    * Must match CANgaroo
                                                    * (GsUsbDriver.cpp / GsUsbProtocol.h). */
/**
  * @brief usb descriptor size define
  */
#define USBD_WINUSB_CONFIG_DESC_SIZE        32
#define USBD_WINUSB_SIZ_STRING_LANGID       4
#define USBD_WINUSB_SIZ_STRING_SERIAL       0x32  /* 50: 3×16-byte serial words + header */

/**
  * @brief usb string define(vendor, product configuration, interface)
  */
#define USBD_WINUSB_DESC_MANUFACTURER_STRING    "GoodTools"
#define USBD_WINUSB_DESC_PRODUCT_STRING         "G2FD"
#define USBD_WINUSB_DESC_CONFIGURATION_STRING   "G2FD Config"
#define USBD_WINUSB_DESC_INTERFACE_STRING       "gs_usb interface"

/**
  * @brief usb mcu id address deine
  */
#define         MCU_ID1                   (0x1FFFF7E8)
#define         MCU_ID2                   (0x1FFFF7EC)
#define         MCU_ID3                   (0x1FFFF7F0)
/**
  * @}
  */

extern usbd_desc_handler winusb_desc_handler;

/* MS OS 2.0 via BOS — see winusb_class.h for rationale */
usbd_desc_t *get_device_winusb_bos(void);            /* BOS descriptor (33 bytes) */
usbd_desc_t *get_device_winusb_msos20_set(void);    /* MS OS 2.0 descriptor set (178 bytes) */


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
