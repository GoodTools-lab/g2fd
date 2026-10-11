/*
 * G2FD - three-channel CANFD gs_usb adapter firmware (AT32F456)
 * Copyright (c) 2026 G2FD contributors
 * SPDX-License-Identifier: MIT
 */

#ifndef __BSP_USB_H__
#define __BSP_USB_H__
#include "at32f45x.h"
#include "usb_conf.h"
#include "usb_core.h"
#include "usbd_int.h"
#include "winusb_class.h"
#include "winusb_desc.h"

void usb_init(void);

#endif
