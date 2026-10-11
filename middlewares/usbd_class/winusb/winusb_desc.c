/**
  **************************************************************************
  * @file     winusb_desc.c
  * @brief    usb winusb device descriptor
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
#include "stdio.h"
#include "usb_std.h"
#include "usbd_sdr.h"
#include "usbd_core.h"
#include "winusb_desc.h"
/** @addtogroup AT32F45x_middlewares_usbd_class
  * @{
  */

/** @defgroup USB_winusb_desc
  * @brief usb device winusb descriptor
  * @{
  */

/** @defgroup USB_winusb_desc_private_functions
  * @{
  */

static usbd_desc_t *get_device_descriptor(void);
static usbd_desc_t *get_device_qualifier(void);
static usbd_desc_t *get_device_configuration(void);
static usbd_desc_t *get_device_other_speed(void);
static usbd_desc_t *get_device_lang_id(void);
static usbd_desc_t *get_device_manufacturer_string(void);
static usbd_desc_t *get_device_product_string(void);
static usbd_desc_t *get_device_serial_string(void);
static usbd_desc_t *get_device_interface_string(void);
static usbd_desc_t *get_device_config_string(void);

static uint16_t usbd_unicode_convert(uint8_t *string, uint8_t *unicode_buf);
static void usbd_int_to_unicode (uint32_t value , uint8_t *pbuf , uint8_t len);
static void get_serial_num(void);

#if (USBD_SUPPORT_WINUSB == 1)
static usbd_desc_t *get_device_winusb_os_string(void);
static usbd_desc_t *get_device_winusb_os_feature(void);
static usbd_desc_t *get_device_winusb_os_property(void);
#endif

#if defined ( __ICCARM__ ) /* iar compiler */
  #pragma data_alignment=4
#endif
ALIGNED_HEAD static uint8_t g_usbd_desc_buffer[256] ALIGNED_TAIL;

/**
  * @brief device descriptor handler structure
  */
usbd_desc_handler winusb_desc_handler =
{
  get_device_descriptor,
  get_device_qualifier,
  get_device_configuration,
  get_device_other_speed,
  get_device_lang_id,
  get_device_manufacturer_string,
  get_device_product_string,
  get_device_serial_string,
  get_device_interface_string,
  get_device_config_string,
#if (USBD_SUPPORT_WINUSB == 1)
  get_device_winusb_os_string,
  get_device_winusb_os_feature,
  get_device_winusb_os_property
#endif
};

/**
  * @brief usb device standard descriptor
  */
#if defined ( __ICCARM__ ) /* iar compiler */
  #pragma data_alignment=4
#endif
ALIGNED_HEAD static uint8_t g_usbd_descriptor[USB_DEVICE_DESC_LEN] ALIGNED_TAIL =
{
  USB_DEVICE_DESC_LEN,                   /* bLength */
  USB_DESCIPTOR_TYPE_DEVICE,             /* bDescriptorType */
  0x10,                                  /* bcdUSB = 0x0210 (USB 2.1) — required to enable BOS
                                          * descriptor query. Without BOS, Windows 10+ uses
                                          * cached MS OS 1.0 string state and never re-queries,
                                          * which prevents dual-interface WinUSB enumeration. */
  0x02,
  0x00,                                  /* bDeviceClass */
  0x00,                                  /* bDeviceSubClass */
  0x00,                                  /* bDeviceProtocol */
  USB_MAX_EP0_SIZE,                      /* bMaxPacketSize */
  LBYTE(USBD_WINUSB_VENDOR_ID),          /* idVendor */
  HBYTE(USBD_WINUSB_VENDOR_ID),          /* idVendor */
  LBYTE(USBD_WINUSB_PRODUCT_ID),         /* idProduct */
  HBYTE(USBD_WINUSB_PRODUCT_ID),         /* idProduct */
  0x01,                                  /* bcdDevice low byte = 0x0201 (VID/PID switch to 0x1209:0x6868,
                                            * forces Windows re-enumeration) */
  0x02,                                  /* bcdDevice high byte */
  USB_MFC_STRING,                        /* Index of manufacturer string */
  USB_PRODUCT_STRING,                    /* Index of product string */
  USB_SERIAL_STRING,                     /* Index of serial number string */
  1                                      /* bNumConfigurations */
};

/**
  * @brief usb configuration standard descriptor
  */
#if defined ( __ICCARM__ ) /* iar compiler */
  #pragma data_alignment=4
#endif
ALIGNED_HEAD static uint8_t g_usbd_configuration[USBD_WINUSB_CONFIG_DESC_SIZE] ALIGNED_TAIL =
{
  USB_DEVICE_CFG_DESC_LEN,               /* bLength: configuration descriptor size */
  USB_DESCIPTOR_TYPE_CONFIGURATION,      /* bDescriptorType: configuration */
  LBYTE(USBD_WINUSB_CONFIG_DESC_SIZE),   /* wTotalLength: bytes returned */
  HBYTE(USBD_WINUSB_CONFIG_DESC_SIZE),   /* wTotalLength: bytes returned */
  0x01,                                  /* bNumInterfaces: 1 interface (gs_usb CAN) */
  0x01,                                  /* bConfigurationValue: configuration value */
  0x00,                                  /* iConfiguration: no string descriptor (per candleLight protocol) */
  0x80,                                  /* bmAttributes: bus powered (match candleLight) */
  0x4B,                                  /* MaxPower 150 mA: match candleLight */
  
  /* ---- Interface 0: GS USB CAN ---- */
  USB_DEVICE_IF_DESC_LEN,                /* bLength: interface descriptor size */
  USB_DESCIPTOR_TYPE_INTERFACE,          /* bDescriptorType: interface descriptor type */
  0x00,                                  /* bInterfaceNumber: number of interface */
  0x00,                                  /* bAlternateSetting: alternate set */
  0x02,                                  /* bNumEndpoints: number of endpoints */
  0xff,                                  /* bInterfaceClass: Vendor specific */
  0xff,                                  /* bInterfaceSubClass: Vendor specific */
  0xff,                                  /* bInterfaceProtocol: Vendor specific */
  0x00,                                  /* iInterface: index of string descriptor */

  USB_DEVICE_EPT_LEN,                    /* bLength: size of endpoint descriptor in bytes */
  USB_DESCIPTOR_TYPE_ENDPOINT,           /* bDescriptorType: endpoint descriptor type */
  USBD_WINUSB_BULK_OUT_EPT,              /* bEndpointAddress: OUT endpoint */
  USB_EPT_DESC_BULK,                     /* bmAttributes: endpoint attributes */
  LBYTE(USBD_WINUSB_OUT_MAXPACKET_SIZE),        
  HBYTE(USBD_WINUSB_OUT_MAXPACKET_SIZE), /* wMaxPacketSize */
  0x00,                                  /* bInterval */

  USB_DEVICE_EPT_LEN,                    /* bLength: size of endpoint descriptor in bytes */
  USB_DESCIPTOR_TYPE_ENDPOINT,           /* bDescriptorType: endpoint descriptor type */
  USBD_WINUSB_BULK_IN_EPT,               /* bEndpointAddress: IN endpoint */
  USB_EPT_DESC_BULK,                     /* bmAttributes: endpoint attributes */
  LBYTE(USBD_WINUSB_IN_MAXPACKET_SIZE),
  HBYTE(USBD_WINUSB_IN_MAXPACKET_SIZE),  /* wMaxPacketSize */
  0x00,                                  /* bInterval */
};

/**
  * @brief usb string lang id
  */
#if defined ( __ICCARM__ ) /* iar compiler */
  #pragma data_alignment=4
#endif
ALIGNED_HEAD static uint8_t g_string_lang_id[USBD_WINUSB_SIZ_STRING_LANGID] ALIGNED_TAIL =
{
  USBD_WINUSB_SIZ_STRING_LANGID,
  USB_DESCIPTOR_TYPE_STRING,
  0x09,
  0x04,
};

/**
  * @brief usb string serial
  */
#if defined ( __ICCARM__ ) /* iar compiler */
  #pragma data_alignment=4
#endif
ALIGNED_HEAD static uint8_t g_string_serial[USBD_WINUSB_SIZ_STRING_SERIAL] ALIGNED_TAIL =
{
  USBD_WINUSB_SIZ_STRING_SERIAL,
  USB_DESCIPTOR_TYPE_STRING,
};


/* device descriptor */
static usbd_desc_t device_descriptor =
{
  USB_DEVICE_DESC_LEN,
  g_usbd_descriptor
};

/* config descriptor */
static usbd_desc_t config_descriptor =
{
  USBD_WINUSB_CONFIG_DESC_SIZE,
  g_usbd_configuration
};

/* langid descriptor */
static usbd_desc_t langid_descriptor =
{
  USBD_WINUSB_SIZ_STRING_LANGID,
  g_string_lang_id
};

/* serial descriptor */
static usbd_desc_t serial_descriptor =
{
  USBD_WINUSB_SIZ_STRING_SERIAL,
  g_string_serial
};

static usbd_desc_t vp_desc;

/**
  * @brief  standard usb unicode convert
  * @param  string: source string
  * @param  unicode_buf: unicode buffer
  * @retval length
  */
static uint16_t usbd_unicode_convert(uint8_t *string, uint8_t *unicode_buf)
{
  uint16_t str_len = 0, id_pos = 2;
  uint8_t *tmp_str = string;

  while(*tmp_str != '\0')
  {
    str_len ++;
    unicode_buf[id_pos ++] = *tmp_str ++;
    unicode_buf[id_pos ++] = 0x00;
  }

  str_len = str_len * 2 + 2;
  unicode_buf[0] = (uint8_t)str_len;
  unicode_buf[1] = USB_DESCIPTOR_TYPE_STRING;

  return str_len;
}

/**
  * @brief  usb int convert to unicode
  * @param  value: int value
  * @param  pbus: unicode buffer
  * @param  len: length
  * @retval none
  */
static void usbd_int_to_unicode (uint32_t value , uint8_t *pbuf , uint8_t len)
{
  uint8_t idx = 0;

  for( idx = 0 ; idx < len ; idx ++)
  {
    if( ((value >> 28)) < 0xA )
    {
      pbuf[ 2 * idx] = (value >> 28) + '0';
  }
  else
  {
      pbuf[2 * idx] = (value >> 28) + 'A' - 10;
    }

    value = value << 4;

    pbuf[2 * idx + 1] = 0;
  }
}

/**
  * @brief  usb get serial number
  * @param  none
  * @retval none
  */
static void get_serial_num(void)
{
  uint32_t serial0, serial1, serial2;

  serial0 = *(uint32_t*)MCU_ID1;
  serial1 = *(uint32_t*)MCU_ID2;
  serial2 = *(uint32_t*)MCU_ID3;

  if (serial0 != 0 || serial1 != 0 || serial2 != 0)
  {
    usbd_int_to_unicode (serial0, &g_string_serial[2] ,8);
    usbd_int_to_unicode (serial1, &g_string_serial[18] ,8);
    usbd_int_to_unicode (serial2, &g_string_serial[34] ,8);
  }
}

/**
  * @brief  get device descriptor
  * @param  none
  * @retval usbd_desc
  */
static usbd_desc_t *get_device_descriptor(void)
{
  return &device_descriptor;
}

/**
  * @brief  get device qualifier
  * @param  none
  * @retval usbd_desc
  */
static usbd_desc_t * get_device_qualifier(void)
{
  return NULL;
}

/**
  * @brief  get config descriptor
  * @param  none
  * @retval usbd_desc
  */
static usbd_desc_t *get_device_configuration(void)
{
  return &config_descriptor;
}


/**
  * @brief  get other speed descriptor
  * @param  none
  * @retval usbd_desc
  */
static usbd_desc_t *get_device_other_speed(void)
{
  return NULL;
}

/**
  * @brief  get lang id descriptor
  * @param  none
  * @retval usbd_desc
  */
static usbd_desc_t *get_device_lang_id(void)
{
  return &langid_descriptor;
}


/**
  * @brief  get manufacturer descriptor
  * @param  none
  * @retval usbd_desc
  */
static usbd_desc_t *get_device_manufacturer_string(void)
{
  vp_desc.length = usbd_unicode_convert((uint8_t *)USBD_WINUSB_DESC_MANUFACTURER_STRING, g_usbd_desc_buffer);
  vp_desc.descriptor = g_usbd_desc_buffer;
  return &vp_desc;
}

/**
  * @brief  get product descriptor
  * @param  none
  * @retval usbd_desc
  */
static usbd_desc_t *get_device_product_string(void)
{
  vp_desc.length = usbd_unicode_convert((uint8_t *)USBD_WINUSB_DESC_PRODUCT_STRING, g_usbd_desc_buffer);
  vp_desc.descriptor = g_usbd_desc_buffer;
  return &vp_desc;
}

/**
  * @brief  get serial descriptor
  * @param  none
  * @retval usbd_desc
  */
static usbd_desc_t *get_device_serial_string(void)
{
  get_serial_num();
  return &serial_descriptor;
}

/**
  * @brief  get interface descriptor
  * @param  none
  * @retval usbd_desc
  */
static usbd_desc_t *get_device_interface_string(void)
{
  vp_desc.length = usbd_unicode_convert((uint8_t *)USBD_WINUSB_DESC_INTERFACE_STRING, g_usbd_desc_buffer);
  vp_desc.descriptor = g_usbd_desc_buffer;
  return &vp_desc;
}

/**
  * @brief  get device config descriptor
  * @param  none
  * @retval usbd_desc
  */
static usbd_desc_t *get_device_config_string(void)
{
  vp_desc.length = usbd_unicode_convert((uint8_t *)USBD_WINUSB_DESC_CONFIGURATION_STRING, g_usbd_desc_buffer);
  vp_desc.descriptor = g_usbd_desc_buffer;
  return &vp_desc;
}

#if (USBD_SUPPORT_WINUSB == 1)

#define USB_LEN_OS_FEATURE_DESC   0x28
#define USB_LEN_OS_PROPERTY_DESC  0x92

/* OS string descriptor: hardcoded 18 bytes (matching candleLight MS OS 2.0 spec).
 * Using usbd_unicode_convert produces 20 bytes (for "MSFT100" + vendor code + null),
 * which is 2 bytes too long and causes Windows to NOT request the MS OS descriptors.
 * Must be exactly 18 bytes: length(0x12) + type(0x03) + "MSFT100\x20" in Unicode. */
ALIGNED_HEAD static uint8_t g_usbd_winusb_os_string[18] ALIGNED_TAIL = {
    0x12, 0x03,                               /* bLength=18, bDescriptorType=STRING */
    0x4D, 0x00, 0x53, 0x00, 0x46, 0x00, 0x54, 0x00, /* "MSFT" */
    0x31, 0x00, 0x30, 0x00, 0x30, 0x00,             /* "100" */
    WINUSB_BMS_VENDOR_CODE, 0x00                     /* vendor code + null terminator */
};

#define U32TO8C(v) ((v & 0xFF), ((v >> 8) & 0xFF), ((v >> 16) & 0xFF), ((v >> 24) & 0xFF))

#if defined ( __ICCARM__ ) /* iar compiler */
  #pragma data_alignment=4
#endif
ALIGNED_HEAD static uint8_t g_usbd_winusb_os_feature[USB_LEN_OS_FEATURE_DESC] ALIGNED_TAIL =
{
  USB_LEN_OS_FEATURE_DESC, 0x00, 0x00, 0x00,     /* dwlength: 4 byte */
  0x00, 0x01,                                    /* bcd version 1.0 */
  0x04, 0x00,                                    /* windex: extended compat ID descritor */
  0x01,                                          /* bcount: 1 function section */
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,      /* reserved */
  /* function section 1: Interface 0 (gs_usb) */
  0x00,                                          /* first interface number */
  0x01,                                          /* reserved */
  'W', 'I', 'N', 'U', 'S', 'B', 0x00, 0x00,      /* 8 byte: the function compatible id */
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,/* 8 byte: the function subcompatible id */
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,            /* reserved 6 bytes */
};

#if defined ( __ICCARM__ ) /* iar compiler */
  #pragma data_alignment=4
#endif
ALIGNED_HEAD static uint8_t g_usbd_winusb_os_property[USB_LEN_OS_PROPERTY_DESC] ALIGNED_TAIL =
{
	0x92, 0x00, 0x00, 0x00, /* dwlength: 4 byte */
  0x00, 0x01,             /* bcd version 1.0 */
  0x05, 0x00,             /* property descriptor index (0x0005) */
  0x01, 0x00,             /* wcount: number of section (1) */
  
  /* custom property section */
  0x88, 0x00, 0x00, 0x00, /* dwsize: size of property section */
  0x07, 0x00, 0x00, 0x00, /* property data type 7: Unicode REG_MULTI_SZ */
  0x2a, 0x00,             /* property name length */

  /* property name "DeviceInterfaceGUIDs" */
  0x44, 0x00, 0x65, 0x00, /* D e */
  0x76, 0x00, 0x69, 0x00, /* v i */
  0x63, 0x00, 0x65, 0x00, /* c e */
  0x49, 0x00, 0x6e, 0x00, /* I n */
  0x74, 0x00, 0x65, 0x00, /* t e */
  0x72, 0x00, 0x66, 0x00, /* r f */
  0x61, 0x00, 0x63, 0x00, /* a c */
  0x65, 0x00, 0x47, 0x00, /* e G */
  0x55, 0x00, 0x49, 0x00, /* U I */
  0x44, 0x00, 0x73, 0x00, /* D s */
  0x00, 0x00,

  0x50, 0x00, 0x00, 0x00, /* property data length */

  /* property data: "{c15b4308-04d3-11e6-b3ea-6057189e6443}\0\0" */
  0x7b, 0x00, 0x63, 0x00, /* { c */
  0x31, 0x00, 0x35, 0x00, /* 1 5 */
  0x62, 0x00, 0x34, 0x00, /* b 4 */
  0x33, 0x00, 0x30, 0x00, /* 3 0 */
  0x38, 0x00, 0x2d, 0x00, /* 8 - */
  0x30, 0x00, 0x34, 0x00, /* 0 4 */
  0x64, 0x00, 0x33, 0x00, /* d 3 */
  0x2d, 0x00, 0x31, 0x00, /* - 1 */
  0x31, 0x00, 0x65, 0x00, /* 1 e */
  0x36, 0x00, 0x2d, 0x00, /* 6 - */
  0x62, 0x00, 0x33, 0x00, /* b 3 */
  0x65, 0x00, 0x61, 0x00, /* e a */
  0x2d, 0x00, 0x36, 0x00, /* - 6 */
  0x30, 0x00, 0x35, 0x00, /* 0 5 */
  0x37, 0x00, 0x31, 0x00, /* 7 1 */
  0x38, 0x00, 0x39, 0x00, /* 8 9 */
  0x65, 0x00, 0x36, 0x00, /* e 6 */
  0x34, 0x00, 0x34, 0x00, /* 4 4 */
  0x33, 0x00, 0x7d, 0x00, /* 3 } */
  0x00, 0x00, 0x00, 0x00  /* \0\0 (REG_MULTI_SZ double null) */
};

static usbd_desc_t winusb_os_feature_descriptor =
{
  USB_LEN_OS_FEATURE_DESC,
  g_usbd_winusb_os_feature
};

static usbd_desc_t winusb_os_property_descriptor =
{
  USB_LEN_OS_PROPERTY_DESC,
  g_usbd_winusb_os_property
};

static usbd_desc_t winusb_os_string_descriptor =
{
  18,
  g_usbd_winusb_os_string
};

/**
  * @brief  get winusb os string - returns hardcoded 18-byte descriptor
  * @param  none
  * @retval usbd_desc
  */
static usbd_desc_t *get_device_winusb_os_string(void)
{
  return &winusb_os_string_descriptor;
}

/**
  * @brief  get winusb os feature
  * @param  none
  * @retval usbd_desc
  */
static usbd_desc_t *get_device_winusb_os_feature(void)
{
	return &winusb_os_feature_descriptor;
}

/**
  * @brief  get winusb os property
  * @param  none
  * @retval usbd_desc
  */
static usbd_desc_t *get_device_winusb_os_property(void)
{
	return &winusb_os_property_descriptor;
}

/* ============================================================================
 * MS OS 2.0 descriptors (via BOS) — see winusb_class.h for rationale.
 * When bcdUSB=0x0210, Windows MUST query the BOS on every enumeration (USB
 * spec mandates this for USB 2.1+ devices). The BOS carries an MS OS 2.0
 * platform capability that points Windows to a vendor request for the MS OS
 * 2.0 descriptor set, which declares CompatibleID=WINUSB and a
 * DeviceInterfaceGUIDs registry property for EACH interface. This bypasses
 * the MS OS 1.0 string cache that previously prevented dual-interface
 * enumeration.
 * ========================================================================== */

/* BOS Descriptor (33 bytes): header (5) + MS OS 2.0 platform capability (28).
 *   MS OS 2.0 UUID = {D8DD60DF-4589-4CC7-9CD2-659D9E648A9F}
 *   UUID byte order in descriptor (mixed endian per USB spec):
 *     first 4 bytes LE | next 2 LE | next 2 LE | last 8 BE */
#if defined ( __ICCARM__ ) /* iar compiler */
  #pragma data_alignment=4
#endif
ALIGNED_HEAD static uint8_t g_usbd_bos_descriptor[33] ALIGNED_TAIL =
{
  /* --- BOS Header (5 bytes) --- */
  0x05,                                  /* bLength */
  USB_DESCIPTOR_TYPE_BOS,                /* bDescriptorType = 0x0F */
  0x21, 0x00,                            /* wTotalLength = 33 */
  0x01,                                  /* bNumDeviceCaps = 1 */

  /* --- MS OS 2.0 Platform Capability Descriptor (28 bytes) --- */
  0x1C,                                  /* bLength = 28 */
  USB_DESCIPTOR_TYPE_DEVICE_CAP,         /* bDescriptorType = 0x10 (Device Capability) */
  0x05,                                  /* bDevCapabilityType = 0x05 (Platform) */
  0x00,                                  /* bReserved = 0 */
  /* PlatformCapabilityUUID (16 bytes) — MS OS 2.0 UUID */
  0xDF, 0x60, 0xDD, 0xD8,                /* 1st 4 bytes LE  (D8DD60DF -> DF60DDD8) */
  0x89, 0x45,                            /* next 2 bytes LE (4589 -> 8945) */
  0xC7, 0x4C,                            /* next 2 bytes LE (4CC7 -> C74C) */
  0x9C, 0xD2, 0x65, 0x9D, 0x9E, 0x64, 0x8A, 0x9F, /* last 8 BE */
  /* dwWindowsVersion (4 bytes) = 0x06030000 (Windows 8.1+, LE) */
  0x00, 0x00, 0x03, 0x06,
  /* wDescriptorSetTotalLength (2 bytes) = 338 (0x0152) */
  LBYTE(WINUSB20_DESC_SET_SIZE), HBYTE(WINUSB20_DESC_SET_SIZE),
  /* bVendorCode (1 byte) */
  WINUSB_BMS_VENDOR_CODE,
  /* bAltEnumCode (1 byte) = 0 */
  0x00
};

static usbd_desc_t bos_descriptor =
{
  33,
  g_usbd_bos_descriptor
};

/* ----- Helper macro: encode one ASCII char as UTF-16LE -----
 * Used to keep the registry property name/data arrays readable. */
#define U16(c)  (uint8_t)(c), 0x00

/* MS OS 2.0 Descriptor Set (178 bytes = 0x00B2).
 *
 * Layout:
 *   [Set Header             10]  type 0x0000
 *   [Config Subset Header    8]  type 0x0001
 *   [Func0 Subset Header     8]  type 0x0002  bFirstInterface=0
 *     [Compatible ID        20]  type 0x0003  ("WINUSB")
 *     [Registry Property   132]  type 0x0004  DeviceInterfaceGUIDs (IF0)
 *
 * Registry Property descriptor layout (MS OS 2.0 spec):
 *   wLength(2) + wDescriptorType(2) + wPropertyDataType(2) + wPropertyNameLength(2)
 *   + PropertyName(42 = "DeviceInterfaceGUIDs\0" in UTF-16LE)
 *   + wPropertyDataLength(2) + PropertyData(80 = "{guid}\0\0" in UTF-16LE)
 *   = 132 bytes total
 * Note: wPropertyDataLength is 2 bytes (not 4) in MS OS 2.0, unlike MS OS 1.0.
 */
#if defined ( __ICCARM__ ) /* iar compiler */
  #pragma data_alignment=4
#endif
ALIGNED_HEAD static uint8_t g_usbd_msos20_desc_set[WINUSB20_DESC_SET_SIZE] ALIGNED_TAIL =
{
  /* === Set Header (10 bytes) === */
  0x0A, 0x00,                            /* wLength = 10 */
  0x00, 0x00,                            /* wDescriptorType = 0x0000 (Set Header) */
  0x00, 0x00, 0x03, 0x06,                /* dwWindowsVersion = 0x06030000 */
  LBYTE(WINUSB20_DESC_SET_SIZE), HBYTE(WINUSB20_DESC_SET_SIZE),  /* wTotalLength = 338 */

  /* === Configuration Subset Header (8 bytes) === */
  0x08, 0x00,                            /* wLength = 8 */
  0x01, 0x00,                            /* wDescriptorType = 0x0001 (Configuration Subset) */
  0x00,                                  /* bConfigurationValue = 0 */
  0x00,                                  /* bReserved = 0 */
  LBYTE(WINUSB20_CFG_SUBSET_SIZE), HBYTE(WINUSB20_CFG_SUBSET_SIZE), /* wTotalLength = 328 */

  /* === Function Subset 0 (160 bytes) — Interface 0 (gs_usb / CANgaroo) === */
  0x08, 0x00,                            /* wLength = 8 */
  0x02, 0x00,                            /* wDescriptorType = 0x0002 (Function Subset) */
  0x00,                                  /* bFirstInterface = 0 */
  0x00,                                  /* bReserved = 0 */
  LBYTE(WINUSB20_FUNC_SUBSET_SIZE), HBYTE(WINUSB20_FUNC_SUBSET_SIZE), /* wSubsetLength = 160 */

  /* -- Compatible ID Descriptor (20 bytes) -- MS OS 2.0 spec: wLength=0x14
   *   wLength(2) + wDescriptorType(2) + CompatibleID(8) + SubCompatibleID(8) = 20 */
  0x14, 0x00,                            /* wLength = 20 */
  0x03, 0x00,                            /* wDescriptorType = 0x0003 (Compatible ID) */
  'W','I','N','U','S','B',0x00,0x00,     /* CompatibleID (8 bytes) */
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, /* SubCompatibleID (8 bytes) */

  /* -- Registry Property Descriptor (132 bytes) -- DeviceInterfaceGUIDs (IF0) */
  0x84, 0x00,                            /* wLength = 132 */
  0x04, 0x00,                            /* wDescriptorType = 0x0004 (Registry Property) */
  0x07, 0x00,                            /* wPropertyDataType = 7 (REG_MULTI_SZ) */
  0x2A, 0x00,                            /* wPropertyNameLength = 42 bytes */
  /* Property name: "DeviceInterfaceGUIDs\0" UTF-16LE (21 chars × 2 = 42 bytes) */
  U16('D'), U16('e'), U16('v'), U16('i'), U16('c'), U16('e'),   /* "Device" */
  U16('I'), U16('n'), U16('t'), U16('e'), U16('r'), U16('f'),   /* "Interf" */
  U16('a'), U16('c'), U16('e'), U16('G'), U16('U'), U16('I'),   /* "aceGUI" */
  U16('D'), U16('s'), 0x00, 0x00,                                /* "Ds\0" */
  0x50, 0x00,                            /* wPropertyDataLength = 80 bytes (2 bytes in MS OS 2.0) */
  /* Property data: "{c15b4308-04d3-11e6-b3ea-6057189e6443}\0\0" UTF-16LE (80 bytes) */
  U16('{'), U16('c'), U16('1'), U16('5'), U16('b'), U16('4'),   /* "{c15b4" */
  U16('3'), U16('0'), U16('8'), U16('-'), U16('0'), U16('4'),   /* "308-04" */
  U16('d'), U16('3'), U16('-'), U16('1'), U16('1'), U16('e'),   /* "d3-11e" */
  U16('6'), U16('-'), U16('b'), U16('3'), U16('e'), U16('a'),   /* "6-b3ea" */
  U16('-'), U16('6'), U16('0'), U16('5'), U16('7'), U16('1'),   /* "-60571" */
  U16('8'), U16('9'), U16('e'), U16('6'), U16('4'), U16('4'),   /* "89e644" */
  U16('3'), U16('}'),                                            /* "3}" */
  0x00, 0x00,                            /* string null terminator */
  0x00, 0x00,                            /* REG_MULTI_SZ final null terminator */
};

static usbd_desc_t msos20_descriptor_set =
{
  WINUSB20_DESC_SET_SIZE,
  g_usbd_msos20_desc_set
};

/**
  * @brief  get BOS descriptor (for USB 2.1 enumeration)
  *         Called by class_setup_handler when Windows queries descriptor type 0x0F.
  */
usbd_desc_t *get_device_winusb_bos(void)
{
  return &bos_descriptor;
}

/**
  * @brief  get MS OS 2.0 descriptor set
  *         Called by class_setup_handler when Windows sends vendor request
  *         bRequest=WINUSB_BMS_VENDOR_CODE, wIndex=MSOS20_DESC_SET_INDEX.
  */
usbd_desc_t *get_device_winusb_msos20_set(void)
{
  return &msos20_descriptor_set;
}

#endif

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
