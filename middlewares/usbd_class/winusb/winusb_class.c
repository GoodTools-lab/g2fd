/**
  **************************************************************************
  * @file     winusb_class.c
  * @brief    usb winusb class type
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
#include "usbd_core.h"
#include "usb_core.h"
#include "winusb_class.h"
#include "winusb_desc.h"
#include "gs_usb_protocol.h"
#include "bsp_can.h"
#include "at32f45x_board.h"
#include "string.h"
#include "stdio.h"
#include "stdlib.h"

/* External gs_usb bulk endpoint callbacks */
extern void gs_usb_bulk_tx_complete(void);
extern uint8_t gs_usb_bulk_rx_notify(const uint8_t *data, uint16_t len);

/** @addtogroup AT32F45x_middlewares_usbd_class
  * @{
  */

/** @defgroup USB_winusb_class
  * @brief usb device class win usb
  * @{
  */

/** @defgroup USB_winusb_class_private_functions
  * @{
  */

static usb_sts_type class_init_handler(void *udev);
static usb_sts_type class_clear_handler(void *udev);
static usb_sts_type class_setup_handler(void *udev, usb_setup_type *setup);
static usb_sts_type class_ept0_tx_handler(void *udev);
static usb_sts_type class_ept0_rx_handler(void *udev);
static usb_sts_type class_in_handler(void *udev, uint8_t ept_num);
static usb_sts_type class_out_handler(void *udev, uint8_t ept_num);
static usb_sts_type class_sof_handler(void *udev);
static usb_sts_type class_event_handler(void *udev, usbd_event_type event);
#if (USBD_SUPPORT_WINUSB == 1)
static usb_sts_type usbd_get_winusb_descriptor(usbd_core_type *udev);
#endif
static usb_sts_type winusb_struct_init(winusb_struct_type *p_winusb);

/* winusb data struct */
winusb_struct_type winusb_struct;

/* Standalone Bulk IN TX completion flag - isolated from winusb_struct_type
 * to prevent struct corruption from affecting Bulk IN state */
volatile uint8_t g_bulk_in_tx_completed = 1;
/* Bulk IN watchdog: records when send_data starts; TX complete clears it.
 * If the completion event is swallowed by a race between the main loop and
 * the OTG interrupt (both operate on the endpoint), the flag sticks at 0 and
 * the data path deadlocks entirely (observed in the 20:22:10 trace snapshot).
 * A 500ms timeout triggers classified recovery. */
static volatile uint32_t g_bulk_in_tx_start_tick = 0;

/* ====== Bulk OUT endpoint NAK flow control ======
 * When TX ring is full, the OUT endpoint is paused (not re-armed).
 * USB hardware automatically NAKs host OUT requests.
 * Host controller retries in next USB frame (1ms FS) — zero frame loss.
 * usb_winusb_resume_out() re-arms endpoint when ring has space. */
volatile uint8_t g_usb_out_paused = 0;

/* ZLP pending flag: when transfer size is an exact multiple of maxpacket (64B),
 * USB FS bulk transfer has no short packet to terminate. A zero-length packet
 * must be sent after TX complete to signal transfer end to the host. */
static volatile uint8_t g_zlp_pending = 0;

/* ====== USB link alive detection (SOF based) ======
 * USB_VBUS_IGNORE disables VBUS session-end detection, so cable unplug never
 * triggers usbd_discon_handler/class_clear_handler. Track the last SOF tick
 * instead (host emits SOF every 1ms) and let the main loop detect silence. */
#define USB_SOF_TIMEOUT_MS   1000u
static volatile uint32_t g_usb_sof_tick = 0;

/* winusb receive buffer define - sized for CAN FD frames (80 bytes) */
static uint32_t g_winusb_rx_buffer[20]; /* 20 * 4 = 80 bytes */

/* usb device class handler */
usbd_class_handler winusb_class_handler =
{
  class_init_handler,
  class_clear_handler,
  class_setup_handler,
  class_ept0_tx_handler,
  class_ept0_rx_handler,
  class_in_handler,
  class_out_handler,
  class_sof_handler,
  class_event_handler,
  &winusb_struct
};
/**
  * @brief  initialize usb endpoint
  * @param  udev: to the structure of usbd_core_type
  * @retval status of usb_sts_type
  */
static usb_sts_type class_init_handler(void *udev)
{
  usb_sts_type status = USB_OK;
  usbd_core_type *pudev = (usbd_core_type *)udev;
  winusb_struct_type *p_winusb = (winusb_struct_type *)pudev->class_handler->pdata;

  /* init winusb struct */
  winusb_struct_init(p_winusb);

  p_winusb->maxpacket = USBD_FS_WINUSB_MAXPACKET_SIZE;

  /* open out endpoint */
  usbd_ept_open(pudev, USBD_WINUSB_BULK_OUT_EPT, EPT_BULK_TYPE, USBD_WINUSB_OUT_MAXPACKET_SIZE);

  /* open in endpoint */
  usbd_ept_open(pudev, USBD_WINUSB_BULK_IN_EPT, EPT_BULK_TYPE, USBD_WINUSB_IN_MAXPACKET_SIZE);

  /* set out endpoint to receive status */
  usbd_ept_recv(pudev, USBD_WINUSB_BULK_OUT_EPT, p_winusb->g_rx_buff, USBD_WINUSB_OUT_MAXPACKET_SIZE);

  return status;
}

/**
  * @brief  clear endpoint or other state
  * @param  udev: to the structure of usbd_core_type
  * @retval status of usb_sts_type
  */
static usb_sts_type class_clear_handler(void *udev)
{
  usb_sts_type status = USB_OK;
  usbd_core_type *pudev = (usbd_core_type *)udev;

  /* close in endpoint */
  usbd_ept_close(pudev, USBD_WINUSB_BULK_IN_EPT);

  /* close out endpoint */
  usbd_ept_close(pudev, USBD_WINUSB_BULK_OUT_EPT);

  return status;
}

/**
  * @brief  usb device class setup request handler
  * @param  udev: to the structure of usbd_core_type
  * @param  setup: setup packet
  * @retval status of usb_sts_type
  */
static usb_sts_type class_setup_handler(void *udev, usb_setup_type *setup)
{
  usb_sts_type status = USB_OK;
  usbd_core_type *pudev = (usbd_core_type *)udev;
  winusb_struct_type *p_winusb = (winusb_struct_type *)pudev->class_handler->pdata;

  switch(setup->bmRequestType & USB_REQ_TYPE_RESERVED)
  {
#if (USBD_SUPPORT_WINUSB == 1)
    /* CLASS requests: STALL (no DFU interface in runtime firmware).
     * This matches candleLight's behavior where CLASS requests are only
     * handled for DFU interface; all other CLASS requests go to
     * Config_Request which STALLs unknown requests. */
    case USB_REQ_TYPE_CLASS:
      usbd_ctrl_unsupport(pudev);
      return USB_FAIL;

    /* VENDOR requests: gs_usb protocol or MS OS descriptor */
    case USB_REQ_TYPE_VENDOR:
      if(setup->bRequest <= GS_USB_BREQ_MAX)
      {
        status = (usb_sts_type)gs_usb_handle_setup(udev, setup);
      }
      else if(setup->bRequest == WINUSB_BMS_VENDOR_CODE)
      {
        status = usbd_get_winusb_descriptor(pudev);
      }
      else
      {
        usbd_ctrl_unsupport(pudev);
        return USB_FAIL;
      }
      break;
#else
    case USB_REQ_TYPE_CLASS:
    case USB_REQ_TYPE_VENDOR:
      usbd_ctrl_unsupport(pudev);
      return USB_FAIL;
#endif
    /* standard request - matches candleLight USBD_GS_CAN_Setup:
     * only handle GET_INTERFACE/SET_INTERFACE, let the stack handle
     * everything else (returns USB_OK instead of STALLing).
     * Exception: GET_DESCRIPTOR(type=BOS) is routed here from usbd_sdr.c
     * so the winusb class can return the MS OS 2.0 platform capability. */
    case USB_REQ_TYPE_STANDARD:
      /* Handle GET_DESCRIPTOR for BOS — routed from usbd_get_descriptor
       * because it is not a standard type/index handled by the BSP's switch. */
      if(setup->bRequest == USB_STD_REQ_GET_DESCRIPTOR)
      {
        uint8_t desc_type = (uint8_t)(setup->wValue >> 8);
        if(desc_type == USB_DESCIPTOR_TYPE_BOS)
        {
          usbd_desc_t *bos = get_device_winusb_bos();
          if(bos != NULL && bos->length != 0 && setup->wLength != 0)
          {
            uint16_t len = MIN(bos->length, setup->wLength);
            usbd_ctrl_send(pudev, bos->descriptor, len);
          }
          else
          {
            usbd_ctrl_unsupport(pudev);
          }
          return USB_OK;
        }
      }
      switch(setup->bRequest)
      {
        case USB_STD_REQ_GET_INTERFACE:
          usbd_ctrl_send(pudev, (uint8_t *)&p_winusb->alt_setting, 1);
          break;
        case USB_STD_REQ_SET_INTERFACE:
          p_winusb->alt_setting = setup->wValue;
          break;
        default:
          break;
      }
      break;
    default:
      usbd_ctrl_unsupport(pudev);
      break;
  }
  return status;
}

#if (USBD_SUPPORT_WINUSB == 1)
static usb_sts_type usbd_get_winusb_descriptor(usbd_core_type *udev)
{
	usb_sts_type ret = USB_OK;
  uint16_t len = 0;
  usbd_desc_t *desc = NULL;
  uint8_t desc_type = udev->setup.wIndex;

	switch(desc_type)
  {
			case USB_WINUSB_COMPAT_ID:
				desc = udev->desc_handler->get_device_winusb_os_feature();
			  break;
			case USB_WINUSB_PROPERTIES_ID:
				/* MS OS 1.0 spec: wValue high byte = interface number.
				 * Some implementations use wIndex high byte instead — check both. */
				{
					uint8_t iface_wval = (uint8_t)(udev->setup.wValue >> 8);
					uint8_t iface_widx = (uint8_t)(udev->setup.wIndex >> 8);
					uint8_t iface = (iface_wval != 0) ? iface_wval : iface_widx;
					if(iface == 0)
					{
						desc = udev->desc_handler->get_device_winusb_os_property();
					}
					else
					{
						usbd_ctrl_unsupport(udev);
						return USB_FAIL;
					}
				}
				break;
		case MSOS20_DESC_SET_INDEX:
			/* MS OS 2.0 Descriptor Set request (wIndex=0x0007).
			 * Sent by Windows 10+ after reading the BOS descriptor's MS OS 2.0
			 * platform capability. Returns the descriptor set declaring
			 * CompatibleID=WINUSB and DeviceInterfaceGUIDs for interface 0. */
			desc = get_device_winusb_msos20_set();
			break;
			default:
      usbd_ctrl_unsupport(udev);
      return USB_FAIL;
	}
	if(desc != NULL)
  {
    if((desc->length != 0) && (udev->setup.wLength != 0))
    {
      len = MIN(desc->length , udev->setup.wLength);
      usbd_ctrl_send(udev, desc->descriptor, len);
    }
  }
  else
  {
    /* matches candleLight: no descriptor found → STALL */
    usbd_ctrl_unsupport(udev);
    return USB_FAIL;
  }
		
	return ret;
}
#endif

/**
  * @brief  usb device endpoint 0 in status stage complete
  * @param  udev: to the structure of usbd_core_type
  * @retval status of usb_sts_type
  */
static usb_sts_type class_ept0_tx_handler(void *udev)
{
  usb_sts_type status = USB_OK;

  /* ...user code... */

  return status;
}

/**
  * @brief  usb device endpoint 0 out status stage complete
  * @param  udev: usb device core handler type
  * @retval status of usb_sts_type
  */
static usb_sts_type class_ept0_rx_handler(void *udev)
{
  usb_sts_type status = USB_OK;
  
  /* Only handle gs_usb EP0 RX if this was a gs_usb request */
  if(gs_usb_is_pending_request()) {
    /* Handle gs_usb EP0 RX complete and propagate the result.
     * This matches candleLight's USBD_GS_CAN_EP0_RxReady which
     * returns USBD_FAIL on error. */
    status = (usb_sts_type)gs_usb_handle_ep0_rx(udev);
  }
  
  return status;
}

/**
  * @brief  usb device transmision complete handler
  * @param  udev: to the structure of usbd_core_type
  * @param  ept_num: endpoint number
  * @retval status of usb_sts_type
  */
static usb_sts_type class_in_handler(void *udev, uint8_t ept_num)
{
  usbd_core_type *pudev = (usbd_core_type *)udev;
  winusb_struct_type *p_winusb = (winusb_struct_type *)pudev->class_handler->pdata;
  usb_sts_type status = USB_OK;

  /* Host fetched data (IN complete) = proof of link liveness; refresh the alive timestamp */
  usb_link_traffic_seen();

  /* Notify gs_usb that TX completed */
  gs_usb_bulk_tx_complete();

  if (g_zlp_pending) {
    /* Previous transfer was exact multiple of maxpacket (64B): send ZLP
     * to terminate the bulk IN transfer. g_bulk_in_tx_completed stays 0
     * until the ZLP's own TX complete fires, preventing the main loop
     * from starting a new transfer before the ZLP is sent. */
    g_zlp_pending = 0;
    usbd_ept_send(pudev, USBD_WINUSB_BULK_IN_EPT,
                  (uint8_t *)p_winusb->g_rx_buff, 0);
  } else {
    /* Normal TX complete (or ZLP complete): mark USB idle.
     * TX FIFO is already empty after Transfer Complete; no flush needed. */
    g_bulk_in_tx_start_tick = 0;   /* stop the watchdog timer */
    g_bulk_in_tx_completed = 1;
  }

  return status;
}

/**
  * @brief  usb device endpoint receive data
  * @param  udev: to the structure of usbd_core_type
  * @param  ept_num: endpoint number
  * @retval status of usb_sts_type
  */
static usb_sts_type class_out_handler(void *udev, uint8_t ept_num)
{
  usb_sts_type status = USB_OK;
  usbd_core_type *pudev = (usbd_core_type *)udev;
  winusb_struct_type *p_winusb = (winusb_struct_type *)pudev->class_handler->pdata;

  /* Host sent data arrived (OUT complete) = proof of link liveness; refresh the alive timestamp */
  usb_link_traffic_seen();

  /* Fix (root cause: XferCompl is handled before the RXFLVL FIFO drain):
   * In usbd_irq_handler the OUT endpoint XferCompl interrupt runs before the
   * RXFLVL handler that moves the packet from the FIFO into g_rx_buff. If we
   * notified immediately here, usbd_get_recv_len() would read a stale
   * trans_len snapshot (the just-received packet is still sitting in the
   * FIFO), so the content/length pair is mismatched -> occasional frame
   * tearing in the backpressure recovery window (one-way ID swap with the
   * total count intact). The ring-full pause decision also lagged by one
   * packet: packets already ACKed into the FIFO were silently orphaned when
   * resume reset trans_len.
   * New design: the ISR only records the completion event - no notify, no
   * re-arm (the endpoint stays NAK; g_rx_buff is owned exclusively by the
   * main loop). All data consumption is unified in usb_winusb_poll_bulk_out()
   * (main loop), where the RXFLVL drain is guaranteed complete, so the
   * [content, length] snapshot is consistent by construction. */
  p_winusb->g_rx_completed = 1;

  return status;
}

/**
  * @brief  Consume the OUT completion event in the main loop (called at the
  *         top of gs_usb_process_bulk_out).
  *         The OTG ISR has already exited at this point: the RXFLVL drain for
  *         the packet that raised XferCompl is guaranteed complete, so the
  *         trans_len returned by usbd_get_recv_len() strictly matches the
  *         g_rx_buff content. While consuming, the endpoint is not re-armed
  *         (NAK) and the host retries automatically, so no new data is
  *         written to g_rx_buff - eliminating both the content/length
  *         snapshot mismatch of the in-ISR inline notify and the frame loss
  *         from resume orphaning.
  * @param  udev: pointer to usbd_core_type
  * @retval 1 = this packet filled the TX ring (OUT endpoint paused, NAK
  *         backpressure), 0 = normal
  */
uint8_t usb_winusb_poll_bulk_out(void *udev)
{
  usbd_core_type *pudev = (usbd_core_type *)udev;
  winusb_struct_type *p_winusb =
      (winusb_struct_type *)pudev->class_handler->pdata;

  if (!p_winusb->g_rx_completed) {
    return 0;
  }
  p_winusb->g_rx_completed = 0;

  /* Main loop reads the length: the RXFLVL drain is complete, snapshot is consistent */
  uint16_t len = (uint16_t)usbd_get_recv_len(pudev, USBD_WINUSB_BULK_OUT_EPT);
  p_winusb->g_rxlen = len;

  if (len == 0) {
    /* Defensive: XferCompl with no data (theoretically unreachable). Re-arm to resume reception */
    usbd_ept_recv(pudev, USBD_WINUSB_BULK_OUT_EPT, p_winusb->g_rx_buff,
                  p_winusb->maxpacket);
    return 0;
  }

  /* Notify with consistent content/length: reassemble and push to the TX ring */
  uint8_t ring_full = gs_usb_bulk_rx_notify(p_winusb->g_rx_buff, len);

  if (ring_full) {
    /* TX ring full: do not re-arm; the USB hardware NAKs automatically for
     * backpressure. Recovery: usb_winusb_resume_out() at the end of
     * gs_usb_process_bulk_out. */
    g_usb_out_paused = 1;
  } else {
    /* Ring has space: re-arm to receive the next packet */
    usbd_ept_recv(pudev, USBD_WINUSB_BULK_OUT_EPT, p_winusb->g_rx_buff,
                  p_winusb->maxpacket);
  }
  return ring_full;
}

/**
  * @brief  resume bulk out endpoint after NAK pause
  *         Called from gs_usb_process_bulk_out when TX ring has space again.
  *         Re-arms the OUT endpoint so USB hardware stops NAKing and accepts
  *         the host's retried OUT packet.
  * @param  udev: usb device handle
  * @retval none
  */
void usb_winusb_resume_out(void *udev)
{
  if (g_usb_out_paused && udev) {
    usbd_core_type *pudev = (usbd_core_type *)udev;
    winusb_struct_type *p_winusb = (winusb_struct_type *)pudev->class_handler->pdata;
    usbd_ept_recv(pudev, USBD_WINUSB_BULK_OUT_EPT, p_winusb->g_rx_buff, p_winusb->maxpacket);
    g_usb_out_paused = 0;
  }
}

/**
  * @brief  usb device sof handler
  * @param  udev: to the structure of usbd_core_type
  * @retval status of usb_sts_type
  */
static usb_sts_type class_sof_handler(void *udev)
{
  usb_sts_type status = USB_OK;

  /* Link alive heartbeat: full-speed host emits SOF every 1ms */
  g_usb_sof_tick = HAL_GetTick();

  /* Update SOF timestamp for gs_usb CMD 6 (TIMESTAMP) */
  gs_usb_update_sof_timestamp(HAL_GetTick() * 1000);

  return status;
}

/**
  * @brief  check usb link alive (SOF received recently)
  * @param  none
  * @retval 1 = host SOF seen within USB_SOF_TIMEOUT_MS, 0 = silent (detached/suspended)
  */
uint8_t usb_link_alive(void)
{
  return ((HAL_GetTick() - g_usb_sof_tick) < USB_SOF_TIMEOUT_MS) ? 1u : 0u;
}

/**
  * @brief  refresh link-alive timestamp from real USB traffic (ISR context)
  * @note   SOF is not the only evidence that the host is alive: bulk IN/OUT
  *         completion events equally prove the host is transferring data.
  *         Under heavy traffic the SOF interrupt can be preempted and starved
  *         while data transfer never stops - judging liveness by SOF alone
  *         mistakes "busiest" for "detached" (measured: 47 false detach
  *         events in 2180s, which even triggered lockups). Every class
  *         endpoint completion event refreshes the alive timestamp,
  *         complementing SOF: idle links rely on SOF, busy links rely on the
  *         data flow. A real detach = both silent at the same time.
  */
void usb_link_traffic_seen(void)
{
  g_usb_sof_tick = HAL_GetTick();
}

/**
  * @brief  usb device event handler
  * @param  udev: to the structure of usbd_core_type
  * @param  event: usb device event
  * @retval status of usb_sts_type
  */
static usb_sts_type class_event_handler(void *udev, usbd_event_type event)
{
  usb_sts_type status = USB_OK;
  (void)udev;
  (void)event;
  return status;
}

/**
  * @brief  usb device winusb init
  * @param  p_winusb: to the structure of winusb_struct
  * @retval status of usb_sts_type
  */
static usb_sts_type winusb_struct_init(winusb_struct_type *p_winusb)
{
  g_bulk_in_tx_completed = 1;
  g_zlp_pending = 0;
  p_winusb->g_tx_completed = 1;
  p_winusb->g_rx_completed = 0;
  p_winusb->alt_setting = 0;
  p_winusb->g_rx_buff = (uint8_t *)g_winusb_rx_buffer;
  return USB_OK;
}

/**
  * @brief  usb device class send data
  * @param  udev: to the structure of usbd_core_type
  * @param  send_data: send data buffer
  * @param  len: send length
  * @retval error status
  */
error_status usb_winusb_send_data(void *udev, uint8_t *send_data, uint16_t len)
{
  error_status status = SUCCESS;
  usbd_core_type *pudev = (usbd_core_type *)udev;
  if(g_bulk_in_tx_completed)
  {
    uint32_t primask = __get_PRIMASK();
    /* Critical section over the whole block: the historical root cause was
     * completion events lost to a race between the main loop and the OTG ISR.
     * The interrupts-off window is only a few microseconds (usbd_ept_send
     * register writes + FIFO push). The higher-priority CAN interrupt is
     * briefly masked too, but the CAN hardware mailbox buffers tolerate it. */
    __disable_irq();
    g_bulk_in_tx_completed = 0;
    g_bulk_in_tx_start_tick = HAL_GetTick();   /* start the watchdog timer */
    /* The ZLP flag must be set before arming the transfer: if the send is
     * issued first and the flag set afterwards, the transfer can complete at
     * microsecond scale and the ISR runs first (normal-complete branch); the
     * main loop then writes zlp=1 as a stale flag, and every subsequent
     * completion of any length is misread as needing a ZLP, permanently
     * desynchronizing the state machine */
    if (len > 0 && (len % USBD_WINUSB_IN_MAXPACKET_SIZE) == 0) {
      g_zlp_pending = 1;
    }
    usbd_ept_send(pudev, USBD_WINUSB_BULK_IN_EPT, send_data, len);
    __set_PRIMASK(primask);
  }
  else
  {
    status = ERROR;
  }
  return status;
}

/**
  * @brief  Bulk IN completion-event watchdog (called every main loop pass,
  *         with hardware state classification).
  *         When pending for over 500ms while SOF is alive, read the DIEP
  *         registers to distinguish three real states:
  *   [ARMED]  dieptsiz!=0: the transfer is still waiting at hardware level for
  *            the host IN token - the host has not fetched it (upper-layer
  *            application stopped reading / driver buffer full), which is
  *            normal NAK flow control. Never re-send (rewriting dieptsiz on
  *            an already-armed endpoint risks data misalignment); keep the
  *            transfer and it completes naturally once the host resumes
  *            reading, with zero frame loss.
  *   [XFERC]  diepint.xferc=1: the hardware completion flag is set but the
  *            ISR has not processed it - an interrupt race. Clearing the
  *            flag and marking idle recovers; the data was already fully
  *            delivered to the FIFO.
  *   [IDLE]   the endpoint is idle but the software flag is stuck at 0 - the
  *            completion event was truly lost; flush + flag recovery.
  */
void winusb_bulk_in_watchdog(void *udev)
{
  usbd_core_type *pudev = (usbd_core_type *)udev;
  usb_reg_type *usbx = pudev->usb_reg;
  usb_ept_info *ept_info = &pudev->ept_in[USBD_WINUSB_BULK_IN_EPT & 0x7F];
  otg_eptin_type *ept_in = USB_INEPT(usbx, ept_info->eptn);
  uint32_t tsiz, dint;

  if (g_bulk_in_tx_completed || g_bulk_in_tx_start_tick == 0)
    return;
  if ((HAL_GetTick() - g_bulk_in_tx_start_tick) <= 500)
    return;
  if (!usb_link_alive())
    return;   /* Host is genuinely offline/suspended, not a deadlock */

  /* conn_state self-heal: completion event dispatch is gated on
   * conn_state == CONFIGURED (usbd_core_in/out_handler); a resume/wakeup
   * rollback bug or another path can leave it stuck at non-CONFIGURED -
   * SOF alive + bulk transfer pending for >500ms is the deadlock signature.
   * The bus is active and the endpoint is open at this point, so restoring
   * CONFIGURED revives event dispatch immediately. */
  if (pudev->conn_state != USB_CONN_STATE_CONFIGURED)
  {
    pudev->conn_state = USB_CONN_STATE_CONFIGURED;
  }

  tsiz = ept_in->dieptsiz;
  dint = ept_in->diepint;

  if (tsiz != 0)
  {
    /* ARMED: the transfer is still valid at hardware level (data or pending
     * ZLP: either of xfersize|pktcnt is nonzero), waiting for the host IN
     * token - the host has not fetched it yet */
    return;   /* Keep armed: no re-send, no flag set */
  }

  if ((dint & 0x1u) != 0)
  {
    /* XFERC: hardware completion flag pending, unprocessed by the ISR - interrupt race */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    ept_in->diepint = 0x1u;   /* write-1-to-clear xferc */
    g_bulk_in_tx_start_tick = 0;
    g_bulk_in_tx_completed = 1;
    __set_PRIMASK(primask);
    return;
  }

  /* IDLE: endpoint idle and no completion event - the event was truly lost */
  {
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    /* Recheck inside the critical section: the ISR may have completed
     * processing while we were sampling (false-positive guard) */
    if (g_bulk_in_tx_completed)
    {
      __set_PRIMASK(primask);
      return;   /* false positive: the ISR completed processing within the sampling window */
    }
    /* Note: do not flush the TX FIFO - when tsiz==0 the FIFO must already be
     * empty, and flushing only adds a GRSTCTL spin risk */
    g_bulk_in_tx_start_tick = 0;
    g_zlp_pending = 0;
    g_bulk_in_tx_completed = 1;
    __set_PRIMASK(primask);
  }
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

