/*
 * G2FD - three-channel CANFD gs_usb adapter firmware (AT32F456)
 * Copyright (c) 2026 G2FD contributors
 * SPDX-License-Identifier: MIT
 */

/**
  **************************************************************************
  * @file     bsp_can_init.h
  * @brief    CAN hardware initialization declarations
  **************************************************************************
  */

#ifndef __BSP_CAN_INIT_H__
#define __BSP_CAN_INIT_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "at32f45x.h"

/**
  * @brief  Initialize CAN1 hardware
  * @retval error_status
  */
error_status can_configuration(void);

/**
 * @brief  Initialize CAN2 hardware
 * @retval error_status
 */
error_status can2_configuration(void);

/**
 * @brief  Initialize CAN3 hardware
 * @retval error_status
 */
error_status can3_configuration(void);

/**
 * @brief  Whether a channel is powered down (clock-gated + transceiver in standby).
 *         Accessing CANx registers while its clock is gated stalls the bus; query this first.
 * @param  channel: gs_usb channel number (0=CAN3, 1=CAN2, 2=CAN1)
 * @retval 1: disabled; 0: enabled
 */
uint8_t can_ch_is_disabled(uint32_t channel);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_CAN_INIT_H__ */
