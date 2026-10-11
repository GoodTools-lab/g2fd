/*
 * G2FD - three-channel CANFD gs_usb adapter firmware (AT32F456)
 * Copyright (c) 2026 G2FD contributors
 * SPDX-License-Identifier: MIT
 */

#pragma once
#include <stdint.h>

/* ====== Indicator LED assignment (six single-color green LEDs) ======
 *  LED_CH0/CH1/CH2 : PA10/PA8/PC6  CAN1/2/3 data LEDs - fast blink on TX/RX (300ms), fast blink on bus-off error
 *  LED_LIN         : PB12          LIN     data LED  - fast blink on TX/RX (reserved)
 *  LED_STAT        : PA15          mode LED          - steady on = gs_usb
 *  LED_REC         : PB2           status LED        - double blink on low power > off (priority arbitration)
 */
enum e_pcan_led
{
  LED_CH0,
  LED_CH1,
  LED_CH2,
  LED_LIN,
  LED_STAT,
  LED_REC,
  LED_TOTAL,
};

enum e_led_mode
{
  LED_MODE_DEVICE = 0, /* we can control led */
  LED_MODE_BLINK_FAST,     /* 10 Hz symmetric blink: TX/RX activity / error indication (optional timeout to auto-off) */
  LED_MODE_BLINK_SLOW,     /* 2.5 Hz symmetric blink */
  LED_MODE_ON,
  LED_MODE_OFF,
  LED_MODE_BLINK_OFFLINE,  /* 0.5 Hz symmetric blink */
  LED_MODE_DOUBLE_BLINK,   /* double blink: on 150 - off 150 - on 150 - off 1050 (low power) */
  LED_MODE_BLINK_ERR,      /* continuous 5 Hz fast blink: error indication */
  LED_MODE_WRITE_PULSE,    /* write pulse: on 150 - off 850 cycle */
};

/* Mode indicator semantics (PA15), passed to led_mode_indicator() */
#define LED_APPMODE_GS_USB    0

void led_init( void );
void led_set_mode( int led, int mode, uint32_t arg );
void led_poll( void );
void led_turnoff_all(void);
void led_tx_blink(uint32_t ch);

/* Mode indicator (PA15): steady on in gs_usb mode */
void led_mode_indicator(int app_mode);
