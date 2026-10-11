/*
 * G2FD - three-channel CANFD gs_usb adapter firmware (AT32F456)
 * Copyright (c) 2026 G2FD contributors
 * SPDX-License-Identifier: MIT
 */

#include <assert.h>
#include "at32f45x.h"
#include "at32f45x_board.h"
#include "led.h"


/* ====== Pin assignments (all six LEDs are green) ====== */
#define LED_CH0_PORT         GPIOA
#define LED_CH0_PIN          GPIO_PINS_10

#define LED_CH1_PORT         GPIOA
#define LED_CH1_PIN          GPIO_PINS_8

#define LED_CH2_PORT         GPIOC
#define LED_CH2_PIN          GPIO_PINS_6

#define LED_LIN_PORT         GPIOB
#define LED_LIN_PIN          GPIO_PINS_12

#define LED_STAT_PORT         GPIOA
#define LED_STAT_PIN          GPIO_PINS_15

#define LED_REC_PORT          GPIOB
#define LED_REC_PIN           GPIO_PINS_2

#define LED_CH0_OFF()    LED_CH0_PORT->clr = LED_CH0_PIN
#define LED_CH0_ON()     LED_CH0_PORT->scr = LED_CH0_PIN

#define LED_CH1_OFF()    LED_CH1_PORT->clr = LED_CH1_PIN
#define LED_CH1_ON()     LED_CH1_PORT->scr = LED_CH1_PIN

#define LED_CH2_OFF()    LED_CH2_PORT->clr = LED_CH2_PIN
#define LED_CH2_ON()     LED_CH2_PORT->scr = LED_CH2_PIN

#define LED_LIN_OFF()    LED_LIN_PORT->clr = LED_LIN_PIN
#define LED_LIN_ON()     LED_LIN_PORT->scr = LED_LIN_PIN

#define LED_STAT_OFF()    LED_STAT_PORT->clr = LED_STAT_PIN
#define LED_STAT_ON()     LED_STAT_PORT->scr = LED_STAT_PIN

#define LED_REC_OFF()     LED_REC_PORT->clr = LED_REC_PIN
#define LED_REC_ON()      LED_REC_PORT->scr = LED_REC_PIN

void led_turnoff_all(void);
static void _led_update_state( int led, uint8_t state );

/* ====== Blink timing tables (ms): hi[]=on-phase sequence, lo[]=off-phase sequence, repeated ======
 * Data-LED patterns (FAST/SLOW/OFFLINE) keep the original symmetric rhythm;
 * Status-LED patterns (DOUBLE/ERR/PULSE) use asymmetric timing */
typedef struct
{
  const uint16_t *hi;
  const uint16_t *lo;
  uint8_t hi_len;
  uint8_t lo_len;
} led_pattern_t;

static const uint16_t pat_hi_fast[]    = {50};
static const uint16_t pat_lo_fast[]    = {50};
static const uint16_t pat_hi_slow[]    = {200};
static const uint16_t pat_lo_slow[]    = {200};
static const uint16_t pat_hi_offline[] = {1000};
static const uint16_t pat_lo_offline[] = {1000};
static const uint16_t pat_hi_double[]  = {150, 150};
static const uint16_t pat_lo_double[]  = {150, 1050};
static const uint16_t pat_hi_err[]     = {100};
static const uint16_t pat_lo_err[]     = {100};
static const uint16_t pat_hi_pulse[]   = {150};
static const uint16_t pat_lo_pulse[]   = {850};

static const led_pattern_t *led_pattern_get(int mode)
{
  switch (mode)
  {
    case LED_MODE_BLINK_FAST:    { static const led_pattern_t p = {pat_hi_fast,    pat_lo_fast,    1, 1}; return &p; }
    case LED_MODE_BLINK_SLOW:    { static const led_pattern_t p = {pat_hi_slow,    pat_lo_slow,    1, 1}; return &p; }
    case LED_MODE_BLINK_OFFLINE: { static const led_pattern_t p = {pat_hi_offline, pat_lo_offline, 1, 1}; return &p; }
    case LED_MODE_DOUBLE_BLINK:  { static const led_pattern_t p = {pat_hi_double,  pat_lo_double,  2, 2}; return &p; }
    case LED_MODE_BLINK_ERR:     { static const led_pattern_t p = {pat_hi_err,     pat_lo_err,     1, 1}; return &p; }
    case LED_MODE_WRITE_PULSE:   { static const led_pattern_t p = {pat_hi_pulse,   pat_lo_pulse,   1, 1}; return &p; }
    default: return 0;
  }
}

static struct
{
  uint32_t arg;       /* 0xFFFFFFFF = never expires, otherwise absolute time when the LED auto-switches to OFF */
  uint32_t timestamp;
  uint16_t mode;
  uint8_t  state;     /* current output level */
  uint8_t  phase;     /* blink phase index (cycles through hi[]/lo[]) */
}
led_mode_array[LED_TOTAL] = { 0 };

uint32_t timestamp_millis( void )
{
  return  HAL_GetTick();
}


void led_init( void )
{
  gpio_init_type gpio_init_struct;
  /* enable the led clock */
  crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_GPIOC_PERIPH_CLOCK, TRUE);

  /* set default parameter */
  gpio_default_para_init(&gpio_init_struct);

  /* configure the led gpio */
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init_struct.gpio_out_type  = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init_struct.gpio_pins = LED_CH0_PIN;
  gpio_init(LED_CH0_PORT, &gpio_init_struct);

  gpio_init_struct.gpio_pins = LED_CH1_PIN;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init(LED_CH1_PORT, &gpio_init_struct);

  gpio_init_struct.gpio_pins = LED_CH2_PIN;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init(LED_CH2_PORT, &gpio_init_struct);

  gpio_init_struct.gpio_pins = LED_LIN_PIN;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init(LED_LIN_PORT, &gpio_init_struct);

  gpio_init_struct.gpio_pins = LED_STAT_PIN;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init(LED_STAT_PORT, &gpio_init_struct);

  gpio_init_struct.gpio_pins = LED_REC_PIN;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init(LED_REC_PORT, &gpio_init_struct);

  led_turnoff_all();
}

void led_set_mode( int led, int mode, uint32_t arg )
{
  //assert( led < LED_TOTAL );

  /* Repeated calls in blink mode (data LEDs are retriggered once per frame):
   * only refresh the timeout arg, do not reset phase/timestamp -- otherwise,
   * under high-rate traffic the timestamp keeps being refreshed, the phase
   * never toggles, and the LED never blinks */
  if ( led_mode_array[led].mode == mode &&
       ( mode == LED_MODE_BLINK_FAST || mode == LED_MODE_BLINK_SLOW ||
         mode == LED_MODE_BLINK_OFFLINE || mode == LED_MODE_DOUBLE_BLINK ||
         mode == LED_MODE_BLINK_ERR || mode == LED_MODE_WRITE_PULSE ) )
  {
    if (arg != 0xFFFFFFFF)
    {
      arg = timestamp_millis() + arg;
    }
    led_mode_array[led].arg = arg;
    return;
  }

  led_mode_array[led].mode = mode;
  led_mode_array[led].state = 0;    /* start from the off phase, matching the original rhythm */
  led_mode_array[led].phase = 0;
  led_mode_array[led].timestamp = timestamp_millis();

  if (arg != 0xFFFFFFFF)
  {
    /* update to absolute */
    arg = timestamp_millis() + arg;
  }
  led_mode_array[led].arg = arg;

  _led_update_state( led, 0 );
}

static void _led_update_state( int led, uint8_t state )
{
  switch( led )
  {
    case LED_STAT:
      if ( state )
        LED_STAT_ON();
      else
        LED_STAT_OFF();
      break;
    case LED_REC:
      if ( state )
        LED_REC_ON();
      else
        LED_REC_OFF();
      break;
    case LED_LIN:
      if ( state )
        LED_LIN_ON();
      else
        LED_LIN_OFF();
      break;
    case LED_CH0:
      if( state )
        LED_CH0_ON();
      else
        LED_CH0_OFF();
      break;
    case LED_CH1:
      if( state )
        LED_CH1_ON();
      else
        LED_CH1_OFF();
      break;
    case LED_CH2:
      if( state )
        LED_CH2_ON();
      else
        LED_CH2_OFF();
      break;
    default:
      break;
  }
}

/* Duration of the current phase: on -> hi[phase], off -> lo[phase] */
static uint16_t _led_phase_ms( int led )
{
  const led_pattern_t *p = led_pattern_get( led_mode_array[led].mode );
  if ( p == 0 )
    return 0;
  return led_mode_array[led].state
       ? p->hi[led_mode_array[led].phase]
       : p->lo[led_mode_array[led].phase % p->lo_len];
}

void led_poll( void )
{
  uint32_t ts_ms = timestamp_millis();
  uint32_t i = 0;

  for(i = 0; i < LED_TOTAL; i++ )
  {
    if( !led_mode_array[i].timestamp  )
      continue;
    if( (uint32_t)( ts_ms - led_mode_array[i].timestamp ) < _led_phase_ms(i) )
      continue;

    switch( led_mode_array[i].mode )
    {
      default:
      case LED_MODE_DEVICE:
        led_mode_array[i].timestamp = 0;
        break;
      case LED_MODE_OFF:
      case LED_MODE_ON:
        led_mode_array[i].state = ( led_mode_array[i].mode == LED_MODE_ON );
        led_mode_array[i].timestamp = 0;
        break;
      case LED_MODE_BLINK_FAST:
      case LED_MODE_BLINK_SLOW:
      case LED_MODE_BLINK_OFFLINE:
      case LED_MODE_DOUBLE_BLINK:
      case LED_MODE_BLINK_ERR:
      case LED_MODE_WRITE_PULSE:
      {
        const led_pattern_t *p = led_pattern_get( led_mode_array[i].mode );
        if( led_mode_array[i].state )
        {
          /* On phase ended -> go off, phase unchanged (off duration taken from lo[phase]) */
          led_mode_array[i].state = 0;
        }
        else
        {
          /* Off phase ended -> go on, advance to the next on phase */
          led_mode_array[i].state = 1;
          led_mode_array[i].phase++;
          if( p && led_mode_array[i].phase >= p->hi_len )
            led_mode_array[i].phase = 0;
        }
        led_mode_array[i].timestamp = ts_ms;
        if( ( led_mode_array[i].arg != 0xFFFFFFFF ) && ( led_mode_array[i].arg <= ts_ms ) )
        {
          led_set_mode( i, LED_MODE_OFF, 0 );
        }
        break;
      }
    }

    _led_update_state( i, led_mode_array[i].state );
  }
}

void led_turnoff_all(void)
{
    LED_CH0_OFF();
    LED_CH1_OFF();
    LED_CH2_OFF();
    LED_LIN_OFF();
    LED_STAT_OFF();
    LED_REC_OFF();
}

void led_tx_blink(uint32_t ch)
{
  if (ch < LED_TOTAL)
    led_set_mode((int)ch, LED_MODE_BLINK_FAST, 300);
}

/* ====== Mode indicator (PA15) ====== */
static int mode_indicator_last = -1;

void led_mode_indicator(int app_mode)
{
  int want = (app_mode == LED_APPMODE_GS_USB) ? LED_MODE_ON : LED_MODE_OFF;

  if (mode_indicator_last != want)
  {
    mode_indicator_last = want;
    led_set_mode(LED_STAT, want, 0xFFFFFFFF);
  }
}
