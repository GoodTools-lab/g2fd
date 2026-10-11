/*
 * G2FD - three-channel CANFD gs_usb adapter firmware (AT32F456)
 * Copyright (c) 2026 G2FD contributors
 * SPDX-License-Identifier: MIT
 */

#include "bsp_usb.h"

otg_core_type otg_core_struct;
/**
  * @brief  usb 48M clock select
  * @param  clk_s:USB_CLK_HICK, USB_CLK_HEXT
  * @retval none
  */
void usb_clock48m_select(usb_clk48_s clk_s)
{
  if(clk_s == USB_CLK_HICK)
  {
    crm_usb_clock_source_select(CRM_USB_CLOCK_SOURCE_HICK);

    /* enable the acc calibration ready interrupt */
    crm_periph_clock_enable(CRM_ACC_PERIPH_CLOCK, TRUE);

    /* update the c1\c2\c3 value */
    acc_write_c1(7980);
    acc_write_c2(8000);
    acc_write_c3(8020);

    /* open acc calibration */
    acc_calibration_mode_enable(ACC_CAL_HICKTRIM, TRUE);
  }
  else
  {
    crm_usb_clock_source_select(CRM_USB_CLOCK_SOURCE_PLLU);
  }
}


/**
  * @brief  this function config gpio.
  * @param  none
  * @retval none
  */
void usb_gpio_config(void)
{
  gpio_init_type gpio_init_struct;

  crm_periph_clock_enable(OTG_PIN_GPIO_CLOCK, TRUE);
  gpio_default_para_init(&gpio_init_struct);

  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init_struct.gpio_out_type  = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
}

#ifdef USB_LOW_POWER_WAKUP
/**
  * @brief  usb low power wakeup interrupt config
  * @param  none
  * @retval none
  */
void usb_low_power_wakeup_config(void)
{
  exint_init_type exint_init_struct;

  crm_periph_clock_enable(CRM_SCFG_PERIPH_CLOCK, TRUE);
  exint_default_para_init(&exint_init_struct);

  exint_init_struct.line_enable = TRUE;
  exint_init_struct.line_mode = EXINT_LINE_INTERRUPT;
  exint_init_struct.line_select = OTG_WKUP_EXINT_LINE;
  exint_init_struct.line_polarity = EXINT_TRIGGER_RISING_EDGE;
  exint_init(&exint_init_struct);

  /* USB interrupt priority (2,0) is lower than CAN RX (1,0) so the CAN RX ISR
   * can preempt the USB interrupt. This prevents CAN hardware buffer overflow
   * and frame loss during high-rate recording, where USB interrupts would
   * otherwise delay the CAN RX ISR.
   * In the initial version (72b8101) CAN RX was (0,0), the same priority as
   * USB, so USB could not preempt CAN RX. */
  nvic_irq_enable(OTG_WKUP_IRQ, 2, 0);
}

/**
  * @brief  this function handles otgfs wakup interrupt.
  * @param  none
  * @retval none
  */
void OTG_WKUP_HANDLER(void)
{
  exint_flag_clear(OTG_WKUP_EXINT_LINE);
}

#endif

/**
  * @brief  this function handles otgfs interrupt.
  * @param  none
  * @retval none
  */
void OTG_IRQ_HANDLER(void)
{
  usbd_irq_handler(&otg_core_struct);
}

void usb_init(void)
{
    usb_gpio_config();
    /* enable otgfs clock */
    crm_periph_clock_enable(OTG_CLOCK, TRUE);

    /* select usb 48m clcok source */
    usb_clock48m_select(USB_CLK_HEXT);

    /* enable otgfs irq -- priority (2,0) lower than CAN RX (1,0), see OTG_WKUP_IRQ comment */
    nvic_irq_enable(OTG_IRQ, 2, 0);

    /* init usb */
    usbd_init(&otg_core_struct,
            USB_FULL_SPEED_CORE_ID,
            USB_ID,
            &winusb_class_handler,
            &winusb_desc_handler);
}
