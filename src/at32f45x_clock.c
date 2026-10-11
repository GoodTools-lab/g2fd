/**
  **************************************************************************
  * @file     at32f45x_clock.c
  * @brief    system clock config program
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

/* includes ------------------------------------------------------------------*/
#include "at32f45x_clock.h"
#include "at32f45x_misc.h"

/** @addtogroup 456_TMR_timer_base TMR_timer_base
  * @{
  */
volatile uint32_t system_tick = 0;

crm_clocks_freq_type crm_clocks_freq_struct = {0};

/**
  * @brief  system clock config program
  * @note   the system clock is configured as follow:
  *         - system clock        = (hext * pll_ns)/(pll_ms * pll_fp)
  *         - system clock source = pll (hext)
  *         - hext                = 8000000
  *         - sclk                = 160000000
  *         - ahbdiv              = 1
  *         - ahbclk              = 160000000
  *         - apb2div             = 1
  *         - apb2clk             = 160000000
  *         - apb1div             = 1
  *         - apb1clk             = 160000000
  *         - pll_ns              = 120
  *         - pll_ms              = 1
  *         - pll_fp              = 6
  *         - pllu                = 48000000 (vco 960mhz / fu 20, usb clock)
  *
  *         note: 160mhz = 32 * 5mbps, so the can fd data phase at 5mbps can be
  *               prescaled exactly; usb still runs on pllu 48mhz and is not
  *               affected by the sclk change
  * @param  none
  * @retval none
  */
void system_clock_config(void)
{
    /* reset crm */
    crm_reset();

    /* set the flash clock divider (160mhz: 129~160mhz -> 4 wait cycles) */
    flash_psr_set(FLASH_WAIT_CYCLE_4);

      /* enable pwc periph clock */
    crm_periph_clock_enable(CRM_PWC_PERIPH_CLOCK, TRUE);
  
    /* config ldo voltage */
    pwc_ldo_output_voltage_set(PWC_LDO_OUTPUT_1V3);

    crm_clock_source_enable(CRM_CLOCK_SOURCE_HEXT, TRUE);

    /* wait till hext is ready */
    while(crm_hext_stable_wait() == ERROR)
    {
    }

    /* config pll clock resource: vco = 8mhz * 120 / 1 = 960mhz */
    crm_pll_config(CRM_PLL_SOURCE_HEXT, 120, 1, CRM_PLL_FP_6);

    /* config pllu divider: pllu = 960mhz / 20 = 48mhz (usb) */
    crm_pllu_div_set(CRM_PLL_FU_20);

    /* pllu enable  */
    crm_pllu_output_set(TRUE);

    /* enable pll */
    crm_clock_source_enable(CRM_CLOCK_SOURCE_PLL, TRUE);

    /* wait till pll is ready */
    while(crm_flag_get(CRM_PLL_STABLE_FLAG) != SET)
    {
    }

    /* config ahbclk */
    crm_ahb_div_set(CRM_AHB_DIV_1);

    /* config apb3clk, the maximum frequency of APB3 clock is 90 MHz */
    crm_apb3_div_set(CRM_APB3_DIV_4);

    /* config apb2clk, the maximum frequency of APB2 clock is 192 MHz */
    crm_apb2_div_set(CRM_APB2_DIV_1);

    /* config apb1clk, the maximum frequency of APB1 clock is 192 MHz */
    crm_apb1_div_set(CRM_APB1_DIV_1);

    /* enable auto step mode */
    crm_auto_step_mode_enable(TRUE);

    /* select pll as system clock source */
    crm_sysclk_switch(CRM_SCLK_PLL);

    /* wait till pll is used as system clock source */
    while(crm_sysclk_switch_status_get() != CRM_SCLK_PLL)
    {
    }

    /* disable auto step mode */
    crm_auto_step_mode_enable(FALSE);

    /* update system_core_clock global variable */
    system_core_clock_update();

    /* vector table at flash base (standalone image, no bootloader offset) */
    nvic_vector_table_set(NVIC_VECTTAB_FLASH, 0x0);
}

void system_tick_config(void)
{
    /* get system clock */
    crm_clocks_freq_get(&crm_clocks_freq_struct);
    /* enable tmr1 clock */
    crm_periph_clock_enable(CRM_TMR1_PERIPH_CLOCK, TRUE);

    /* tmr1 configuration */
    /* time base configuration */
    tmr_base_init(TMR1, 999, (crm_clocks_freq_struct.apb2_freq / 1000000) - 1);
    tmr_cnt_dir_set(TMR1, TMR_COUNT_UP);

    /* overflow interrupt enable */
    tmr_interrupt_enable(TMR1, TMR_OVF_INT, TRUE);

    /* tmr1 hall interrupt nvic init */
    nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
    nvic_irq_enable(TMR1_OVF_TMR10_IRQn, 1, 0);

    /* enable tmr1 */
    tmr_counter_enable(TMR1, TRUE);
}

/**
  * @brief  timer1 overflow interrupt handler (1ms system tick)
  */
void TMR1_OVF_TMR10_IRQHandler(void)
{
    if(tmr_interrupt_flag_get(TMR1, TMR_OVF_FLAG) == SET)
    {
        tmr_flag_clear(TMR1, TMR_OVF_FLAG);
        system_tick++;
    }
}

uint32_t HAL_GetTick(void)
{
    return system_tick;

}
