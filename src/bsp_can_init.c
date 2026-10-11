/*
 * G2FD - three-channel CANFD gs_usb adapter firmware (AT32F456)
 * Copyright (c) 2026 G2FD contributors
 * SPDX-License-Identifier: MIT
 */

#include "at32f45x.h"
#include "at32f45x_board.h"
#include "led.h"
#include "bsp_can.h"
#include "bsp_can_init.h"
#include "gs_usb_protocol.h"
#include <string.h>

/* macro definition about some parameters of can bit time */
#define BITTIME_DIV                      2
#define AC_BTS1_SIZE                     76
#define AC_BTS2_SIZE                     20
#define AC_RSAW_SIZE                     7

#define FD_BTS1_SIZE                     18
#define FD_BTS2_SIZE                     6
#define FD_RSAW_SIZE                     3
#define FD_SSP_OFFSET                    19
/* the calculation process of can_clk is as follows:
   can_clk = can_pclk/BITTIME_DIV = 192M/2 = 96M; 
*/
/* the calculation process of nominal bit rate is as follows: 
   ac_baudrate = can_clk/(AC_BTS1_SIZE + AC_BTS2_SIZE) = 96M/(144 + 48) = 500k;
   ac_samplepoint = AC_BTS1_SIZE/(AC_BTS1_SIZE + AC_BTS2_SIZE) = 144/192 = 75%;
*/
/* the calculation process of data bit rate is as follows: 
   fd_baudrate = can_clk/(FD_BTS1_SIZE + FD_BTS2_SIZE) = 96M/(36 + 12) = 2M;
   fd_samplepoint = FD_BTS1_SIZE/(FD_BTS1_SIZE + FD_BTS2_SIZE) = 36/48 = 75%;
*/
/* Secondary Sample Point(SSP) is suggested to set the FD_SSP_OFFSET equal to FD_BTS1_SIZE+1. */


volatile uint32_t can_rx_cnt = 0;
volatile uint32_t can2_rx_cnt = 0;
volatile uint32_t can3_rx_cnt = 0;

volatile uint32_t can_tx_cnt = 0;

volatile uint32_t can_err_cnt = 0;
volatile uint32_t can2_err_cnt = 0;
volatile uint32_t can3_err_cnt = 0;
volatile uint32_t can3_err1_cnt = 0;
volatile uint32_t can3_err2_cnt = 0;
volatile uint32_t can3_err3_cnt = 0;

/* Per-channel RX FIFO hardware overflow counters: an overflow occurs when
 * the channel's RX ISR stays blocked longer than FIFO depth x frame interval
 * (3 x ~250us @ 4000fps); overflowed frames are dropped by hardware while the
 * controller still ACKs them at the protocol level (invisible to the transmitter).
 * CAN1=can_rx_ovf_cnt, CAN2=can2_rx_ovf_cnt, CAN3=can3_err3_cnt (pre-existing). */
volatile uint32_t can_rx_ovf_cnt = 0;
volatile uint32_t can2_rx_ovf_cnt = 0;

/**
  *  @brief  can gpio config
  *  @param  none
  *  @retval none
  */
static void can_gpio_config(void)
{
    gpio_init_type gpio_init_struct;

    /* enable the gpio clock */
    crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOC_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOD_PERIPH_CLOCK, TRUE);

    gpio_default_para_init(&gpio_init_struct);

    /* configure the can tx, rx, stb pin */
    /* can_stb connect to the transceiver <stby> pin, default output low, keep the transceiver
        running in normal mode */
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
    gpio_init_struct.gpio_pins = GPIO_PINS_8 | GPIO_PINS_9;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init(GPIOB, &gpio_init_struct);
    //gpio_init_struct.gpio_pins = GPIO_PINS_7;
    //gpio_init(GPIOB, &gpio_init_struct);

    gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE8, GPIO_MUX_9);
    gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE9, GPIO_MUX_9);
    //gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE7, GPIO_MUX_11);


    /* configure the STB gpio */
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init_struct.gpio_out_type  = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
    gpio_init_struct.gpio_pins = GPIO_PINS_7;
    gpio_init_struct.gpio_pull = GPIO_PULL_UP;
    gpio_init(GPIOB, &gpio_init_struct);
    gpio_bits_reset(GPIOB, GPIO_PINS_7);

    /* configure the CAN1 Wakeup gpio */
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init_struct.gpio_out_type  = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
    gpio_init_struct.gpio_pins = GPIO_PINS_0;
    gpio_init_struct.gpio_pull = GPIO_PULL_UP;
    gpio_init(GPIOC, &gpio_init_struct);
}

static void can2_gpio_config(void)
{
    gpio_init_type gpio_init_struct;

    /* enable the gpio clock */
    crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOC_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOD_PERIPH_CLOCK, TRUE);

    gpio_default_para_init(&gpio_init_struct);

    /* configure the can tx, rx, stb pin */
    /* can_stb connect to the transceiver <stby> pin, default output low, keep the transceiver
        running in normal mode */
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
    gpio_init_struct.gpio_pins = GPIO_PINS_5 | GPIO_PINS_6;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init(GPIOB, &gpio_init_struct);
    //gpio_init_struct.gpio_pins = GPIO_PINS_7;
    //gpio_init(GPIOB, &gpio_init_struct);

    gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE5, GPIO_MUX_9);
    gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE6, GPIO_MUX_9);
    //gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE7, GPIO_MUX_11);


    /* configure the STB gpio */
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init_struct.gpio_out_type  = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
    gpio_init_struct.gpio_pins = GPIO_PINS_12;
    gpio_init_struct.gpio_pull = GPIO_PULL_UP;
    gpio_init(GPIOC, &gpio_init_struct);
    gpio_bits_reset(GPIOC, GPIO_PINS_12);

    /* configure the CAN1 Wakeup gpio */
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init_struct.gpio_out_type  = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
    gpio_init_struct.gpio_pins = GPIO_PINS_1;
    gpio_init_struct.gpio_pull = GPIO_PULL_UP;
    gpio_init(GPIOC, &gpio_init_struct);
}


static void can3_gpio_config(void)
{
    gpio_init_type gpio_init_struct;

    /* enable the gpio clock */
    crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOC_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOD_PERIPH_CLOCK, TRUE);

    gpio_default_para_init(&gpio_init_struct);

    /* configure the can tx, rx, stb pin */
    /* can_stb connect to the transceiver <stby> pin, default output low, keep the transceiver
        running in normal mode */
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
    gpio_init_struct.gpio_pins = GPIO_PINS_3 | GPIO_PINS_4;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init(GPIOB, &gpio_init_struct);

    gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE3, GPIO_MUX_11);
    gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE4, GPIO_MUX_11);


    /* configure the STB gpio */
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init_struct.gpio_out_type  = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
    gpio_init_struct.gpio_pins = GPIO_PINS_10;
    gpio_init_struct.gpio_pull = GPIO_PULL_UP;
    gpio_init(GPIOC, &gpio_init_struct);
    gpio_bits_reset(GPIOC, GPIO_PINS_10);

    /* configure the CAN1 Wakeup gpio */
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init_struct.gpio_out_type  = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
    gpio_init_struct.gpio_pins = GPIO_PINS_2;
    gpio_init_struct.gpio_pull = GPIO_PULL_UP;
    gpio_init(GPIOC, &gpio_init_struct);
}

/**
  *  @brief  can configiguration.
  *  @param  none
  *  @retval the result of can_configuration
  *          this parameter can be one of the following values:
  *          SUCCESS or ERROR
  */
error_status can_configuration(void)
{
    can_bittime_type can_bittime_struct;

    /* as specified in can protocol, the maximum allowable oscillator tolerance is 1%. 
    The hick accuracy does not meet the clock requirements in can protocol. to guarantee normal 
    communication, it is recommended to use hext as the system clock source. */
    if(crm_flag_get(CRM_HEXT_STABLE_FLAG) != SET)
    {
    return ERROR;
    }

    /* enable the can clock */
    crm_periph_clock_enable(CRM_CAN1_PERIPH_CLOCK, TRUE);
    crm_can_clock_select(CRM_CAN1, CRM_CAN_CLOCK_SOURCE_PCLK);

    can_software_reset(CAN1, TRUE);

    /* set can bit time */
    can_bittime_default_para_init(&can_bittime_struct);
    can_bittime_struct.bittime_div = BITTIME_DIV;
    can_bittime_struct.ac_bts1_size = AC_BTS1_SIZE;
    can_bittime_struct.ac_bts2_size = AC_BTS2_SIZE;
    can_bittime_struct.ac_rsaw_size = AC_RSAW_SIZE;

    can_bittime_struct.fd_bts1_size = FD_BTS1_SIZE;
    can_bittime_struct.fd_bts2_size = FD_BTS2_SIZE;
    can_bittime_struct.fd_rsaw_size = FD_RSAW_SIZE;
    can_bittime_struct.fd_ssp_offset = FD_SSP_OFFSET;
    can_bittime_set(CAN1, &can_bittime_struct);

    can_fd_iso_mode_enable(CAN1, TRUE);
    can_stb_transmit_mode_set(CAN1, CAN_STB_TRANSMIT_BY_FIFO);    

    can_software_reset(CAN1, FALSE);

    can_retransmission_limit_set(CAN1, CAN_RE_TRANS_TIMES_UNLIMIT);
    can_rearbitration_limit_set(CAN1, CAN_RE_ARBI_TIMES_UNLIMIT);
    can_rxbuf_warning_set(CAN1, 3);
    can_mode_set(CAN1, CAN_MODE_COMMUNICATE);

    /* can interrupt config */
    #if CAN_USE_ISR
    nvic_irq_enable(CAN1_RX_IRQn, 1, 0);
    nvic_irq_enable(CAN1_TX_IRQn, 1, 0);
    nvic_irq_enable(CAN1_ERR_IRQn, 2, 0);
    #endif
    can_interrupt_enable(CAN1, CAN_RAFIE_INT|CAN_RFIE_INT|CAN_ROIE_INT|CAN_RIE_INT, TRUE);
#if CAN_ERROR_FRAME_ENABLE
    can_interrupt_enable(CAN1, CAN_BEIE_INT, TRUE);
    /* Error interrupt enables for BERR_REPORTING: bus error (BEIE), error/passive
     * state change (EIE+EPIE). ALIE not enabled (arbitration loss is normal). */
    can_interrupt_enable(CAN1, CAN_EIE_INT, TRUE);
    can_interrupt_enable(CAN1, CAN_EPIE_INT, TRUE);
#endif
    can_interrupt_enable(CAN1, CAN_TSIE_INT, TRUE);

    return SUCCESS;
}


/**
  *  @brief  can configiguration.
  *  @param  none
  *  @retval the result of can_configuration
  *          this parameter can be one of the following values:
  *          SUCCESS or ERROR
  */
error_status can2_configuration(void)
{
    can_bittime_type can_bittime_struct;

    /* as specified in can protocol, the maximum allowable oscillator tolerance is 1%. 
    The hick accuracy does not meet the clock requirements in can protocol. to guarantee normal 
    communication, it is recommended to use hext as the system clock source. */
    if(crm_flag_get(CRM_HEXT_STABLE_FLAG) != SET)
    {
    return ERROR;
    }

    /* enable the can clock */
    crm_periph_clock_enable(CRM_CAN2_PERIPH_CLOCK, TRUE);
    crm_can_clock_select(CRM_CAN2, CRM_CAN_CLOCK_SOURCE_PCLK);

    can_software_reset(CAN2, TRUE);

    /* set can bit time */
    can_bittime_default_para_init(&can_bittime_struct);
    can_bittime_struct.bittime_div = BITTIME_DIV;
    can_bittime_struct.ac_bts1_size = AC_BTS1_SIZE;
    can_bittime_struct.ac_bts2_size = AC_BTS2_SIZE;
    can_bittime_struct.ac_rsaw_size = AC_RSAW_SIZE;

    can_bittime_struct.fd_bts1_size = FD_BTS1_SIZE;
    can_bittime_struct.fd_bts2_size = FD_BTS2_SIZE;
    can_bittime_struct.fd_rsaw_size = FD_RSAW_SIZE;
    can_bittime_struct.fd_ssp_offset = FD_SSP_OFFSET;
    can_bittime_set(CAN2, &can_bittime_struct);

    can_fd_iso_mode_enable(CAN2, TRUE);
    can_stb_transmit_mode_set(CAN2, CAN_STB_TRANSMIT_BY_FIFO);    

    can_software_reset(CAN2, FALSE);

    can_retransmission_limit_set(CAN2, CAN_RE_TRANS_TIMES_UNLIMIT);
    can_rearbitration_limit_set(CAN2, CAN_RE_ARBI_TIMES_UNLIMIT);
    can_rxbuf_warning_set(CAN2, 3);
    can_mode_set(CAN2, CAN_MODE_COMMUNICATE);

    /* can interrupt config */
    #if CAN_USE_ISR
    /* CAN2 priority is kept identical to CAN1/CAN3 so CAN2 cannot preempt the
     * CAN1/CAN3 RX ISRs and cause concurrent access to the SRAM ring buffers.
     * With all three channels at the same priority, the initial version (72b8101)
     * runs without lockups. */
    nvic_irq_enable(CAN2_RX_IRQn, 1, 0);
    nvic_irq_enable(CAN2_TX_IRQn, 1, 0);
    nvic_irq_enable(CAN2_ERR_IRQn, 2, 0);
    #endif
    can_interrupt_enable(CAN2, CAN_RAFIE_INT|CAN_RFIE_INT|CAN_ROIE_INT|CAN_RIE_INT, TRUE);
#if CAN_ERROR_FRAME_ENABLE
    can_interrupt_enable(CAN2, CAN_BEIE_INT, TRUE);
    /* Error interrupt enables for BERR_REPORTING (see CAN1 config for details). */
    can_interrupt_enable(CAN2, CAN_EIE_INT, TRUE);
    can_interrupt_enable(CAN2, CAN_EPIE_INT, TRUE);
#endif
    can_interrupt_enable(CAN2, CAN_TSIE_INT, TRUE);

    return SUCCESS;
}


/**
  *  @brief  can configiguration.
  *  @param  none
  *  @retval the result of can_configuration
  *          this parameter can be one of the following values:
  *          SUCCESS or ERROR
  */
error_status can3_configuration(void)
{
    can_bittime_type can_bittime_struct;

    /* as specified in can protocol, the maximum allowable oscillator tolerance is 1%. 
    The hick accuracy does not meet the clock requirements in can protocol. to guarantee normal 
    communication, it is recommended to use hext as the system clock source. */
    if(crm_flag_get(CRM_HEXT_STABLE_FLAG) != SET)
    {
    return ERROR;
    }

    /* enable the can clock */
    crm_periph_clock_enable(CRM_CAN3_PERIPH_CLOCK, TRUE);
    crm_can_clock_select(CRM_CAN3, CRM_CAN_CLOCK_SOURCE_PCLK);

    can_software_reset(CAN3, TRUE);

    /* set can bit time */
    can_bittime_default_para_init(&can_bittime_struct);
    can_bittime_struct.bittime_div = BITTIME_DIV;
    can_bittime_struct.ac_bts1_size = AC_BTS1_SIZE;
    can_bittime_struct.ac_bts2_size = AC_BTS2_SIZE;
    can_bittime_struct.ac_rsaw_size = AC_RSAW_SIZE;

    can_bittime_struct.fd_bts1_size = FD_BTS1_SIZE;
    can_bittime_struct.fd_bts2_size = FD_BTS2_SIZE;
    can_bittime_struct.fd_rsaw_size = FD_RSAW_SIZE;
    can_bittime_struct.fd_ssp_offset = FD_SSP_OFFSET;
    can_bittime_set(CAN3, &can_bittime_struct);

    can_fd_iso_mode_enable(CAN3, TRUE);
    can_stb_transmit_mode_set(CAN3, CAN_STB_TRANSMIT_BY_FIFO);    

    can_software_reset(CAN3, FALSE);

    can_retransmission_limit_set(CAN3, CAN_RE_TRANS_TIMES_UNLIMIT);
    can_rearbitration_limit_set(CAN3, CAN_RE_ARBI_TIMES_UNLIMIT);
    can_rxbuf_warning_set(CAN3, 3);
    can_mode_set(CAN3, CAN_MODE_COMMUNICATE);

    /* can interrupt config */
    #if CAN_USE_ISR
    nvic_irq_enable(CAN3_RX_IRQn, 1, 0);
    nvic_irq_enable(CAN3_TX_IRQn, 1, 0);
    nvic_irq_enable(CAN3_ERR_IRQn, 2, 0);
    #endif
    can_interrupt_enable(CAN3, CAN_RAFIE_INT|CAN_RFIE_INT|CAN_ROIE_INT|CAN_RIE_INT, TRUE);
#if CAN_ERROR_FRAME_ENABLE
    can_interrupt_enable(CAN3, CAN_BEIE_INT, TRUE);
    /* Error interrupt enables for BERR_REPORTING (see CAN1 config for details). */
    can_interrupt_enable(CAN3, CAN_EIE_INT, TRUE);
    can_interrupt_enable(CAN3, CAN_EPIE_INT, TRUE);
#endif
    can_interrupt_enable(CAN3, CAN_TSIE_INT, TRUE);

    return SUCCESS;
}


void can_init(void)
{
    /* Initialize software state (queues, counters) */
    can_sw_init();

    /* Initialize CAN1 hardware (bit timing taken from g_can_config[2], defaults or host-set via gs_usb) */
    can_gpio_config();
    can_configuration_();

    /* Initialize CAN2 hardware (bit timing taken from g_can_config[1]) */
    can2_gpio_config();
    can2_configuration_();

    /* Initialize CAN3 hardware (bit timing taken from g_can_config[0]) */
    can3_gpio_config();
    can3_configuration_();
}

/* ====== Hardware power-down state tracking ======
 * The open-source slim build has no offline-recording/power-saving feature: all three
 * channels stay powered and enabled, so g_can_ch_disabled is always 0. The tracking is
 * kept for defensive checks by other modules (bus-off recovery / filter polling). */
static uint8_t g_can_ch_disabled[3] = {0, 0, 0};

uint8_t can_ch_is_disabled(uint32_t channel)
{
    return (channel < 3u) ? g_can_ch_disabled[channel] : 1u;
}

/**
  *  @brief  can transmit data
  *  @param  none
  *  @retval none
  */
void can_transmit_data(void)
{
    can_txbuf_type can_txbuf_struct = {0};

    /* write transmit TEST_EXT_ID buffer */
    can_txbuf_struct.id = 123;
    can_txbuf_struct.id_type = CAN_ID_EXTENDED;
    can_txbuf_struct.frame_type = CAN_FRAME_DATA;
    can_txbuf_struct.fd_format = CAN_FORMAT_FD;//CAN_FORMAT_FD;
    can_txbuf_struct.fd_rate_switch = CAN_BRS_ON;
    can_txbuf_struct.data_length = CAN_DLC_BYTES_64;
    can_txbuf_struct.data[0] = 0x00;
    can_txbuf_struct.data[1] = 0x11;
    can_txbuf_struct.data[2] = 0x22;
    can_txbuf_struct.data[3] = 0x33;
    can_txbuf_struct.data[4] = 0x44;
    can_txbuf_struct.data[5] = 0x55;
    can_txbuf_struct.data[6] = 0x66;
    can_txbuf_struct.data[7] = 0x77;
    can_txbuf_struct.data[63] = 0x63;
    while(can_txbuf_write(CAN1, CAN_TXBUF_STB, &can_txbuf_struct) != SUCCESS);
    while(can_txbuf_write(CAN2, CAN_TXBUF_STB, &can_txbuf_struct) != SUCCESS);

    /* write transmit TEST_STD_ID buffer */
    can_txbuf_struct.id = 223;
    can_txbuf_struct.id_type = CAN_ID_STANDARD;
    can_txbuf_struct.frame_type = CAN_FRAME_DATA;
    can_txbuf_struct.fd_format = CAN_FORMAT_CLASSIC;
    can_txbuf_struct.fd_rate_switch = CAN_BRS_OFF;
    can_txbuf_struct.data_length = CAN_DLC_BYTES_8;
    can_txbuf_struct.data[0] = 0x00;
    can_txbuf_struct.data[1] = 0x11;
    can_txbuf_struct.data[2] = 0x22;
    can_txbuf_struct.data[3] = 0x33;
    can_txbuf_struct.data[4] = 0x44;
    can_txbuf_struct.data[5] = 0x55;
    can_txbuf_struct.data[6] = 0x66;
    can_txbuf_struct.data[7] = 0x77;
    //can_txbuf_struct.data[63] = 0x63;
    while(can_txbuf_write(CAN1, CAN_TXBUF_STB, &can_txbuf_struct) != SUCCESS);
    while(can_txbuf_write(CAN2, CAN_TXBUF_STB, &can_txbuf_struct) != SUCCESS);

    /* transmit all frames of the secondary transmit buffer */
    can_txbuf_transmit(CAN1, CAN_TRANSMIT_STB_ALL);
    while(can_flag_get(CAN1, CAN_TSIF_FLAG) != SET);
    can_flag_clear(CAN1, CAN_TSIF_FLAG);

    can_txbuf_transmit(CAN2, CAN_TRANSMIT_STB_ALL);
    while(can_flag_get(CAN2, CAN_TSIF_FLAG) != SET);
    can_flag_clear(CAN2, CAN_TSIF_FLAG);
}


/**
  *  @brief  can1 interrupt function rx, map to CAN3
  *  @param  none
  *  @retval none
  */
void CAN1_RX_IRQHandler(void)
{
    can_rxbuf_type can_rxbuf_struct;

    /* rx_buffer had data be received */
    if(can_interrupt_flag_get(CAN1, CAN_RIF_FLAG) != RESET)
    {
        can_flag_clear(CAN1, CAN_RIF_FLAG);
        while(ERROR != can_rxbuf_read(CAN1, &can_rxbuf_struct))
        {
            can_stub_push_rx_from_isr(2, &can_rxbuf_struct);  /* CAN1 = gs_usb ch=2 */
            led_set_mode(LED_CH2, LED_MODE_BLINK_FAST, 300 );
        }
        can_rx_cnt++;
    }

    /* rx_buffer almost full */
    if(can_interrupt_flag_get(CAN1, CAN_RAFIF_FLAG) != RESET)
    {
        can_flag_clear(CAN1, CAN_RAFIF_FLAG);
    }

    /* rx_buffer full */
    if(can_interrupt_flag_get(CAN1, CAN_RFIF_FLAG) != RESET)
    {
        can_flag_clear(CAN1, CAN_RFIF_FLAG);
    }

    /* rx_buffer overflow: frame dropped by hardware (counter lets the main loop attribute frame loss) */
    if(can_interrupt_flag_get(CAN1, CAN_ROIF_FLAG) != RESET)
    {
        can_rx_ovf_cnt++;
        can_flag_clear(CAN1, CAN_ROIF_FLAG);
    }
}

/**
  *  @brief  can1 interrupt function rx
  *  @param  none
  *  @retval none
  */
void CAN2_RX_IRQHandler(void)
{
    can_rxbuf_type can_rxbuf_struct;

    /* rx_buffer had data be received */
    if(can_interrupt_flag_get(CAN2, CAN_RIF_FLAG) != RESET)
    {
        can_flag_clear(CAN2, CAN_RIF_FLAG);
        while(ERROR != can_rxbuf_read(CAN2, &can_rxbuf_struct))
        {
            can_stub_push_rx_from_isr(1, &can_rxbuf_struct);  /* CAN2 = gs_usb ch=1 */
            led_set_mode(LED_CH1, LED_MODE_BLINK_FAST, 300 );
        }
        can2_rx_cnt++;
    }

    /* rx_buffer almost full */
    if(can_interrupt_flag_get(CAN2, CAN_RAFIF_FLAG) != RESET)
    {
        can_flag_clear(CAN2, CAN_RAFIF_FLAG);
    }

    /* rx_buffer full */
    if(can_interrupt_flag_get(CAN2, CAN_RFIF_FLAG) != RESET)
    {
        can_flag_clear(CAN2, CAN_RFIF_FLAG);
    }

    /* rx_buffer overflow: frame dropped by hardware (counter lets the main loop attribute frame loss) */
    if(can_interrupt_flag_get(CAN2, CAN_ROIF_FLAG) != RESET)
    {
        can2_rx_ovf_cnt++;
        can_flag_clear(CAN2, CAN_ROIF_FLAG);
    }
}


/**
  *  @brief  can1 interrupt function rx, map to CAN1
  *  @param  none
  *  @retval none
  */
void CAN3_RX_IRQHandler(void)
{
    can_rxbuf_type can_rxbuf_struct;

    /* rx_buffer had data be received */
    if(can_interrupt_flag_get(CAN3, CAN_RIF_FLAG) != RESET)
    {
        can_flag_clear(CAN3, CAN_RIF_FLAG);
        while(ERROR != can_rxbuf_read(CAN3, &can_rxbuf_struct))
        {
            can_stub_push_rx_from_isr(0, &can_rxbuf_struct);  /* CAN3 = gs_usb ch=0 */
            led_set_mode(LED_CH0, LED_MODE_BLINK_FAST, 300 );
            can3_rx_cnt++;
        }

    }

    /* rx_buffer almost full */
    if(can_interrupt_flag_get(CAN3, CAN_RAFIF_FLAG) != RESET)
    {
        can3_err1_cnt++;
        can_flag_clear(CAN3, CAN_RAFIF_FLAG);
    }

    /* rx_buffer full */
    if(can_interrupt_flag_get(CAN3, CAN_RFIF_FLAG) != RESET)
    {
        can3_err2_cnt++;
        can_flag_clear(CAN3, CAN_RFIF_FLAG);
    }

    /* rx_buffer overflow */
    if(can_interrupt_flag_get(CAN3, CAN_ROIF_FLAG) != RESET)
    {
        can3_err3_cnt++;
        can_flag_clear(CAN3, CAN_ROIF_FLAG);
    }
}

/**
  *  @brief  Common CAN error ISR handler.
  *         Decodes AT32F45x CAN error sources (KOER/ALC/TECNT/RECNT) and stores
  *         SocketCAN-style error frames in a per-channel pending ring buffer.
  *         The main loop later drains the buffer and pushes to the RX queue.
  *
  *         This deferred-push design avoids PSRAM cache access from the ERR ISR,
  *         eliminating reentrancy risks with the RX ISR and reducing ISR overhead.
  *
  *         Error sources handled:
  *         - BEIF (bus error): decode KOER → CAN_ERR_PROT (bit/form/stuff/CRC) or CAN_ERR_ACK
  *         - ALIF (arbitration lost): → CAN_ERR_LOSTARB, data[0]=ALC bit position
  *           (ALIE is NOT enabled by default — ALIF only fires if ALIE is manually enabled)
  *         - EPIF (error passive state change): → CAN_ERR_CRTL, data[1]=RX/TX passive/warning
  *         - EIF  (error warning / bus-off): → CAN_ERR_BUSOFF when BUSOFF status set
  *
  *         KOER and ALC are cleared together with BEIF/ALIF, so ERR register is
  *         read BEFORE clearing any flags.
  *
  * @param  can_x: CAN instance (CAN1/CAN2/CAN3)
  * @param  ch: gs_usb channel index (0=CAN3, 1=CAN2, 2=CAN1)
  * @param  err_cnt_ptr: pointer to the per-channel error counter
  */
static void can_err_isr_common(can_type *can_x, uint8_t ch, volatile uint32_t *err_cnt_ptr)
{
    /* Snapshot ERR register first: KOER and ALC are cleared when BEIF/ALIF are cleared */
    uint32_t err_reg = can_x->err;
    uint8_t  koer = (err_reg >> 13) & 0x07;  /* bits [15:13] */
    uint8_t  alc  = (err_reg >> 8) & 0x1F;   /* bits [12:8] */
    uint8_t  tecnt = (err_reg >> 24) & 0xFF; /* bits [31:24] */
    uint8_t  recnt = (err_reg >> 16) & 0xFF; /* bits [23:16] */
    uint32_t sts = can_x->sts;
    uint8_t  data[8];

    (*err_cnt_ptr)++;

    /* --- BEIF: bus error (bit/form/stuff/ack/crc) --- */
    if (can_interrupt_flag_get(can_x, CAN_BEIF_FLAG) != RESET)
    {
        can_flag_clear(can_x, CAN_BEIF_FLAG);

        memset(data, 0, 8);
        /* data[6]=TECNT, data[7]=RECNT per SocketCAN can_frame layout for error counters */
        data[6] = tecnt;
        data[7] = recnt;

        uint32_t err_type = 0;
        uint8_t  prot_type = 0;  /* data[2] for CAN_ERR_PROT */
        uint8_t  prot_loc = 0;   /* data[3] for CAN_ERR_PROT */

        switch (koer) {
            case 1:  /* Bit Error */
                err_type = CAN_ERR_PROT;
                prot_type = CAN_ERR_PROT_BIT;
                break;
            case 2:  /* Form Error */
                err_type = CAN_ERR_PROT;
                prot_type = CAN_ERR_PROT_FORM;
                break;
            case 3:  /* Stuff Error */
                err_type = CAN_ERR_PROT;
                prot_type = CAN_ERR_PROT_STUFF;
                break;
            case 4:  /* ACK Error */
                err_type = CAN_ERR_ACK;
                break;
            case 5:  /* CRC Error */
                err_type = CAN_ERR_PROT;
                prot_loc = CAN_ERR_PROT_LOC_CRC;
                break;
            case 6:  /* Other Error */
                err_type = CAN_ERR_BUSERROR;
                break;
            default: /* 0 = no error (shouldn't happen with BEIF set) */
                err_type = CAN_ERR_BUSERROR;
                break;
        }

        if (err_type == CAN_ERR_PROT) {
            data[2] = prot_type;
            data[3] = prot_loc;
        }
        /* Store in pending buffer — main loop pushes to RX queue */
        can_err_isr_store(ch, err_type, data);
    }

    /* --- ALIF: arbitration lost (only if ALIE is manually enabled) --- */
    if (can_interrupt_flag_get(can_x, CAN_ALIF_FLAG) != RESET)
    {
        can_flag_clear(can_x, CAN_ALIF_FLAG);

        memset(data, 0, 8);
        data[0] = alc;  /* arbitration lost bit position */
        data[6] = tecnt;
        data[7] = recnt;
        /* Store in pending buffer — main loop pushes to RX queue */
        can_err_isr_store(ch, CAN_ERR_LOSTARB, data);
    }

    /* --- EPIF: error passive state change --- */
    if (can_interrupt_flag_get(can_x, CAN_EPIF_FLAG) != RESET)
    {
        can_flag_clear(can_x, CAN_EPIF_FLAG);

        memset(data, 0, 8);
        /* data[1] encodes which counter crossed the passive threshold (≥128) */
        if (sts & CAN_EPASS_FLAG) {
            /* Entered error passive */
            if (tecnt >= 128) data[1] |= CAN_ERR_CRTL_TX_PASSIVE;
            if (recnt >= 128) data[1] |= CAN_ERR_CRTL_RX_PASSIVE;
        } else {
            /* Left error passive (back to warning or active) */
            if (sts & CAN_EWARN_FLAG) {
                if (tecnt >= 96) data[1] |= CAN_ERR_CRTL_TX_WARNING;
                if (recnt >= 96) data[1] |= CAN_ERR_CRTL_RX_WARNING;
            }
        }
        data[6] = tecnt;
        data[7] = recnt;
        /* Store in pending buffer — main loop pushes to RX queue */
        can_err_isr_store(ch, CAN_ERR_CRTL, data);
    }

    /* --- EIF: error warning / bus-off --- */
    if (can_interrupt_flag_get(can_x, CAN_EIF_FLAG) != RESET)
    {
        can_flag_clear(can_x, CAN_EIF_FLAG);

        if (can_busoff_get(can_x) == SET) {
            /* Bus-off entered */
            memset(data, 0, 8);
            data[6] = tecnt;
            data[7] = recnt;
            /* Store in pending buffer — main loop pushes to RX queue.
             * CAN_ERR_BUSOFF bypasses rate limiting (critical event). */
            can_err_isr_store(ch, CAN_ERR_BUSOFF, data);
        }
        /* Error warning (EWARN set, not bus-off) is reported by EPIF above
         * when the state transitions. EIF alone for EWARN doesn't generate
         * a separate frame to avoid duplicate notifications. */
    }

    /* Always refresh the cached CAN controller state for GET_STATE queries */
    can_update_error_state(ch, can_x);
}

/**
  *  @brief  can1 interrupt function error (CAN1 = gs_usb ch=2)
  *  @param  none
  *  @retval none
  */
void CAN1_ERR_IRQHandler(void)
{
    can_err_isr_common(CAN1, 2, &can_err_cnt);
}

/**
  *  @brief  can2 interrupt function error (CAN2 = gs_usb ch=1)
  *  @param  none
  *  @retval none
  */
void CAN2_ERR_IRQHandler(void)
{
    can_err_isr_common(CAN2, 1, &can2_err_cnt);
}


/**
  *  @brief  can3 interrupt function error (CAN3 = gs_usb ch=0)
  *  @param  none
  *  @retval none
  */
void CAN3_ERR_IRQHandler(void)
{
    can_err_isr_common(CAN3, 0, &can3_err_cnt);
}

/**
  *  @brief  can1 tx complete interrupt function (CAN1 = gs_usb ch=2)
  *          Triggered when STB transmission completes; advances the echo queue's
  *          complete_idx so the main loop only echoes frames actually transmitted
  *          onto the CAN bus.
  *  @param  none
  *  @retval none
  */
void CAN1_TX_IRQHandler(void)
{
    if(can_interrupt_flag_get(CAN1, CAN_TSIF_FLAG) != RESET)
    {
        can_flag_clear(CAN1, CAN_TSIF_FLAG);
        g_can_tx_complete[2]++;          /* CAN1 = gs_usb ch=2 */
        can_stub_notify_tx_complete(2);  /* advance the echo queue complete_idx */
    }
}

/**
  *  @brief  can2 tx complete interrupt function (CAN2 = gs_usb ch=1)
  *  @param  none
  *  @retval none
  */
void CAN2_TX_IRQHandler(void)
{
    if(can_interrupt_flag_get(CAN2, CAN_TSIF_FLAG) != RESET)
    {
        can_flag_clear(CAN2, CAN_TSIF_FLAG);
        g_can_tx_complete[1]++;          /* CAN2 = gs_usb ch=1 */
        can_stub_notify_tx_complete(1);  /* advance the echo queue complete_idx */
    }
}

/**
  *  @brief  can3 tx complete interrupt function (CAN3 = gs_usb ch=0)
  *  @param  none
  *  @retval none
  */
void CAN3_TX_IRQHandler(void)
{
    if(can_interrupt_flag_get(CAN3, CAN_TSIF_FLAG) != RESET)
    {
        can_flag_clear(CAN3, CAN_TSIF_FLAG);
        g_can_tx_complete[0]++;          /* CAN3 = gs_usb ch=0 */
        can_stub_notify_tx_complete(0);  /* advance the echo queue complete_idx */
    }
}

/**
  * @brief  Set bit timing for all CAN interfaces (gs_usb protocol support)
  * @param  bittime: pointer to bit timing structure
  * @retval none
  */
void can_set_bittime_all(can_bittime_type *bittime)
{
    /* Apply new bit timing to all CAN interfaces */
    can_software_reset(CAN1, TRUE);
    can_bittime_set(CAN1, bittime);
    can_software_reset(CAN1, FALSE);
    
    can_software_reset(CAN2, TRUE);
    can_bittime_set(CAN2, bittime);
    can_software_reset(CAN2, FALSE);
    
    can_software_reset(CAN3, TRUE);
    can_bittime_set(CAN3, bittime);
    can_software_reset(CAN3, FALSE);
}

/**
  * @brief  Enable all CAN interfaces (gs_usb protocol support)
  * @param  none
  * @retval none
  */
void can_enable_all(void)
{
    /* Set to communication mode for all CAN interfaces */
    can_mode_set(CAN1, CAN_MODE_COMMUNICATE);
    can_mode_set(CAN2, CAN_MODE_COMMUNICATE);
    can_mode_set(CAN3, CAN_MODE_COMMUNICATE);
}

/**
  * @brief  Disable all CAN interfaces (gs_usb protocol support)
  * @param  none
  * @retval none
  */
void can_disable_all(void)
{

}

