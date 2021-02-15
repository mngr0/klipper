// samd21 serial port
//
// Copyright (C) 2018-2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "board/armcm_boot.h" // armcm_enable_irq
#include "board/serial_irq.h" // serial_rx_data
#include "command.h" // DECL_CONSTANT_STR
#include "internal.h" // enable_pclock
#include "sched.h" // DECL_INIT

void
serial_enable_tx_irq(void)
{
    SERCOM0->USART.INTENSET.reg = SERCOM_USART_INTENSET_DRE;
}

void
SERCOM0_Handler(void)
{
    uint32_t status = SERCOM0->USART.INTFLAG.reg;
    if (status & SERCOM_USART_INTFLAG_RXC)
        serial_rx_byte(SERCOM0->USART.DATA.reg);
    if (status & SERCOM_USART_INTFLAG_DRE) {
        uint8_t data;
        int ret = serial_get_tx_byte(&data);
        if (ret)
            SERCOM0->USART.INTENCLR.reg = SERCOM_USART_INTENSET_DRE;
        else
            SERCOM0->USART.DATA.reg = data;
    }
}

DECL_CONSTANT_STR("RESERVE_PINS_serial", "PA05,PA04");

void
serial_init(void)
{
    // Enable serial clock
    enable_pclock(SERCOM0_GCLK_ID_CORE, ID_SERCOM0);
    // Enable pins
    gpio_peripheral(GPIO('A', 4), 'D', 0);
    gpio_peripheral(GPIO('A', 5), 'D', 0);
    // Configure serial



	// if (!hri_sercomusart_is_syncing(hw, SERCOM_USART_SYNCBUSY_SWRST)) {
	// 	uint32_t mode = _usarts[i].ctrl_a & SERCOM_USART_CTRLA_MODE_Msk;
	// 	if (hri_sercomusart_get_CTRLA_reg(hw, SERCOM_USART_CTRLA_ENABLE)) {
	// 		hri_sercomusart_clear_CTRLA_ENABLE_bit(hw);
	// 		hri_sercomusart_wait_for_sync(hw, SERCOM_USART_SYNCBUSY_ENABLE);
	// 	}
	// 	hri_sercomusart_write_CTRLA_reg(hw, SERCOM_USART_CTRLA_SWRST | mode);
	// }
	// hri_sercomusart_wait_for_sync(hw, SERCOM_USART_SYNCBUSY_SWRST);

	// hri_sercomusart_write_CTRLA_reg(hw, _usarts[i].ctrl_a);
	// hri_sercomusart_write_CTRLB_reg(hw, _usarts[i].ctrl_b);
	// hri_sercomusart_write_CTRLC_reg(hw, _usarts[i].ctrl_c);
	// if ((_usarts[i].ctrl_a & SERCOM_USART_CTRLA_SAMPR(0x1)) || (_usarts[i].ctrl_a & SERCOM_USART_CTRLA_SAMPR(0x3))) {
	// 	((Sercom *)hw)->USART.BAUD.FRAC.BAUD = _usarts[i].baud;
	// 	((Sercom *)hw)->USART.BAUD.FRAC.FP   = _usarts[i].fractional;
	// } else {
	// 	hri_sercomusart_write_BAUD_reg(hw, _usarts[i].baud);
	// }

	// hri_sercomusart_write_RXPL_reg(hw, _usarts[i].rxpl);
	// hri_sercomusart_write_DBGCTRL_reg(hw, _usarts[i].debug_ctrl);




    SercomUsart *su = &SERCOM0->USART;
    su->CTRLA.reg = 0;
    uint32_t areg = (SERCOM_USART_CTRLA_MODE(1)
                     | SERCOM_USART_CTRLA_DORD
                     | SERCOM_USART_CTRLA_SAMPR(1)
                     | SERCOM_USART_CTRLA_RXPO(0)
                     | SERCOM_USART_CTRLA_TXPO(1));
    su->CTRLA.reg = areg;
    su->CTRLB.reg = SERCOM_USART_CTRLB_RXEN | SERCOM_USART_CTRLB_TXEN;

    uint32_t freq = get_pclock_frequency(SERCOM0_GCLK_ID_CORE);
    uint32_t baud8 = freq / (2 * CONFIG_SERIAL_BAUD);
    su->BAUD.reg = (SERCOM_USART_BAUD_FRAC_BAUD(baud8 / 8)
                    | SERCOM_USART_BAUD_FRAC_FP(baud8 % 8));
    // enable irqs
    su->INTENSET.reg = SERCOM_USART_INTENSET_RXC;
    su->CTRLA.reg = areg | SERCOM_USART_CTRLA_ENABLE;
// #if CONFIG_MACH_SAME54
    armcm_enable_irq(SERCOM0_Handler, SERCOM0_0_IRQn, 0);
    armcm_enable_irq(SERCOM0_Handler, SERCOM0_1_IRQn, 0);
    armcm_enable_irq(SERCOM0_Handler, SERCOM0_2_IRQn, 0);
    armcm_enable_irq(SERCOM0_Handler, SERCOM0_3_IRQn, 0);
// #endif
}
DECL_INIT(serial_init);
