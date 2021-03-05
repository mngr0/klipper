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
    SERCOM2->USART.INTENSET.reg = SERCOM_USART_INTENSET_DRE;
}

void
SERCOM2_Handler(void)
{
    uint32_t status = SERCOM2->USART.INTFLAG.reg;
    if (status & SERCOM_USART_INTFLAG_RXC)
        serial_rx_byte(SERCOM2->USART.DATA.reg);
    if (status & SERCOM_USART_INTFLAG_DRE) {
        uint8_t data;
        int ret = serial_get_tx_byte(&data);
        if (ret)
            SERCOM2->USART.INTENCLR.reg = SERCOM_USART_INTENSET_DRE;
        else
            SERCOM2->USART.DATA.reg = data;
    }
}

#if CONFIG_MACH_SAME54
DECL_CONSTANT_STR("RESERVE_PINS_serial", "PB24,PB25");
#warning "same54 serial"
void
serial_init(void)
{
    // Enable serial clock
    enable_pclock(SERCOM2_GCLK_ID_CORE, ID_SERCOM2);
    // Enable pins
    gpio_peripheral(GPIO('B', 24), 'D', 0);
    gpio_peripheral(GPIO('B', 25), 'D', 0);
    // Configure serial
    SercomUsart *su = &SERCOM2->USART;
    su->CTRLA.reg = 0;
    uint32_t areg = (SERCOM_USART_CTRLA_MODE(1)
                     | SERCOM_USART_CTRLA_DORD
                     | SERCOM_USART_CTRLA_SAMPR(1)
                     | SERCOM_USART_CTRLA_RXPO(1)
                     | SERCOM_USART_CTRLA_TXPO(0));
    su->CTRLA.reg = areg;
    su->CTRLB.reg = SERCOM_USART_CTRLB_RXEN | SERCOM_USART_CTRLB_TXEN;
    uint32_t freq = get_pclock_frequency(SERCOM2_GCLK_ID_CORE);
    uint32_t baud8 = freq / (2 * CONFIG_SERIAL_BAUD);
    su->BAUD.reg = (SERCOM_USART_BAUD_FRAC_BAUD(baud8 / 8)
                    | SERCOM_USART_BAUD_FRAC_FP(baud8 % 8));
    // enable irqs
    su->INTENSET.reg = SERCOM_USART_INTENSET_RXC;
    su->CTRLA.reg = areg | SERCOM_USART_CTRLA_ENABLE;
// #if CONFIG_MACH_SAME54
    armcm_enable_irq(SERCOM2_Handler, SERCOM2_0_IRQn, 0);
    armcm_enable_irq(SERCOM2_Handler, SERCOM2_1_IRQn, 0);
    armcm_enable_irq(SERCOM2_Handler, SERCOM2_2_IRQn, 0);
    armcm_enable_irq(SERCOM2_Handler, SERCOM2_3_IRQn, 0);
// #endif
}
DECL_INIT(serial_init);
#else
DECL_CONSTANT_STR("RESERVE_PINS_serial", "PA12,PA13");
#warning "same51 serial"
void
serial_init(void)
{
    // Enable serial clock
    enable_pclock(SERCOM2_GCLK_ID_CORE, ID_SERCOM2);
    // Enable pins
    gpio_peripheral(GPIO('A', 12), 'C', 0);
    gpio_peripheral(GPIO('A', 13), 'C', 0);
    // Configure serial
    SercomUsart *su = &SERCOM2->USART;
    su->CTRLA.reg = 0;
    uint32_t areg = (SERCOM_USART_CTRLA_MODE(1)
                     | SERCOM_USART_CTRLA_DORD
                     | SERCOM_USART_CTRLA_SAMPR(1)
                     | SERCOM_USART_CTRLA_RXPO(0)
                     | SERCOM_USART_CTRLA_TXPO(1));
    su->CTRLA.reg = areg;
    su->CTRLB.reg = SERCOM_USART_CTRLB_RXEN | SERCOM_USART_CTRLB_TXEN;
    uint32_t freq = get_pclock_frequency(SERCOM2_GCLK_ID_CORE);
    uint32_t baud8 = freq / (2 * CONFIG_SERIAL_BAUD);
    su->BAUD.reg = (SERCOM_USART_BAUD_FRAC_BAUD(baud8 / 8)
                    | SERCOM_USART_BAUD_FRAC_FP(baud8 % 8));
    // enable irqs
    su->INTENSET.reg = SERCOM_USART_INTENSET_RXC;
    su->CTRLA.reg = areg | SERCOM_USART_CTRLA_ENABLE;
// #if CONFIG_MACH_SAME54
    armcm_enable_irq(SERCOM2_Handler, SERCOM2_0_IRQn, 0);
    armcm_enable_irq(SERCOM2_Handler, SERCOM2_1_IRQn, 0);
    armcm_enable_irq(SERCOM2_Handler, SERCOM2_2_IRQn, 0);
    armcm_enable_irq(SERCOM2_Handler, SERCOM2_3_IRQn, 0);
// #endif
}
DECL_INIT(serial_init);
#endif