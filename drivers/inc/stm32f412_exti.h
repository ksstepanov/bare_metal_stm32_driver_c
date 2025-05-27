/*
 * stm32f412_exti.h
 *
 *  Created on: May 26, 2025
 *      Author: konstantin
 */

#ifndef INC_STM32F412_EXTI_H_
#define INC_STM32F412_EXTI_H_
#include "stm32f412disco.h"

/*
 * The main features of the EXTI controller are the following:
 * 1) independent trigger and mask on each interrupt/event line
 * 2) dedicated status bit for each interrupt line
 * 3) generation of up to 23 software event/interrupt requests
 * 4) detection of external signals with a pulse width lower than the APB2 clock period. Refer
 *    to the electrical characteristics section of the STM32F4xx datasheets for details on this
 *    parameter.
 */

typedef struct {
	/* [0x00] Interrupt mask register
	 * MR[18:0] [22:21]: Interrupt mask on line x
	 */
	reg_t IMR;

	/* [0x04] Event mask register
	 * MR[18:0] [22:21]: Event mask on line x
	 */
	reg_t EMR;

	/* [0x08] Rising trigger selection register
	 * TR[18:0] [22:21]: Rising trigger event configuration bit of line x
	 */
	reg_t RTSR;

	/* [0x0C] Falling trigger selection register
	 * FR[18:0] [22:21]: Falling trigger event configuration bit of line x
	 */
	reg_t FTSR;

	/* [0x10] Software interrupt event register
	 * SWIER[18:0] [22:21]: Software Interrupt on line x
	 * If interrupt are enabled on line x in the EXTI_IMR register, writing '1' to SWIERx bit when it is
	 * set at '0' sets the corresponding pending bit in the EXTI_PR register, thus resulting in an
	 * interrupt request generation.
	 * This bit is cleared by clearing the corresponding bit in EXTI_PR (by writing a 1 to the bit).
	 */
	reg_t SWIER;

	/* [0x14] Pending register
	 * PR[18:0] [22:21]: Pending bit
	 */
	reg_t PR;
} EXTI_RegDef_t;

#define EXTI ((EXTI_RegDef_t *)EXTI_BASE)

/*
 * @brief     configure falling trigger detection
 * @param[in] pin from gpio_pin_t or 21-22
 */
static inline void hal_exti_gpio_falling_trigger_conf(uint8_t pin, hal_enable_disable_t val)
{
	EXTI_RegDef_t *const pEXTI = EXTI;
	if (val == HAL_ENABLE) {
		pEXTI->FTSR |= (1 << pin);
	} else {
		pEXTI->FTSR &= ~(1 << pin);
	}
}

/*
 * @brief     configure rising trigger detection
 * @param[in] pin from gpio_pin_t or 21-22
 */
static inline void hal_exti_gpio_rising_trigger_conf(uint8_t pin, hal_enable_disable_t val)
{
	EXTI_RegDef_t *const pEXTI = EXTI;
	if (val == HAL_ENABLE) {
		pEXTI->RTSR |= (1 << pin);
	} else {
		pEXTI->RTSR &= ~(1 << pin);
	}
}

/*
 * @brief     enable or disable interrupt line
 * @param[in] line_number from gpio_pin_t or 21-22
 */
static inline void hal_exti_int_line_conf(uint8_t line_number, hal_enable_disable_t val)
{
	EXTI_RegDef_t *const pEXTI = EXTI;
	if (val == HAL_DISABLE) {
		pEXTI->IMR &= ~(1 << line_number);
	} else {
		pEXTI->IMR |= (1 << line_number);
	}
}

static inline void hal_exti_interrupt_clear(uint8_t line_number)
{
	EXTI_RegDef_t *const pEXTI = EXTI;
	if (pEXTI->PR & (1 << line_number)) {
		pEXTI->PR |= (1 << line_number);
	}
}
#endif /* INC_STM32F412_EXTI_H_ */
