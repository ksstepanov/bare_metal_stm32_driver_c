/*
 * stm32f412_exti.h
 *
 *  Created on: May 26, 2025
 *      Author: konstantin
 */

#ifndef INC_STM32F412_EXTI_H_
#define INC_STM32F412_EXTI_H_
#include "stm32f412disco.h"
#include "stm32f412_nvic_config.h"
#include "stm32f412disco_gpio.h"

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

typedef enum {
	EXTI_LINE_GPIO0 = GPIO_PIN0,
	EXTI_LINE_GPIO1 = GPIO_PIN1,
	EXTI_LINE_GPIO2 = GPIO_PIN2,
	EXTI_LINE_GPIO3 = GPIO_PIN3,
	EXTI_LINE_GPIO4 = GPIO_PIN4,
	EXTI_LINE_GPIO5 = GPIO_PIN5,
	EXTI_LINE_GPIO6 = GPIO_PIN6,
	EXTI_LINE_GPIO7 = GPIO_PIN7,
	EXTI_LINE_GPIO8 = GPIO_PIN8,
	EXTI_LINE_GPIO9 = GPIO_PIN9,
	EXTI_LINE_GPIO10 = GPIO_PIN10,
	EXTI_LINE_GPIO11 = GPIO_PIN11,
	EXTI_LINE_GPIO12 = GPIO_PIN12,
	EXTI_LINE_GPIO13 = GPIO_PIN13,
	EXTI_LINE_GPIO14 = GPIO_PIN14,
	EXTI_LINE_GPIO15 = GPIO_PIN15,
	EXTI_LINE_PVD = 16,
	EXTI_LINE_RTC_ALARM = 17,
	EXTI_LINE_USB_OTG_FS_WAKEUP = 18,
	EXTI_LINE_RESERVED_19 = 19,
	EXTI_LINE_RESERVED_20 = 20,
	EXTI_LINE_RTC_TAMPER_TIMESTAMP = 21,
	EXTI_LINE_RTC_WAKEUP = 22,
	_EXTI_NUM_LINES
} exti_line_num_t;

/*
 * @brief     EXTI line to NVIC IRQ mapping
 * @param[in] pin from exti_line_num_t
 */
static inline nvic_interrupt_number_t hal_exti_line_to_nvic_irq(exti_line_num_t exti_line)
{
	assert(exti_line >= 0 && exti_line < _EXTI_NUM_LINES &&
			exti_line != EXTI_LINE_RESERVED_19 && exti_line != EXTI_LINE_RESERVED_20);

	if (exti_line < EXTI_LINE_GPIO5) {
		return exti_line + NVIC_IRQ_EXTI0;
	} else if (exti_line >= EXTI_LINE_GPIO5 && exti_line < EXTI_LINE_GPIO10) {
		return NVIC_IRQ_EXTI9_5;
	} else if (exti_line >= EXTI_LINE_GPIO10 && exti_line <= EXTI_LINE_GPIO15) {
		return NVIC_IRQ_EXTI15_10;
	} else if (exti_line == EXTI_LINE_PVD) {
		return NVIC_IRQ_PVD;
	} else if (exti_line == EXTI_LINE_RTC_ALARM) {
		return NVIC_IRQ_RTC_Alarm;
	} else if (exti_line == EXTI_LINE_USB_OTG_FS_WAKEUP) {
		return NVIC_IRQ_OTG_FS_WKUP;
	} else if (exti_line == EXTI_LINE_RTC_TAMPER_TIMESTAMP) {
		return NVIC_IRQ_TAMP_STAMP;
	} else {  /* if (exti_line == EXTI_LINE_RTC_WAKEUP) Ok not to check because of assert */
		return NVIC_IRQ_RTC_WKUP;
	}
}

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
 * @param[in] pin from exti_line_num_t
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
 * @param[in] line_number from exti_line_num_t
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

/*
 * @brief     clear pending interrupt request
 * @param[in] line_number from exti_line_num_t
 */
static inline void hal_exti_interrupt_clear(uint8_t line_number)
{
	EXTI_RegDef_t *const pEXTI = EXTI;
	if (pEXTI->PR & (1 << line_number)) {
		pEXTI->PR |= (1 << line_number);
	}
}
#endif /* INC_STM32F412_EXTI_H_ */
