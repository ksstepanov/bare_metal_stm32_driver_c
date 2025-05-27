/*
 * stm32f412disco_gpio.c
 *
 *  Created on: Apr 5, 2025
 *      Author: konstantin
 */


#include "stm32f412disco_gpio.h"
#include "stm32f412_rcc.h"
#include "stm32f412_exti.h"
#include "stm32f412_syscfg.h"
#include "arm_cortex_m4_nvic.h"
#include "assert.h"
#include <stdlib.h>

/**
 * @brief Maps enum gpio_port_t to actual GPIO addresses
 */
static GPIO_RegDef_t *const gpio_port_to_address[NUM_OF_GPIOS] = {
		GPIOA, /* GPIOA_PORT */
		GPIOB, /* GPIOB_PORT */
		GPIOC, /* GPIOC_PORT */
		GPIOD, /* GPIOD_PORT */
		GPIOE, /* GPIOE_PORT */
		GPIOF, /* GPIOF_PORT */
		GPIOG, /* GPIOG_PORT */
		GPIOH  /* GPIOH_PORT */
};

#define IS_VALID_PORT(X) ((X >= 0) && (X < NUM_OF_GPIOS))
#define IS_VALID_PIN(X) ((X >= 0) && (X <= GPIO_PIN15))
#define IS_VALID_MODE(X) ((X >= 0) && (X <= GPIO_ANALOG))
#define IS_VALID_SPEED(X) ((X >= 0) && (X <= GPIO_OUT_SP_VHIGH))
#define IS_VALID_PU_PD(X) ((X >= 0) && (X <= GPIO_PIN_PD))
#define IS_VALID_OUT_TYPE(MODE, OTYPE)\
	(((MODE == GPIO_OUTPUT) && (OTYPE >= 0) && (OTYPE <= GPIO_OUT_OPEN_DRAIN)) ||\
			((MODE != GPIO_OUTPUT) && (OTYPE == GPIO_OUT_DEFAULT)))
#define IS_VALID_AFUNCTION(MODE, AFUNCTION)\
	(((MODE == GPIO_ALT_FUNC) && (AFUNCTION >= 0) && (AFUNCTION <= 0xF)) ||\
			((MODE != GPIO_ALT_FUNC) && (AFUNCTION == GPIO_AF_DEFAULT_VAL)))
#define IS_VALID_INT_TRIGGER(MODE, INT_TRIGGER)\
	(((MODE == GPIO_INPUT) && (INT_TRIGGER >= GPIO_IN_INT_TRIG_DISABLED) && (INT_TRIGGER <= GPIO_IN_INT_TRIG_RISING_FALLING)) ||\
			((MODE != GPIO_INPUT) && (INT_TRIGGER == GPIO_IN_INT_TRIG_DISABLED)))

static inline void validate_pin_cfg(const GPIO_PinCfg_t *pGPIO_conf)
{
	assert(pGPIO_conf != NULL);
    assert(IS_VALID_PORT(pGPIO_conf->portNumber));
    assert(IS_VALID_PIN(pGPIO_conf->pinNumber));
    assert(IS_VALID_MODE(pGPIO_conf->pinMode));
    assert(IS_VALID_SPEED(pGPIO_conf->pinSpeed));
    assert(IS_VALID_PU_PD(pGPIO_conf->pinPuPdControl));
    assert(IS_VALID_OUT_TYPE(pGPIO_conf->pinMode, pGPIO_conf->pinOpType));
    assert(IS_VALID_AFUNCTION(pGPIO_conf->pinMode, pGPIO_conf->pinAFmode));
    assert(IS_VALID_INT_TRIGGER(pGPIO_conf->pinMode, pGPIO_conf->pinInIntTrig));
}

/**
 * @brief     Enable or disable clock
 * @param[in] GPIO port from gpio_port_num_t
 * @param[in] Enable or disable
 * */
static inline void hal_gpio_perip_clk_ctrl(gpio_port_num_t port_number, hal_enable_disable_t val)
{
	if (val == HAL_ENABLE) {
		hal_rcc_gpio_pclk_en(port_number);
	} else {
		hal_rcc_gpio_pclk_di(port_number);
	}
}

static inline void hal_gpio_enable_interrupt(const GPIO_Handle_t *pGPIO)
{
	/* 1. Configure rising and falling edge trigger: */
    if (pGPIO->PinCfg.pinInIntTrig == GPIO_IN_INT_TRIG_FALLING) {
    	hal_exti_gpio_falling_trigger_conf(pGPIO->PinCfg.pinNumber, HAL_ENABLE);
    	hal_exti_gpio_rising_trigger_conf(pGPIO->PinCfg.pinNumber, HAL_DISABLE);
    } else if (pGPIO->PinCfg.pinInIntTrig == GPIO_IN_INT_TRIG_RISING) {
    	hal_exti_gpio_rising_trigger_conf(pGPIO->PinCfg.pinNumber, HAL_ENABLE);
    	hal_exti_gpio_falling_trigger_conf(pGPIO->PinCfg.pinNumber, HAL_DISABLE);
    } else if (pGPIO->PinCfg.pinInIntTrig == GPIO_IN_INT_TRIG_RISING_FALLING) {
    	hal_exti_gpio_falling_trigger_conf(pGPIO->PinCfg.pinNumber, HAL_ENABLE);
    	hal_exti_gpio_rising_trigger_conf(pGPIO->PinCfg.pinNumber, HAL_ENABLE);
    }

    // 2. Configure GPIO port selection using SYSCFG EXTICR */
    hal_rcc_syscfg_pclk_conf(HAL_ENABLE);
    hal_syscfg_exti_gpio_int_conf(pGPIO->PinCfg.portNumber, pGPIO->PinCfg.pinNumber, HAL_ENABLE);

    // 3. Enable EXTI interrupt delivery with IMR:
    hal_exti_int_line_conf(pGPIO->PinCfg.pinNumber, HAL_ENABLE);
}

/**
 * @brief      Init GPIO X
 * @param[in]  Gpio config structure
 * @param[out] GPIO X handling structure
 * */
void hal_gpio_init(const GPIO_PinCfg_t *pGPIO_conf, GPIO_Handle_t *pOut)
{
	validate_pin_cfg(pGPIO_conf);

	pOut->pGPIO_Base = gpio_port_to_address[pGPIO_conf->portNumber];
	pOut->PinCfg = *pGPIO_conf;

	GPIO_RegDef_t *const pGPIO = pOut->pGPIO_Base;

	// 0. Enable GPIO clock:
	hal_gpio_perip_clk_ctrl(pGPIO_conf->portNumber, HAL_ENABLE);

	// 1. configure the mode of GPIO pin: push-pull / open drain : 2 bits
	pGPIO->MODER &= ~(0x3U << pOut->PinCfg.pinNumber * 2);
	pGPIO->MODER |= (pOut->PinCfg.pinMode << pOut->PinCfg.pinNumber * 2);

	// 1.1 COnfigure pin interrupt mode:
	if (pOut->PinCfg.pinInIntTrig != GPIO_IN_INT_TRIG_DISABLED)
		hal_gpio_enable_interrupt(pOut);

	// 2. configure the speed : 2 bits
	pGPIO->OSPEEDR &= ~(0x3U << pOut->PinCfg.pinNumber * 2);
	pGPIO->OSPEEDR |= (pOut->PinCfg.pinSpeed << pOut->PinCfg.pinNumber * 2);

	// 3. configure pull-up/down resistors : 2 bits
	pGPIO->PUPDR &= ~(0x3U << pOut->PinCfg.pinNumber * 2);
	pGPIO->PUPDR |= (pOut->PinCfg.pinPuPdControl << pOut->PinCfg.pinNumber * 2);

	// 4. configure output type: 1 bit
	if (pOut->PinCfg.pinMode == GPIO_OUTPUT) {
		pGPIO->OTYPER &= ~(0x1U << pOut->PinCfg.pinNumber * 1);
	    pGPIO->OTYPER |= (pOut->PinCfg.pinOpType << pOut->PinCfg.pinNumber * 1);
	}
	// 5. configure alternative function: 4 bits
	if (pOut->PinCfg.pinMode == GPIO_ALT_FUNC) {
		uint8_t reg_index = pOut->PinCfg.pinNumber / 8; // FIXME ? SHould be uint8 or uint32? what's more efficient
		uint8_t reg_offset = pOut->PinCfg.pinNumber % 8;

		pGPIO->AFR[reg_index] &= ~(0xFU << reg_offset * 4);
	    pGPIO->AFR[reg_index] |= (pOut->PinCfg.pinAFmode<< reg_offset * 4);
	}
}

/**
 * @brief     Put all GPIO X registers to default reset state. Uses RCC reset register.
 * @param[in] Base address of the GPIO peripheral
 * */
void hal_gpio_deinit(gpio_port_num_t gpio)
{
	hal_rcc_gpio_peri_reset(gpio);
}

/* Input/ Output */
/**
 * @brief     Read value from GPIO pin
 * @param[in] Base address of the GPIO peripheral
 * @param[in] GPIO pin number
 * @return    pin value
 * */
uint8_t hal_gpio_read_input_pin(GPIO_Handle_t *pGPIO)
{
	uint8_t val = 0;
	val = (uint8_t)((pGPIO->pGPIO_Base->IDR >> pGPIO->PinCfg.pinNumber) & 0x1U);
	return val;
}

/**
 * @brief     Read value from all GPIO pins
 * @param[in] Base address of the GPIO peripheral
 * @return    pins values grouped in 16-bit
 * */
uint16_t hal_gpio_read_input_port(GPIO_RegDef_t *pGPIOx)
{
	return (uint16_t)pGPIOx->IDR;
}

/**
 * @brief     Write value to GPIO pin
 * @param[in] Base address of the GPIO peripheral
 * @param[in] GPIO pin number
 * @param[in] pin value
 * */
void hal_gpio_write_output_pin(GPIO_RegDef_t *pGPIOx, gpio_pin_t pin, gpio_pin_val_t val)
{
	if (val == GPIO_PIN_RESET)
		// Set 0 to bit position
		pGPIOx->ODR &= ~(1 << pin);
	else
		// Set 1 to bit position
		pGPIOx->ODR |= (1 << pin);
}

/**
 * @brief     Write value to all 16 GPIO pins
 * @param[in] Base address of the GPIO peripheral
 * @param[in] pin values (gpio_pin_val_t) grouped in 16-bit
 * @param[in] pin value
 * */
void hal_gpio_write_output_port(GPIO_RegDef_t *pGPIOx, uint16_t val)
{
	pGPIOx->ODR = val;
}

void hal_gpio_toggle_output_pin(const GPIO_Handle_t *pGPIO)
{
	pGPIO->pGPIO_Base->ODR ^= (1 << pGPIO->PinCfg.pinNumber);
}

void hal_gpio_IRQ_config(const GPIO_Handle_t *pGPIO, uint8_t irq_priority, uint8_t en_di)
{
	nvic_interrupt_number_t irq_num = hal_nvic_exti_line_to_irq(pGPIO->PinCfg.pinNumber);
	hal_nvic_irq_exti_enable_disable(irq_num, en_di);
	hal_nvic_irq_priority_config(irq_num, irq_priority);
}

void hal_gpio_IRQ_handle(uint8_t pin)
{
	// Clear EXTI PR (pending register bit
	hal_exti_interrupt_clear(pin);
}
