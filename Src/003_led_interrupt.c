/*
 * 003_led_interrupt.c
 *
 *  Created on: May 26, 2025
 *      Author: konstantin
 */

#include <stdio.h>
#include "stm32f412disco_gpio.h"
#include "003_led_interrupt.h"
#define BUTTON_PRESSED 1

#define BUTTON_PIN (GPIO_PIN6)
#define LED_PIN (GPIO_PIN7)
#define BUTTON_PORT (GPIOC_PORT)
#define LED_PORT (GPIOC_PORT)


static void delay(uint32_t ticks)
{
	for(int i = 0; i < ticks; i++);
}

static GPIO_Handle_t led_1_gpio;
static GPIO_Handle_t button_gpio;

static void led_ext_button_003(void)
{
	printf("%s started\n", __func__);
	const GPIO_PinCfg_t led_1_gpio_conf = {
		.portNumber = LED_PORT,
		.pinMode = GPIO_OUTPUT,
		.pinNumber = LED_PIN,
		.pinOpType = GPIO_OUT_PUSH_PULL,
		.pinPuPdControl = GPIO_PIN_NO_PU_PD,
		.pinSpeed = GPIO_OUT_SP_HIGH,
		.pinAFmode = GPIO_AF_DEFAULT_VAL,
		.pinInIntTrig = GPIO_IN_INT_TRIG_DISABLED
	};

	hal_gpio_init(&led_1_gpio_conf, &led_1_gpio);


	const GPIO_PinCfg_t button_gpio_conf = {
		.portNumber = BUTTON_PORT,
		.pinMode = GPIO_INPUT,
		.pinNumber = BUTTON_PIN,
		.pinOpType = GPIO_OUT_DEFAULT,
		.pinPuPdControl = GPIO_PIN_PU,
		.pinSpeed = GPIO_OUT_SP_HIGH,
		.pinAFmode = GPIO_AF_DEFAULT_VAL,
		.pinInIntTrig = GPIO_IN_INT_TRIG_FALLING
	};

	hal_gpio_init(&button_gpio_conf, &button_gpio);
	hal_gpio_IRQ_config(&button_gpio, 8, HAL_ENABLE);

	for (;;) {
		/*if (hal_gpio_read_input_pin(&button_gpio) == BUTTON_PRESSED)
		{
			delay(500000);
			hal_gpio_toggle_output_pin(&led_1_gpio);
		}
		*/
	}
}

void led_interrupt_003_button_handler(void)
{
	delay(50000);
	hal_gpio_toggle_output_pin(&led_1_gpio);
	hal_gpio_IRQ_handle(button_gpio.PinCfg.pinNumber);
}

void led_interrupt_003_main(void)
{
	led_ext_button_003();
}

