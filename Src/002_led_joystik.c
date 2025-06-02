/*
 * 002_led_joystik.c
 *
 *  Created on: May 25, 2025
 *      Author: konstantin
 */
#include <stdio.h>
#include "stm32f412disco_gpio.h"
#include "002_led_joystik.h"
#define BUTTON_PRESSED 1

static void delay(uint32_t ticks)
{
	for(int i = 0; i < ticks; i++);
}

static void led_joystick_002(void)
{
	printf("%s started\n", __func__);
	const GPIO_PinCfg_t led_1_gpio_conf = {
		.pinMode = GPIO_OUTPUT,
		.pinNumber = GPIO_PC7,
		.pinOpType = GPIO_OUT_PUSH_PULL,
		.pinPuPdControl = GPIO_PIN_NO_PU_PD,
		.pinSpeed = GPIO_OUT_SP_HIGH,
		.pinAFmode = GPIO_AF_DEFAULT_VAL,
		.pinInIntTrig = GPIO_IN_INT_TRIG_DISABLED
	};

	GPIO_Handle_t led_1_gpio;
	hal_gpio_init(&led_1_gpio_conf, &led_1_gpio);


	const GPIO_PinCfg_t joystick_gpio_conf = {
		.pinMode = GPIO_INPUT,
		.pinNumber = GPIO_PA0,
		.pinOpType = GPIO_OUT_DEFAULT,
		.pinPuPdControl = GPIO_PIN_NO_PU_PD,
		.pinSpeed = GPIO_OUT_SP_HIGH,
		.pinAFmode = GPIO_AF_DEFAULT_VAL,
		.pinInIntTrig = GPIO_IN_INT_TRIG_DISABLED
	};

	GPIO_Handle_t joystick_gpio;
	hal_gpio_init(&joystick_gpio_conf, &joystick_gpio);
	for (;;) {
		if (hal_gpio_read_input_pin(&joystick_gpio) == BUTTON_PRESSED)
		{
			delay(500000);
			hal_gpio_toggle_output_pin(&led_1_gpio);
		}


	}
}

static void led_ext_button_002(void)
{
	printf("%s started\n", __func__);
	const GPIO_PinCfg_t led_1_gpio_conf = {
		.pinMode = GPIO_OUTPUT,
		.pinNumber = GPIO_PC7,
		.pinOpType = GPIO_OUT_PUSH_PULL,
		.pinPuPdControl = GPIO_PIN_NO_PU_PD,
		.pinSpeed = GPIO_OUT_SP_HIGH,
		.pinAFmode = GPIO_AF_DEFAULT_VAL,
		.pinInIntTrig = GPIO_IN_INT_TRIG_DISABLED
	};

	GPIO_Handle_t led_1_gpio;
	hal_gpio_init(&led_1_gpio_conf, &led_1_gpio);


	const GPIO_PinCfg_t button_gpio_conf = {
		.pinMode = GPIO_INPUT,
		.pinNumber = GPIO_PC6,
		.pinOpType = GPIO_OUT_DEFAULT,
		.pinPuPdControl = GPIO_PIN_NO_PU_PD,
		.pinSpeed = GPIO_OUT_SP_HIGH,
		.pinAFmode = GPIO_AF_DEFAULT_VAL,
		.pinInIntTrig = GPIO_IN_INT_TRIG_DISABLED
	};

	GPIO_Handle_t button_gpio;
	hal_gpio_init(&button_gpio_conf, &button_gpio);
	for (;;) {
		if (hal_gpio_read_input_pin(&button_gpio) == BUTTON_PRESSED)
		{
			delay(500000);
			hal_gpio_toggle_output_pin(&led_1_gpio);
		}


	}
}


void led_joystick_002_main(void)
{
	//led_joystick_002();
	led_ext_button_002();
}
