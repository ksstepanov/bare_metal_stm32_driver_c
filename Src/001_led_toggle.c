/*
 * 001_led_toggle.c
 *
 *  Created on: May 25, 2025
 *      Author: konstantin
 */
#include "001_led_toggle.h"
#include "stm32f412disco_gpio.h"
#include <stdio.h>

typedef enum led_pins_ {
	LED_GREEN_PIN = GPIO_PC0,
	LED_ORANGE_PIN = GPIO_PC1,
	LED_RED_PIN = GPIO_PC2,
	LED_BLUE_PIN = GPIO_PC3
} led_pins_t;

static void delay(uint32_t ticks)
{
	for(int i = 0; i < ticks; i++);
}

static void led_toggle_001_push_pull(void)
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
	for (;;) {
		hal_gpio_toggle_output_pin(&led_1_gpio);
		delay(5000000);
	}
}

static void led_toggle_001_open_drain(void)
{
	printf("%s started\n", __func__);
	const GPIO_PinCfg_t led_1_gpio_conf = {
		.pinMode = GPIO_OUTPUT,
		.pinNumber = GPIO_PC7,
		.pinOpType = GPIO_OUT_OPEN_DRAIN,
		.pinPuPdControl = GPIO_PIN_PU,
		.pinSpeed = GPIO_OUT_SP_HIGH,
		.pinAFmode = GPIO_AF_DEFAULT_VAL,
		.pinInIntTrig = GPIO_IN_INT_TRIG_DISABLED
	};

	GPIO_Handle_t led_1_gpio;
	hal_gpio_init(&led_1_gpio_conf, &led_1_gpio);
	for (;;) {
		hal_gpio_toggle_output_pin(&led_1_gpio);
		delay(5000000);
	}
}

void led_toggle_001_main(void)
{
	//led_toggle_001_push_pull();
	led_toggle_001_open_drain();
}
