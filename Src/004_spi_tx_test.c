/*
 * 004_spi_tx_test.c
 *
 *  Created on: May 28, 2025
 *      Author: konstantin
 */

#include "004_spi_tx_test.h"
#include "stm32f412disco_gpio.h"
#include <stdio.h>
#include <string.h>

#define BUTTON_PIN (GPIO_PIN6)
#define BUTTON_PORT (GPIOC_PORT)
#define LED_PIN (GPIO_PIN7)

static GPIO_Handle_t button_gpio;
#define RX_NOT_READY 0
#define RX_READY 1
static volatile uint32_t receiver_ready = RX_NOT_READY;

static void delay(uint32_t ticks)
{
	for(int i = 0; i < ticks; i++);
}

static const GPIO_PinCfg_t button_gpio_conf = {
	.portNumber = BUTTON_PORT,
	.pinMode = GPIO_INPUT,
	.pinNumber = BUTTON_PIN,
	.pinOpType = GPIO_OUT_DEFAULT,
	.pinPuPdControl = GPIO_PIN_PU,
	.pinSpeed = GPIO_OUT_SP_HIGH,
	.pinAFmode = GPIO_AF_DEFAULT_VAL,
	.pinInIntTrig = GPIO_IN_INT_TRIG_RISING
};

/*
 * Interrupt is waiting gpio pc6 line to up, this is the signal from the receiver that it is ready.
 * When transaction is over receiver will put the line to low to process the message and execute the command if needed.
 */
static void gpio_init_slave_ready_interrupt(void)
{

	hal_gpio_init(&button_gpio_conf, &button_gpio);
	hal_gpio_IRQ_config(&button_gpio, 8, HAL_ENABLE);
}


void gpio_interrupt_004_handler(void)
{
	delay(5000);
	receiver_ready = RX_READY;
	hal_gpio_IRQ_handle(button_gpio.PinCfg.pinNumber);
}

static void gpio_pins_enable() {
	GPIO_PinCfg_t gpio_spi2_conf = {
		.portNumber = GPIOB_PORT,
		.pinMode = GPIO_ALT_FUNC,
		.pinNumber = GPIO_PIN9,
		.pinOpType = GPIO_OUT_PUSH_PULL,
		.pinPuPdControl = GPIO_PIN_NO_PU_PD,
		.pinSpeed = GPIO_OUT_SP_HIGH,
		.pinAFmode = 5,
		.pinInIntTrig = GPIO_IN_INT_TRIG_DISABLED
	};

	GPIO_Handle_t gpio_spi2_handle;
	/* NSS */
	hal_gpio_init(&gpio_spi2_conf, &gpio_spi2_handle);

	/* SCLK */
	gpio_spi2_conf.pinNumber = GPIO_PIN10;
	hal_gpio_init(&gpio_spi2_conf, &gpio_spi2_handle);

	/* MISO */
	gpio_spi2_conf.pinNumber = GPIO_PIN14; // 4
	hal_gpio_init(&gpio_spi2_conf, &gpio_spi2_handle);

	/* MOSI */
	gpio_spi2_conf.pinNumber = GPIO_PIN15; // 3
	hal_gpio_init(&gpio_spi2_conf, &gpio_spi2_handle);
}

typedef enum {
	STATE_BEGIN,
	SEND_CMD = STATE_BEGIN,
	RECEIVE_ACK,
	SEND_LEN,
	RECEIVE_ACK2,
	SEND_MSG,
	RECEIVE_ACK3,
	STATE_END = RECEIVE_ACK3
} send_state_t;

void spi_tx_test_004_main(void)
{
	printf("%s started\n", __func__);
	gpio_init_slave_ready_interrupt();

	const SPI_Cfg_t spi2_cfg = {
        .spi_device = SPI2,
        .device_mode = SPI_MASTER,
		.clk_speed_div = SPI_CLK_DIV_4,
		.bus_config = SPI_BUS_CONF_FULL_DUP,
		.CPHA = SPI_CPHA_FIRST_EDGE,
		.CPOL = SPI_CPOL_LOW,
		.SSM = SPI_SSM_HW
	};

	SPI_Handle_t spi2;
	gpio_pins_enable();
	hal_spi_init(&spi2_cfg, &spi2);

	const char *str_data = "Hello World!";
	uint8_t CMD_byte = 0xFF;
	uint8_t dummy_byte = 0x1;
	uint8_t ack_byte = 0;
	send_state_t state = SEND_CMD;
	uint8_t *pTXData = NULL;
	uint8_t tx_len = 0;
	uint8_t *pRXData = NULL;
	uint8_t rx_len = 0;

	uint8_t msg_len = strlen(str_data);

	while (1) {
		while (!receiver_ready) {
			printf("STM32: waiting for the receiver\n");
			delay(100000);
		}
		switch(state) {
		case SEND_CMD:
			pTXData = (uint8_t *)&CMD_byte;
			tx_len = 1;
			pRXData = &dummy_byte;
			rx_len = 1;

			break;
		case RECEIVE_ACK:
			pTXData = (uint8_t *)&dummy_byte;
			tx_len = 1;
			pRXData = &ack_byte;
			rx_len = 1;
			break;
		case SEND_LEN:
			pTXData = (uint8_t *)&msg_len;
			tx_len = 1;
			pRXData = &dummy_byte;
			rx_len = 1;

			break;
		case RECEIVE_ACK2:
			pTXData = (uint8_t *)&dummy_byte;
			tx_len = 1;
			pRXData = &ack_byte;
			rx_len = 1;
			break;
		case SEND_MSG:
			pTXData = (uint8_t *)str_data;
			tx_len = msg_len;
			pRXData = &dummy_byte;
			rx_len = 1;

			break;
		case RECEIVE_ACK3:
			pTXData = (uint8_t *)&dummy_byte;
			tx_len = 1;
			pRXData = &ack_byte;
			rx_len = 1;
			break;
		default:
			printf("UNKNOWN CMD!\n");
		}
		receiver_ready = RX_NOT_READY;
		printf("STM32[%d]: sending the data: %x, len = %u\n", state, pTXData[0], tx_len);
		//if (state == STATE_BEGIN)
			hal_spi_enable(&spi2, HAL_ENABLE);

		hal_spi_send_data(&spi2, pTXData, tx_len);
		hal_spi_receive_data(&spi2, pRXData, rx_len);

		//if (state == STATE_END)
		    hal_spi_enable(&spi2, HAL_DISABLE);

		printf("STM32[%d]: received: %x, len = %u\n", state, pRXData[0], rx_len);
		state++;
		if (state > STATE_END)
			state = STATE_BEGIN;
	}
}
