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
volatile uint32_t test_004_receiver_ready = RX_NOT_READY;
static uint8_t dummy_byte;

static void delay(uint32_t ticks)
{
	for(int i = 0; i < ticks; i++);
}

const GPIO_PinCfg_t test_004_button_gpio_conf = {
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
void test_004_gpio_init_slave_ready_interrupt(void)
{

	hal_gpio_init(&test_004_button_gpio_conf, &button_gpio);
	hal_gpio_IRQ_config(&button_gpio, 8, HAL_ENABLE);
}


void gpio_interrupt_004_handler(void)
{
	delay(5000);
	test_004_receiver_ready = RX_READY;
	hal_gpio_IRQ_handle(button_gpio.PinCfg.pinNumber);
}

void test_004_gpio_pins_enable(void) {
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

void test_004_transaction_init(SPI_transaction_t *pOut, test_004_send_state_t current_state, test_004_data_t *pIn) {
	if ((pOut == NULL) || (pOut->pTX_data == NULL) || (pOut->pRX_data == NULL)) {
		printf("ERROR: Null ptr provided.\n");
		current_state = STATE_ERROR;
	}
	switch(current_state) {
	case SEND_CMD:
		pOut->pTX_data = (uint8_t *)&(pIn->cmd);
		pOut->tx_len = sizeof(pIn->cmd);
		pOut->pRX_data = &dummy_byte; // FIXME
		pOut->rx_len = 1;

		break;
	case RECEIVE_ACK:
		pOut->pTX_data = (uint8_t *)&(dummy_byte);
		pOut->tx_len = 1;
		pOut->pRX_data = &(pIn->ack);
		pOut->rx_len = sizeof(pIn->ack);
		break;
	case SEND_LEN:
		pOut->pTX_data = (uint8_t *)&(pIn->msg_len_out);
		pOut->tx_len = sizeof(pIn->msg_len_out);
		pOut->pRX_data = &dummy_byte;
		pOut->rx_len = 1;
		break;
	case RECEIVE_ACK2:
		pOut->pTX_data = (uint8_t *)&dummy_byte;
		pOut->tx_len = 1;
		pOut->pRX_data = &(pIn->ack);
		pOut->rx_len = sizeof(pIn->ack);
		break;
	case SEND_MSG:
		pOut->pTX_data = (uint8_t *)(pIn->msg_out);
		pOut->tx_len = pIn->msg_len_out;
		pOut->pRX_data = &dummy_byte;
		pOut->rx_len = 1;

		break;
	case RECEIVE_ACK3:
		pOut->pTX_data = (uint8_t *)&dummy_byte;
		pOut->tx_len = 1;
		pOut->pRX_data = &(pIn->ack);
		pOut->rx_len = sizeof(pIn->ack);
		break;
	default:
		printf("ERROR: UNKNOWN CMD!\n");
		pOut->pTX_data = NULL;
		pOut->tx_len = 0;
		pOut->pRX_data = NULL;
		pOut->rx_len = 0;
	}
}

void spi_tx_test_004_main(void)
{
	printf("%s started\n", __func__);
	test_004_gpio_init_slave_ready_interrupt();

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
	test_004_gpio_pins_enable();
	hal_spi_init(&spi2_cfg, &spi2);
	hal_spi_IRQ_config(&spi2, 8, HAL_ENABLE); // FIXME: assert that IRQ is send_with_IT function if IRQ is not enabled

	const char *str_data = "Hello World!";
	char message_in[MAX_MSG_LEN_IN];

	test_004_data_t data = {
			.cmd = (uint8_t)MESSAGE_MASTER_TO_SLAVE,
			.ack = 0,
			.msg_out = str_data,
			.msg_len_out = strlen(str_data) + 1,
			.p_buf_in = message_in,
			.max_msg_len = MAX_MSG_LEN_IN
	};
	SPI_transaction_t t;
	test_004_send_state_t state = SEND_CMD;

	while (1) {
		while (!test_004_receiver_ready) {
			printf("STM32: waiting for the receiver\n");
			delay(100000); // FIXME to time
		}
		test_004_receiver_ready = RX_NOT_READY;
		test_004_transaction_init(&t, state, &data);

		//if (state == STATE_BEGIN) // FIXME
		hal_spi_enable(&spi2, HAL_ENABLE);

		if (t.tx_len > 0) {
			printf("STM32[%d]: sending the data: %x, len = %u\n", state, t.pTX_data[0], t.tx_len);
			hal_spi_send_data_with_polling(&spi2, &t);
		}
		if (t.rx_len > 0) {
			hal_spi_receive_data_with_polling(&spi2, &t);
			printf("STM32[%d]: received: %x, len = %u\n", state, t.pRX_data[0], t.rx_len);
		}
		//if (state == STATE_END)
		    hal_spi_enable(&spi2, HAL_DISABLE);

		state++;
		if (state > STATE_END)
			state = STATE_BEGIN;
	}
}
