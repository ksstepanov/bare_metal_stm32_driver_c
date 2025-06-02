/*
 * 005_spi_interrupts_test.c
 *
 *  Created on: May 30, 2025
 *      Author: konstantin
 */

#include "005_spi_tx_test.h"
#include "004_spi_tx_test.h"
#include "stm32f412disco_gpio.h"
#include <stdio.h>
#include <string.h>

/* Reuse test_004 globals to avoid redefinitions */
extern uint8_t dummy_byte;
extern volatile uint32_t test_004_receiver_ready;
extern const GPIO_PinCfg_t test_004_button_gpio_conf;

static SPI_Handle_t spi2;

static void delay(uint32_t ticks)
{
	for(int i = 0; i < ticks; i++);
}

void gpio_interrupt_005_handler(void)
{
	gpio_interrupt_004_handler();
}

void spi_interrupt_005_handler(void)
{
	delay(5000);
	hal_spi_IRQ_handle(&spi2);
}

void spi_tx_test_005_main(void)
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
		.SSM = SPI_SSM_HW,
		.pins = {
             .MOSI = GPIO_PB15,
         	 .MISO = GPIO_PB14,
         	 .SCLK = GPIO_PB10,
         	 .CS = GPIO_PB9,
		}
	};

	//SPI_Handle_t spi2;
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
			hal_spi_send_data_with_IT(&spi2, &t);
		}
		if (t.rx_len > 0) {
			hal_spi_receive_data_with_IT(&spi2, &t);
			printf("STM32[%d]: received: %x, len = %u\n", state, t.pRX_data[0], t.rx_len);
		}
		//if (state == STATE_END)
		    hal_spi_enable(&spi2, HAL_DISABLE);

		state++;
		if (state > STATE_END)
			state = STATE_BEGIN;
	}
}

void spi_application_return_callback(SPI_Handle_t *pSPI, spi_application_event_t status)
{
	printf("%s\n", hal_spi_dbg_status_to_string(status));
}
