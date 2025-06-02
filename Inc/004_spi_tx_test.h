/*
 * 004_spi_tx_test.h
 *
 *  Created on: May 28, 2025
 *      Author: konstantin
 */

#ifndef _004_SPI_TX_TEST_H_
#define _004_SPI_TX_TEST_H_
#include "stm32f412_spi.h"

#define MAX_MSG_LEN_IN (512)

#define RX_NOT_READY 0
#define RX_READY 1

typedef enum {
	STATE_BEGIN,
	SEND_CMD = STATE_BEGIN,
	RECEIVE_ACK,
	SEND_LEN,
	RECEIVE_ACK2,
	SEND_MSG,
	RECEIVE_ACK3,
	STATE_END = RECEIVE_ACK3,
	STATE_ERROR
} test_004_send_state_t;

typedef enum {
	MESSAGE_MASTER_TO_SLAVE = 0xFF,
	MESSAGE_SLAVE_TO_MASTER = 0xFA,
	ACK_CMD = 0xA0,
	ACK_LEN = 0xA1,
	ACK_MSG_RECEIVED = 0xA2,
	ACK_MSG_PENDING_ON_SLAVE = 0xA3
} test_004_cmd_t;


typedef struct {
	uint8_t cmd; /* command byte */
	uint8_t ack; /* acknowledge byte */
	const char *msg_out;
	char *p_buf_in;
	uint16_t msg_len_out;
	uint16_t max_msg_len;
} test_004_data_t;

void test_004_transaction_init(SPI_transaction_t *pOut, test_004_send_state_t current_state, test_004_data_t *pIn);

void test_004_gpio_pins_enable(void);

void test_004_gpio_init_slave_ready_interrupt(void);

void gpio_interrupt_004_handler(void);

void spi_tx_test_004_main(void);

#endif /* _004_SPI_TX_TEST_H_ */
