/*
 * 005_spi_tx_test.h
 *
 *  Created on: May 28, 2025
 *      Author: konstantin
 */

#ifndef _005_SPI_TX_TEST_H_
#define _005_SPI_TX_TEST_H_
#include "stm32f412_spi.h"

void gpio_interrupt_005_handler(void);
void spi_interrupt_005_handler(void);

void spi_tx_test_005_main(void);

#endif /* _005_SPI_TX_TEST_H_ */
