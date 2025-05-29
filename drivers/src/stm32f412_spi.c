/*
 * stm32f412_spi.c
 *
 *  Created on: May 27, 2025
 *      Author: konstantin
 */


#include "assert.h"
#include <stddef.h>
#include "stm32f412_spi.h"
#include "stm32f412_rcc.h"

/* SPI CR1 (Control register 1) bit shifts */
typedef enum {
	SPI_CR1_CPHA = 0,
	SPI_CR1_CPOL = 1,
	SPI_CR1_MSTR = 2,
	SPI_CR1_BR = 3,
	SPI_CR1_SPE = 6,
	SPI_CR1_LSBFIRST = 7,
	SPI_CR1_SSI = 8,
	SPI_CR1_SSM = 9,
	SPI_CR1_RX_ONLY = 10,
	SPI_CR1_DFF = 11,
	SPI_CR1_CRCNEXT = 12,
	SPI_CR1_CRCEN = 13,
	SPI_CR1_BIDIOE = 14,
	SPI_CR1_BIDIMODE = 15
} spi_cr1_shifts_t;

/* SPI CR2 (Control register 2) bit shifts */
typedef enum {
	SPI_CR2_RXDMAEN = 0,
	SPI_CR2_TXDMAEN = 1,
	SPI_CR2_SSOE = 2,
	SPI_CR2_FRF = 4,
	SPI_CR2_ERRIE = 5,
	SPI_CR2_RXNEIE = 6,
	SPI_CR2_TXEIE = 7,
} spi_cr2_shifts_t;

typedef enum {
	SPI_RX_READY = (1 << SPI_SR_RXNE),
	SPI_TX_EMPTY = (1 << SPI_SR_TXE)
} spi_tx_rx_status_t;

#define IS_VALID_SPI_DEV(SPI_DEV) ((SPI_DEV >= SPI1) && (SPI_DEV <= SPI_TOTAL_NUM))
#define IS_VALID_SPI_MODE(MODE) ((MODE >= SPI_SLAVE) && (MODE < _SPI_MODES_NUM))
#define IS_VALID_SPI_BUS_CONFIG(BUS_CONF_VAL) ((BUS_CONF_VAL >= SPI_BUS_CONF_FULL_DUP) && (BUS_CONF_VAL < _SPI_BUS_CONF_NUM))
#define IS_VALID_SPI_CLK_DIV(SPI_CLK_DIV) ((SPI_CLK_DIV >= SPI_CLK_DIV_2) && (SPI_CLK_DIV < _SPI_CLK_DIV_NUM))
#define IS_VALID_SPI_DFF(SPI_DFF) ((SPI_DFF >= SPI_DFF_8_BIT) && (SPI_DFF < _SPI_DFF_NUM))
#define IS_VALID_SPI_CPOL(SPI_CPOL) ((SPI_CPOL >= SPI_CPOL_LOW) && (SPI_CPOL < _SPI_CPOL_NUM))
#define IS_VALID_SPI_CPHA(SPI_CPHA) ((SPI_CPHA >= SPI_CPHA_FIRST_EDGE) && (SPI_CPHA < _SPI_CPHA_NUM))
#define IS_VALID_SPI_SSM(SPI_SSM) ((SPI_SSM >= SPI_SSM_HW) && (SPI_SSM < _SPI_SSM_NUM))

static inline void validate_spi_cfg(const SPI_Cfg_t *pSPI_conf)
{
	assert(pSPI_conf != NULL);
    assert(IS_VALID_SPI_DEV(pSPI_conf->spi_device));
    assert(IS_VALID_SPI_MODE(pSPI_conf->device_mode));
    assert(IS_VALID_SPI_BUS_CONFIG(pSPI_conf->bus_config));
    assert(IS_VALID_SPI_CLK_DIV(pSPI_conf->clk_speed_div));
    assert(IS_VALID_SPI_DFF(pSPI_conf->DFF));
    assert(IS_VALID_SPI_CPOL(pSPI_conf->CPOL));
    assert(IS_VALID_SPI_CPHA(pSPI_conf->CPHA));
    assert(IS_VALID_SPI_SSM(pSPI_conf->SSM));
}

static inline SPI_I2S_RegDef_t *hal_spi_dev_to_addr(spi_dev_num_t spi_number)
{
	if (spi_number == SPI1)
		return (SPI_I2S_RegDef_t *)SPI1_BASE;
	else if (spi_number == SPI2)
		return (SPI_I2S_RegDef_t *)SPI2_BASE;
	else if (spi_number == SPI3)
		return (SPI_I2S_RegDef_t *)SPI3_BASE;
	else if (spi_number == SPI4)
		return (SPI_I2S_RegDef_t *)SPI4_BASE;
	else /* (spi_number == SPI5) */
		return (SPI_I2S_RegDef_t *)SPI5_BASE;
}

/**
 * @brief     Enable or disable clock
 * @param[in] SPI device from spi_dev_num_t
 * @param[in] Enable or disable
 * */
static inline void hal_spi_perip_clk_ctrl(spi_dev_num_t spi_number, hal_enable_disable_t val)
{
	if (val == HAL_ENABLE) {
		hal_rcc_spi_pclk_en(spi_number);
	} else {
		hal_rcc_spi_pclk_di(spi_number);
	}
}


/**
 * @brief      Init SPI X
 * @param[in]  SPI config structure
 * @param[out] SPI X handling structure
 * */
void hal_spi_init(const SPI_Cfg_t *pSPI_conf, SPI_Handle_t *pOut)
{
	validate_spi_cfg(pSPI_conf);

	pOut->pSPI_Base = hal_spi_dev_to_addr(pSPI_conf->spi_device);
	pOut->SPICfg = *pSPI_conf;

	// 0. Enable SPI clock:
	hal_spi_perip_clk_ctrl(pOut->SPICfg.spi_device, HAL_ENABLE);

	uint32_t cr1_reg = 0;
	// 1. configure the mode of SPI: master slave, bit 2
	cr1_reg |= pOut->SPICfg.device_mode << SPI_CR1_MSTR;

	/* 2. configure bus config:
	 * full duplex: BIDIMODE = 0 (default no action)
	 * half duplex BIDIMODE = 1
	 * simplex rx only: BIDIMODE = 0, RXONLY = 1
	 */
	if (pOut->SPICfg.bus_config == SPI_BUS_CONF_HALF_DUP) {
		cr1_reg |= (1u << SPI_CR1_BIDIMODE); /* BIDIMODE bit 15 */
	} else if (pOut->SPICfg.bus_config == SPI_BUS_CONF_SIMP_RX_ONLY) {
		cr1_reg |= (1u << 10); /* RXONLY bit 10 to 1, BIDIMODE bit 15 left 0 */
	}

	/* SPI speed: configure clock divider */
	cr1_reg |= pOut->SPICfg.clk_speed_div << SPI_CR1_BR; // TODO ADD Assert to check final clock != 0

	cr1_reg |= pOut->SPICfg.DFF << SPI_CR1_DFF;

	cr1_reg |= pOut->SPICfg.CPOL << SPI_CR1_CPOL;
	cr1_reg |= pOut->SPICfg.CPHA << SPI_CR1_CPHA;

	/* SW slave mode: */
	cr1_reg |= pOut->SPICfg.SSM << SPI_CR1_SSM; /* If 1 SSI bit will be used as NSS source, by default SSI is 0 -> grounded -> slave mode */

	/* In case SW slave management enabled and we are in the master mode, put NSS pin to high to avoid
	 * MODF error that indicates multiple masters present on the bus:
	 */
	if (pOut->SPICfg.device_mode == SPI_MASTER) {
		if (pOut->SPICfg.SSM != SPI_SSM_HW) {
			cr1_reg |= 1 << SPI_CR1_SSI;
		} else {
			pOut->pSPI_Base->CR2 |= 1 << SPI_CR2_SSOE; // automatically put NSS line to 0 when SPE is set
		}
	}

	pOut->pSPI_Base->CR1 = cr1_reg;
}

/**
 * @brief     Put all SPI X registers to default reset state. Uses RCC reset register.
 * @param[in] Base address of the SPI peripheral
 * */
void hal_spi_deinit(spi_dev_num_t spi_dev)
{
	hal_rcc_spi_peri_reset(spi_dev);
}

// TODO: not safe to poll. need timeout
static inline void wait_tx_empty(const SPI_I2S_RegDef_t *pSPI)
{
	while (!(pSPI->SR & SPI_TX_EMPTY));
}

static inline void wait_rx_ready(const SPI_I2S_RegDef_t *pSPI)
{
	while (!(pSPI->SR & SPI_RX_READY));
}

static inline hal_enable_disable_t is_spi_enabled(const SPI_I2S_RegDef_t *pSPI)
{
	return (hal_enable_disable_t) (pSPI->CR1 & (1 << SPI_CR1_SPE));
}

/* if (tx_len == 0) {
					val = *pTX_data << 8; // FIXME is this correct?
				} else {
					val = * ((uint16_t *)pTX_data);
					tx_len --;
					pTX_data ++;
				}
				*/
// TODO return an error code
void hal_spi_send_data(const SPI_Handle_t *pSPI_dev, const uint8_t *pTX_data, uint32_t tx_len)
{
	assert(is_spi_enabled(pSPI_dev->pSPI_Base));

	if (pTX_data != NULL && tx_len > 0) {
		while (tx_len > 0) {

			wait_tx_empty(pSPI_dev->pSPI_Base);

			uint16_t val = 0;
			if (pSPI_dev->SPICfg.DFF == SPI_DFF_8_BIT || tx_len == 1) {
				val = *pTX_data;
			} else {
				val = * ((uint16_t *)pTX_data);
				tx_len --;
				pTX_data ++;
			}
			tx_len --;
			pTX_data ++;
			pSPI_dev->pSPI_Base->DR = val;
		}
	}
}


void hal_spi_receive_data(const SPI_Handle_t *pSPI_dev, uint8_t *pRX_data, uint32_t rx_len)
{
	assert(is_spi_enabled(pSPI_dev->pSPI_Base));

	if (pRX_data != NULL && rx_len > 0) {
		while (rx_len > 0) {

			wait_rx_ready(pSPI_dev->pSPI_Base);

			if (pSPI_dev->SPICfg.DFF == SPI_DFF_8_BIT || rx_len == 1) {
				*pRX_data = (uint8_t)(pSPI_dev->pSPI_Base->DR & 0xFF);
			} else {
				*((uint16_t *)pRX_data) = ((uint16_t)(pSPI_dev->pSPI_Base->DR & 0xFFFF));
				rx_len --;
				pRX_data ++;
			}
			rx_len --;
			pRX_data ++;
		}
	}
}

void hal_spi_enable(const SPI_Handle_t *pSPI_dev, hal_enable_disable_t en_dis)
{
	if (en_dis == HAL_DISABLE) {
		// Should wait till SPI HW is not busy:
		while (hal_spi_is_busy(pSPI_dev->pSPI_Base) == SPI_BUSY);
	    pSPI_dev->pSPI_Base->CR1 &= ~(1 << SPI_CR1_SPE);
	}
	else {
		pSPI_dev->pSPI_Base->CR1 |= (1 << SPI_CR1_SPE);
	}
}
