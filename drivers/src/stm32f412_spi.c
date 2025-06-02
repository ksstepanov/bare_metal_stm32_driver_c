/*
 * stm32f412_spi.c
 *
 *  Created on: May 27, 2025
 *      Author: konstantin
 */


#include "assert.h"
#include <stddef.h>
#include <stdbool.h>
#include "stm32f412_spi.h"
#include "stm32f412_rcc.h"
#include "stm32f412_nvic_config.h"

static SPI_transaction_t current_transaction;

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
	SPI_TX_EMPTY = (1 << SPI_SR_TXE),
	SPI_ERR_CRC = (1 << SPI_SR_CRC_ERR),
	SPI_ERR_MODF = (1 << SPI_SR_MODF),
	SPI_ERR_OVERRUN = (1 << SPI_SR_OVR)
} spi_tx_rx_status_t;

/* ========================= static function declarations ============================================== */
static inline void validate_spi_cfg(const SPI_Cfg_t *pSPI_conf);
static inline SPI_I2S_RegDef_t *hal_spi_dev_to_addr(spi_dev_num_t spi_number);
static inline void hal_spi_perip_clk_ctrl(spi_dev_num_t spi_number, hal_enable_disable_t val);

static inline void wait_tx_empty(const SPI_I2S_RegDef_t *pSPI);
static inline void wait_rx_ready(const SPI_I2S_RegDef_t *pSPI);
static inline hal_enable_disable_t is_spi_enabled(const SPI_I2S_RegDef_t *pSPI);

/* ============== Interrupt-enabled checker functions: ================= */
static inline bool is_tx_int_enabled(const SPI_I2S_RegDef_t *pSPI);
static inline bool is_rx_int_enabled(const SPI_I2S_RegDef_t *pSPI);
static inline bool is_err_int_enabled(const SPI_I2S_RegDef_t *pSPI);

/* ============== SPI status register checker functions: ================= */
static inline bool is_tx_empty(const SPI_I2S_RegDef_t *pSPI);
static inline bool is_rx_ready(const SPI_I2S_RegDef_t *pSPI);
static inline bool is_spi_error(const SPI_I2S_RegDef_t *pSPI, spi_tx_rx_status_t err_code);

/* ============== SPI single byte TX/RX routines: ================= */
static inline void spi_tx(SPI_Handle_t *pSPI, SPI_transaction_t *pTransaction);
static inline void spi_rx(SPI_Handle_t *pSPI, SPI_transaction_t *pTransaction);

/* ============== Interrupt handling cases: ============== */
static inline void spi_int_txe_handle(SPI_Handle_t *pSPI);
static inline void spi_int_rxne_handle(SPI_Handle_t *pSPI);
static inline void spi_int_err_overrun_handle(SPI_Handle_t *pSPI);
static inline void spi_int_err_modf_handle(SPI_Handle_t *pSPI);
static inline void spi_int_err_crc_handle(SPI_Handle_t *pSPI);

/* ============== TX/RX transaction close: ================ */
static inline void spi_tx_close(SPI_Handle_t *pSPI);
static inline void spi_rx_close(SPI_Handle_t *pSPI);
static inline void spi_clear_overrun_flag(SPI_Handle_t *pSPI);

/* ============================== SPI exported API ==================================================== */

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
	pOut->data = &current_transaction;
	pOut->data->pRX_data = NULL;
	pOut->data->pTX_data = NULL;
	pOut->data->rx_len = 0;
	pOut->data->tx_len = 0;
	pOut->data->tx_state = SPI_READY;
	pOut->data->rx_state = SPI_READY;

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

void hal_spi_IRQ_config(SPI_Handle_t *pSPI_dev, uint8_t irq_priority, uint8_t en_di)
{
	nvic_interrupt_number_t irq_num = hal_nvic_spi_dev_num_to_irq(pSPI_dev->SPICfg.spi_device);
	hal_nvic_irq_enable_disable(irq_num, en_di);
	hal_nvic_irq_priority_config(irq_num, irq_priority);
}



/**
 * @brief Main SPI interrupt handler
 */
void hal_spi_IRQ_handle(SPI_Handle_t *pSPI)
{
	SPI_I2S_RegDef_t *pS = pSPI->pSPI_Base;
	if (is_tx_empty(pS) && is_tx_int_enabled(pS)) {
		/* Transmit next symbol */
		spi_int_txe_handle(pSPI);
	} else if (is_rx_ready(pS) && is_rx_int_enabled(pS)) {
		/* Receive next symbol */
		spi_int_rxne_handle(pSPI);
	} else if (is_err_int_enabled(pS)) {
		/* Error handling: */
		if (is_spi_error(pS, SPI_ERR_OVERRUN)) {
			spi_int_err_overrun_handle(pSPI);
		} else if (is_spi_error(pS, SPI_ERR_CRC)){
			spi_int_err_crc_handle(pSPI);
		} else if (is_spi_error(pS, SPI_ERR_MODF)){
			spi_int_err_modf_handle(pSPI);
		} else {
			// FIXME: escalate to UsageFault
		}
	}
}

// FIXME: SPI handle should be const */
// FIXME: make tread safe
spi_state_t hal_spi_send_data_with_IT(const SPI_Handle_t *pSPI_dev, SPI_transaction_t *t)
{
	assert(t->pTX_data != NULL);
	if (pSPI_dev->data->tx_state != SPI_BUSY_TX) { //== SPI_READY
		// 1. save the TX_data and len information to internal global var
		*pSPI_dev->data = *t;
		// 2. Mark the SPI state busy in transmission so that noone else (thread) can take over this peripheral
		pSPI_dev->data->tx_state = SPI_BUSY_TX;
		// 3. Enable the TXEIE bit to get interrupt whenever TXE bit is set
		pSPI_dev->pSPI_Base->CR2 |= (1 << SPI_CR2_TXEIE);
		// Data transmission will be handled by the ISR code
	}
	return pSPI_dev->data->tx_state;
}

spi_state_t hal_spi_receive_data_with_IT(const SPI_Handle_t *pSPI_dev, SPI_transaction_t *t)
{
	assert(t->pRX_data != NULL);
	if (pSPI_dev->data->rx_state != SPI_BUSY_RX) { //== SPI_READY
		// 1. save the TX_data and len information to internal global var
		*pSPI_dev->data = *t;
		// 2. Mark the SPI state busy in transmission so that noone else (thread) can take over this peripheral
		pSPI_dev->data->rx_state = SPI_BUSY_RX;
		// 3. Enable the RXEIE bit to get interrupt whenever TXE bit is set
		pSPI_dev->pSPI_Base->CR2 |= (1 << SPI_CR2_RXNEIE);
		// Data transmission will be handled by the ISR code
	}
	return pSPI_dev->data->rx_state;

}


void hal_spi_clear_overrun_flag(SPI_Handle_t *pSPI)
{
	spi_clear_overrun_flag(pSPI);
}

void hal_spi_close_transmission(SPI_Handle_t *pSPI)
{
	spi_tx_close(pSPI);
}

void hal_spi_close_reception(SPI_Handle_t *pSPI)
{
	spi_rx_close(pSPI);
}

__attribute((weak)) void spi_application_return_callback(SPI_Handle_t *pSPI, spi_application_event_t status)
{
	// Should be implemented by the application
}

/* ================================= Static functions definitions ============================ */
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

/* ============== Interrupt-enabled checker functions: ================= */
static inline bool is_tx_int_enabled(const SPI_I2S_RegDef_t *pSPI)
{
	return (pSPI->CR2 & (1 << SPI_CR2_TXEIE))  != 0;
}

static inline bool is_rx_int_enabled(const SPI_I2S_RegDef_t *pSPI)
{
	return (pSPI->CR2 & (1 << SPI_CR2_RXNEIE)) != 0;
}

static inline bool is_err_int_enabled(const SPI_I2S_RegDef_t *pSPI)
{
	return (pSPI->CR2 & (1 << SPI_CR2_ERRIE)) != 0;
}

/* ============== SPI status register checker functions: ================= */

static inline bool is_tx_empty(const SPI_I2S_RegDef_t *pSPI)
{
	return (pSPI->SR & SPI_TX_EMPTY) != 0;
}

static inline bool is_rx_ready(const SPI_I2S_RegDef_t *pSPI)
{
	return (pSPI->SR & SPI_RX_READY) != 0;
}

static inline bool is_spi_error(const SPI_I2S_RegDef_t *pSPI, spi_tx_rx_status_t err_code)
{
	return (pSPI->SR & err_code) != 0;
}

/* ============== Interrupt handling helper functions: ================= */

/**
 * @brief SPI transmission of a single frame (8/16 bits)
 */
static inline void spi_tx(SPI_Handle_t *pSPI, SPI_transaction_t *pTransaction)
{
	uint16_t val = 0;
	if (pSPI->SPICfg.DFF == SPI_DFF_8_BIT || pTransaction->tx_len == 1) {
		val = *(pTransaction->pTX_data);
	} else {
		val = * ((uint16_t *)(pTransaction->pTX_data));
		pTransaction->tx_len --;
		pTransaction->pTX_data ++;
	}
	pTransaction->tx_len --;
	pTransaction->pTX_data ++;
	pSPI->pSPI_Base->DR = val;
}

/**
 * @brief SPI reception of a single frame (8/16 bits)
 */
static inline void spi_rx(SPI_Handle_t *pSPI, SPI_transaction_t *pTransaction)
{
	if (pSPI->SPICfg.DFF == SPI_DFF_8_BIT || pTransaction->rx_len == 1) {
		*(pTransaction->pRX_data) = (uint8_t)(pSPI->pSPI_Base->DR & 0xFF);
	} else {
		*((uint16_t *)(pTransaction->pRX_data)) = ((uint16_t)(pSPI->pSPI_Base->DR & 0xFFFF));
		pTransaction->rx_len --;
		pTransaction->pRX_data ++;
	}
	pTransaction->rx_len --;
	pTransaction->pRX_data ++;
}

/**
 * @brief Handle of SPI TXEIE interrupt: transmission available
 */
static inline void spi_int_txe_handle(SPI_Handle_t *pSPI)
{
	spi_tx(pSPI, pSPI->data); // FIXME: bad parameter passing
	if (pSPI->data->tx_len == 0) {
		// Close the transaction and inform the application that TX is over.
		spi_tx_close(pSPI);
		spi_application_return_callback(pSPI, SPI_EVENT_TX_DONE); // FIXME review
	}
}

/**
 * @brief Handle of SPI RXNEIE interrupt: reception ready
 */
static inline void spi_int_rxne_handle(SPI_Handle_t *pSPI)
{
	spi_rx(pSPI, pSPI->data);
	if (pSPI->data->rx_len == 0) {
		// Close the transaction and inform the application that RX is over.
		spi_rx_close(pSPI);
		spi_application_return_callback(pSPI, SPI_EVENT_RX_DONE); // FIXME review
	}
}

static inline void spi_int_err_overrun_handle(SPI_Handle_t *pSPI)
{
	// clear the OVR flag: read DR and SR registers according to stm32f412 advanced reference manual 26.3.13
	if (pSPI->data->tx_state != SPI_BUSY_TX) {
		spi_clear_overrun_flag(pSPI);
	}
	// inform the application
	spi_application_return_callback(pSPI, SPI_EVENT_ERR_OVERRUN);
}

static inline void spi_int_err_modf_handle(SPI_Handle_t *pSPI)
{
	//FIXME
}

static inline void spi_int_err_crc_handle(SPI_Handle_t *pSPI)
{
	//FIXME
}

static inline void spi_tx_close(SPI_Handle_t *pSPI)
{
	// Clear TX interrupt bit:
	pSPI->pSPI_Base->CR2 &= ~(1 << SPI_CR2_TXEIE);
	/* clear pending interrupt in NVIC (this interrupt occures when SPI HW sends bytes and sets TXE bit in SR)
	   This can be skipped, but leads to additional SPI interrupt handler call.
	*/
	hal_nvic_irq_sw_control(hal_nvic_spi_dev_num_to_irq(pSPI->SPICfg.spi_device), HAL_DISABLE);
	// clear TX transaction data:
	pSPI->data->pTX_data = NULL;
	pSPI->data->tx_len = 0;
	pSPI->data->tx_state = SPI_READY;
}

static inline void spi_rx_close(SPI_Handle_t *pSPI)
{
	// Clear RX interrupt bit:
	pSPI->pSPI_Base->CR2 &= ~(1 << SPI_CR2_RXNEIE);
	// clear RX transaction data:
	pSPI->data->pRX_data = NULL;
	pSPI->data->rx_len = 0;
	pSPI->data->rx_state = SPI_READY;
}

static inline void spi_clear_overrun_flag(SPI_Handle_t *pSPI)
{
	reg_t tmp = pSPI->pSPI_Base->DR;
	tmp = pSPI->pSPI_Base->SR;
	(void)tmp; // TO avoid unused variable warning
}

