/*
 * stm32f412_spi.h
 *
 *  Created on: May 27, 2025
 *      Author: konstantin
 */

#ifndef INC_STM32F412_SPI_H_
#define INC_STM32F412_SPI_H_
#include "stm32f412disco.h"


typedef enum {
	SPI1 = 1,
	SPI2,
	SPI3,
	SPI4,
	SPI5,
	SPI_TOTAL_NUM = SPI5
} spi_dev_num_t;

typedef enum {
	SPI_SLAVE = 0,
	SPI_MASTER,
	_SPI_MODES_NUM
} spi_mode_t;

typedef enum {
	SPI_BUS_CONF_FULL_DUP = 0,
	SPI_BUS_CONF_HALF_DUP,
	SPI_BUS_CONF_SIMP_RX_ONLY,
	_SPI_BUS_CONF_NUM
} spi_bus_conf_t;

/* clock divider: defines final spi speed as BUS_CLOCK/divider */
typedef enum {
	SPI_CLK_DIV_2 = 0,
	SPI_CLK_DIV_4 = 1,
	SPI_CLK_DIV_8 = 2,
	SPI_CLK_DIV_16 = 3,
	SPI_CLK_DIV_32 = 4,
	SPI_CLK_DIV_64 = 5,
	SPI_CLK_DIV_128 = 6,
	SPI_CLK_DIV_256 = 7,
	_SPI_CLK_DIV_NUM
} spi_clock_speed_divisor_t;

/* Data frame format */
typedef enum {
	SPI_DFF_8_BIT = 0,
	SPI_DFF_16_BIT,
	_SPI_DFF_NUM
} spi_dff_t;

/* Clock polarity: controls idle state: 0 - low, 1 - high. */
typedef enum {
	SPI_CPOL_LOW = 0,
	SPI_CPOL_HIGH,
	_SPI_CPOL_NUM
} spi_cpol_t;

/* Clock phase: controls on which clock edge (1-st or 2nd) the data should be sampled.
 * Default mode CPOL 0, CPHA = 0 means data captured on the first edge which is rising.
 * This means first data sample appears half period before first clock, so that first riding edge of clock
 * correspond to middle of the data signal.
 */
typedef enum {
	SPI_CPHA_FIRST_EDGE = 0,
	SPI_CPHA_SECOND_EDGE,
	_SPI_CPHA_NUM
} spi_cpha_t;

/* software slave management (SSM): */
typedef enum {
	SPI_SSM_HW = 0, /* use HW - SSM disabled */
	SPI_SSM_SW,
	_SPI_SSM_NUM
} spi_ssm_t;

typedef struct {
	/* [0x00] SPI control register 1
	 *  Bit 15 BIDIMODE: Bidirectional data mode enable
	 *	Bit 14 BIDIOE: Output enable in bidirectional mode
	 *	Bit 13 CRCEN: Hardware CRC calculation enable
	 *	Bit 12 CRCNEXT: CRC transfer next
 	 *	Bit 11 DFF: Data frame format
	 *	Bit 10 RXONLY: Receive only mode enable
	 *	Bit 9 SSM: Software slave management
	 *	Bit 8 SSI: Internal slave select
	 *	Bit 7 LSBFIRST: Frame format
	 *	Bit 6 SPE: SPI enable
	 *	Bits 5:3 BR[2:0]: Baud rate control
	 *	Bit 2 MSTR: Master selection
	 *	Bit 1 CPOL: Clock polarity
	 *	Bit 0 CPHA: Clock phase
	 */
	reg_t CR1;

	/* [0x04] SPI control register 2
	 * Bit 7 TXEIE: Tx buffer empty interrupt enable
     * Bit 6 RXNEIE: RX buffer not empty interrupt enable
     * Bit 5 ERRIE: Error interrupt enable
     * Bit 4 FRF: Frame format
     * Bit 3 Reserved. Forced to 0 by hardware.
     * Bit 2 SSOE: SS output enable
     * Bit 1 TXDMAEN: Tx buffer DMA enable
     * Bit 0 RXDMAEN: Rx buffer DMA enable
     */
	reg_t CR2;

	/* [0x08] SPI status register
     * Bit 8 FRE: Frame Error
     * Bit 7 BSY: Busy flag
     * Bit 6 OVR: Overrun flag
     * Bit 5 MODF: Mode fault
     * Bit 4 CRCERR: CRC error flag
     * Bit 3 UDR: Underrun flag
     * Bit 2 CHSIDE: Channel side
     * Bit 1 TXE: Transmit buffer empty
     * Bit 0 RXNE: Receive buffer not empty
     */
	reg_t SR;

	/* [0x0C] SPI data register
     * DR[15:0]: Data register Data received or to be transmitted.
     */
	reg_t DR;

    /* [0x10] SPI CRC polynomial register
     * CRCPOLY[15:0]: CRC polynomial register
     */
	reg_t CRCPR;

    /* [0x14] SPI RX CRC register
     * Bits 15:0 RXCRC[15:0]: Rx CRC register. When CRC calculation is enabled, the RxCRC[15:0] bits contain the
     * computed CRC value of the subsequently received bytes.
     */
	reg_t RXCRCR;

    /* [0x18] SPI TX CRC register
     * Bits 15:0 TXCRC[15:0]: Tx CRC register. When CRC calculation is enabled, the TxCRC[7:0] bits contain the
     * computed CRC value of the subsequently transmitted bytes.
     */
	reg_t TXCRCR;

    /* [0x1c] SPI_I2S configuration register
     * Bit 12 ASTREN: Asynchronous start enable.
     * Bit 11 I2SMOD: I2S mode selection
     * Bit 10 I2SE: I2S Enable
     * Bits 9:8 I2SCFG: I2S configuration mode
     * Bit 7 PCMSYNC: PCM frame synchronization
     * Bits 5:4 I2SSTD: I2S standard selection
     * Bit 3 CKPOL: Steady state clock polarity
     * Bits 2:1 DATLEN: Data length to be transferred
     * Bit 0 CHLEN: Channel length (number of bits per audio channel)
     */
	reg_t I2SCFGR;

    /* [0x20] SPI_I2S prescaler register
     * Bit 9 MCKOE: Master clock output enable
     * Bit 8 ODD: Odd factor for the prescaler
     * Bits 7:0 I2SDIV: I2S Linear prescaler
     */
	reg_t I2SPR;
} SPI_I2S_RegDef_t;

/* SPI SR (SPI Status register) bit shifts */
typedef enum {
	SPI_SR_RXNE = 0,     /* Receive buffer not empty */
	SPI_SR_TXE = 1,      /* Transmit buffer empty */
	SPI_SR_CHSIDE = 2,   /* Channel side: not used in SPI */
	SPI_SR_UDR = 3,      /* Underrun flag: not used in SPI */
	SPI_SR_CRC_ERR = 4,  /* CRC error flag */
	SPI_SR_MODF = 5,     /* Mode fault: happens in multi-master mode and signals that one more master is active on the line */
	SPI_SR_OVR = 6,      /* An overrun condition occurs when the master or the slave completes the reception of the
                          * next data frame while the read operation of the previous frame from the Rx buffer has not
                          * completed (case RXNE flag is set). */
	SPI_SR_BSY = 7,      /* indicates that a data transfer is in progress on the SPI */
	SPI_SR_FRE = 8       /* Frame Error: used in SPI TI mode or in I2S mode, detects a change on NSS or WS line which takes place
                          *          in slave mode at a non expected time, informing about a desynchronization between the
                          *          external master device and the slave.
                          */
} spi_sr_shifts_t;

typedef struct {
	uint8_t spi_device;    /* from spi_dev_num_t */
	uint8_t device_mode;   /* from spi_mode_t */
	uint8_t bus_config;    /* from spi_bus_conf_t */
	uint8_t clk_speed_div; /* from spi_clock_speed_divisor_t */
	uint8_t DFF;           /* Data frame format from gpio_output_speed_t */
	uint8_t CPOL;          /* Clock polarity from gpio_pu_pd_resistor_t */
	uint8_t CPHA;          /* Clock phase from gpio_output_type_t */
	uint8_t SSM;           /* Software slave management from ? */
} SPI_Cfg_t; /* FIXME packed ? */


/* FIXME: questionable if this is needed ? */
typedef enum {
	SPI_READY = 0,
	SPI_BUSY_TX = 1,
	SPI_BUSY_RX = 2
} spi_state_t;

typedef enum {
	SPI_EVENT_TX_DONE = 0,
	SPI_EVENT_RX_DONE = 1,
	SPI_EVENT_ERR_CRC = 2,
	SPI_EVENT_ERR_MODF = 3,
	SPI_EVENT_ERR_OVERRUN = 4
} spi_application_event_t;



typedef struct {
	const uint8_t *pTX_data;
	uint8_t *pRX_data;
	uint16_t tx_len;
	uint16_t rx_len;
	uint8_t tx_state;
	uint8_t rx_state;
} SPI_transaction_t;

typedef struct {
	/* Base addr of SPI peripheral */
	SPI_I2S_RegDef_t *pSPI_Base;
	/* SPI configuration settings */
	SPI_Cfg_t SPICfg;

	/* current transaction data */
	SPI_transaction_t *data;
} SPI_Handle_t;

/**
 * @brief      Init SPI X
 * @param[in]  SPI config structure
 * @param[out] SPI X handling structure
 * */
void hal_spi_init(const SPI_Cfg_t *pGPIO_conf, SPI_Handle_t *pOut);

/**
 * @brief     Put all SPI X registers to default reset state. Uses RCC reset register.
 * @param[in] Base address of the SPI peripheral
 * */
void hal_spi_deinit(spi_dev_num_t gpio);

void hal_spi_send_data_with_polling(const SPI_Handle_t *pSPI_dev, SPI_transaction_t *pTransaction);
void hal_spi_receive_data_with_polling(const SPI_Handle_t *pSPI_dev, SPI_transaction_t *pTransaction);

spi_state_t hal_spi_send_data_with_IT(const SPI_Handle_t *pSPI_dev, SPI_transaction_t *t); // FIXME const
spi_state_t hal_spi_receive_data_with_IT(const SPI_Handle_t *pSPI_dev, SPI_transaction_t *t);

/* Interrupts */
void hal_spi_IRQ_config(SPI_Handle_t *pSPI, uint8_t irq_priority, uint8_t en_di);


void hal_spi_IRQ_handle(SPI_Handle_t *pSPI);

void hal_spi_enable(const SPI_Handle_t *pSPI_dev, hal_enable_disable_t en_dis);

typedef enum {
	SPI_NOT_BUSY = 0,
	SPI_BUSY = (1 << SPI_SR_BSY)
} spi_busy_t;

static inline spi_busy_t hal_spi_is_busy(const SPI_I2S_RegDef_t *pSPI)
{
	return (spi_busy_t) (pSPI->SR & (1 << SPI_SR_BSY));
}

static inline const char *hal_spi_dbg_status_to_string(spi_application_event_t status)
{
	if (status == SPI_EVENT_TX_DONE)
		return "TXDONE";
	else if (status == SPI_EVENT_RX_DONE)
		return "RXDONE";
	else if (status == SPI_EVENT_ERR_OVERRUN)
		return "ERR:OVERRUN";
	else if (status == SPI_EVENT_ERR_CRC)
		return "ERR:CRC";
	else if (status == SPI_EVENT_ERR_MODF)
		return "ERR:MODF";
	else
		return "ERROR STATUS UNKNOWN!";
}

void hal_spi_clear_overrun_flag(SPI_Handle_t *pSPI); // FIXME? SHould be exported?
void hal_spi_close_transmission(SPI_Handle_t *pSPI);
void hal_spi_close_reception(SPI_Handle_t *pSPI);

/* Optional callback */
void spi_application_return_callback(SPI_Handle_t *pSPI, spi_application_event_t status);

#endif /* INC_STM32F412_SPI_H_ */
