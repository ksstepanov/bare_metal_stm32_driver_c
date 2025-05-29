/*
 * stm32f412_rcc.h
 *
 *  Created on: May 21, 2025
 *      Author: konstantin
 */

#ifndef INC_STM32F412_RCC_H_
#define INC_STM32F412_RCC_H_
#include "stm32f412disco.h"
#include "stm32f412_spi.h"
#include "assert.h"

/* Reset and clock control */
#define RCC_BASE (AHB1_BASE + 0x3800U)

/* Reset and Clock Control RCC registers */
typedef struct {
	/* [0x00] clock control register - clock params configure*/
	__vo uint32_t CR;
	/* [0x04] PLL configuration register */
	__vo uint32_t PLLCFGR;
	/* [0x08] RCC clock configuration register - select clock source */
	__vo uint32_t CFGR;
	/* [0x0C] RCC clock interrupt register */
	__vo uint32_t CIR;
	/* [0x10] RCC AHB1 peripheral reset register */
	__vo uint32_t AHB1RSTR;
	/* [0x14]  */
	__vo uint32_t AHB2RSTR;
	/* [0x18]  */
	__vo uint32_t AHB3RSTR;
	__vo uint32_t __RESERVED1;
	/* [0x20]  */
	__vo uint32_t APB1RSTR;
	/* [0x24]  */
	__vo uint32_t APB2RSTR;
	__vo uint32_t __RESERVED2[2];
	/* [0x30] RCC AHB1 peripheral clock enable register */
	__vo uint32_t AHB1ENR;
	/* [0x34]  */
	__vo uint32_t AHB2ENR;
	/* [0x38]  */
	__vo uint32_t AHB3ENR;
	__vo uint32_t __RESERVED4;
	/* [0x40]  */
	__vo uint32_t APB1ENR;
	/* [0x44]  */
	__vo uint32_t APB2ENR;
	__vo uint32_t __RESERVED5[2];
	/* [0x50] RCC AHB1 peripheral clock enable in low power mode register */
	__vo uint32_t AHB1LPENR;
	/* [0x54]  */
	__vo uint32_t AHB2LPENR;
	/* [0x58]  */
	__vo uint32_t AHB3LPENR;
	__vo uint32_t __RESERVED7;
	/* [0x60]  */
	__vo uint32_t APB1LPENR;
	/* [0x64]  */
	__vo uint32_t APB2LPENR;
	__vo uint32_t __RESERVED8[2];
	/* [0x70] RCC Backup domain control register */
	__vo uint32_t BDCR;
	/* [0x74] RCC clock control & status register */
	__vo uint32_t CSR;
	__vo uint32_t __RESERVED10[2];
	/* [0x80] RCC spread spectrum clock generation register */
	__vo uint32_t SSCGR;
	/* [0x84] RCC PLLI2S configuration register */
	__vo uint32_t PLLI2SCFGR;
	__vo uint32_t __RESERVED12;
	/* [0x8C] RCC Dedicated Clocks Configuration Register */
	__vo uint32_t DCKCFGR;
	/* [0x90] RCC clocks gated enable register */
	__vo uint32_t CKGATENR;
	/* [0x94] RCC Dedicated Clocks Configuration Register */
	__vo uint32_t DCKCFGR2;
} RCC_RegDef_t;

#define RCC ((RCC_RegDef_t *)RCC_BASE)

enum RCC_PCLK_APB2_EN_BIT {
	RCC_PCLK_TIM1 = 0,
	RCC_PCLK_TIM8 = 1,
	RCC_PCLK_USART1 = 4,
	RCC_PCLK_USART6 = 5,
	RCC_PCLK_ADC1 = 8,
	RCC_PCLK_SDIO = 11,
	RCC_PCLK_SPI1 = 12,
	RCC_PCLK_SPI4 = 13,
	RCC_PCLK_SYSCFG = 14,
	RCC_PCLK_TIM9 = 16,
	RCC_PCLK_TIM10 = 17,
	RCC_PCLK_TIM11 = 17,
	RCC_PCLK_SPI5 = 20,
	RCC_PCLK_DFSDM1 = 25
};

enum I2C_APB1ENR_BIT {
	I2C1_APB1ENR_BIT = 21,
	I2C2_APB1ENR_BIT = 22,
	I2C3_APB1ENR_BIT = 23
};

enum SPI_ENR_BIT {
	/* APB1 */
	SPI2_APB1_PCLK_EN_BIT = 14,
	SPI3_APB1_PCLK_EN_BIT = 15,
	/* APB2 */
	SPI1_APB2_PCLK_EN_BIT = 12,
	SPI4_APB2_PCLK_EN_BIT = 13,
	SPI5_APB2_PCLK_EN_BIT = 20,
};

enum SPI_RST_BIT {
	/* APB1 */
	SPI2_APB1_RST_BIT = 14,
	SPI3_APB1_RST_BIT = 15,
	/* APB2 */
	SPI1_APB2_RST_BIT = 12,
	SPI4_APB2_RST_BIT = 13,
	SPI5_APB2_RST_BIT = 20,
};

static inline void hal_i2c_rcc_pclk_en(I2C_num_t i2c)
{
	RCC_RegDef_t *const pRCC = RCC;
	if (i2c == I2C1n)
		pRCC->APB1ENR |= (1 << I2C1_APB1ENR_BIT);
	else if (i2c == I2C2n)
		pRCC->APB1ENR |= (1 << I2C2_APB1ENR_BIT);
	else if (i2c == I2C3n)
		pRCC->APB1ENR |= (1 << I2C3_APB1ENR_BIT);
}

static inline void hal_i2c_rcc_pclk_di(I2C_num_t i2c)
{
	RCC_RegDef_t *const pRCC = RCC;
	if (i2c == I2C1n)
		pRCC->APB1ENR &= ~(1 << I2C1_APB1ENR_BIT);
	else if (i2c == I2C2n)
		pRCC->APB1ENR &= ~(1 << I2C2_APB1ENR_BIT);
	else if (i2c == I2C3n)
		pRCC->APB1ENR &= ~(1 << I2C3_APB1ENR_BIT);
}

static inline void hal_rcc_gpio_pclk_en(gpio_port_num_t gpio)
{
	RCC_RegDef_t *const pRCC = RCC;
	pRCC->AHB1ENR |= (1 << gpio);
}

static inline void hal_rcc_gpio_pclk_di(gpio_port_num_t gpio)
{
	RCC_RegDef_t *const pRCC = RCC;
	pRCC->AHB1ENR &= ~(1 << gpio);
}

static inline void hal_rcc_spi_pclk_en(spi_dev_num_t spi_dev)
{
	RCC_RegDef_t *const pRCC = RCC;
	/* APB1 */
	if (spi_dev == SPI2)
		pRCC->APB1ENR |= (1 << SPI2_APB1_PCLK_EN_BIT);
	else if (spi_dev == SPI3)
		pRCC->APB1ENR |= (1 << SPI3_APB1_PCLK_EN_BIT);
	/* APB2 */
	else if (spi_dev == SPI1)
		pRCC->APB2ENR |= (1 << SPI1_APB2_PCLK_EN_BIT);
	else if (spi_dev == SPI4)
		pRCC->APB2ENR |= (1 << SPI1_APB2_PCLK_EN_BIT);
	else if (spi_dev == SPI5)
		pRCC->APB2ENR |= (1 << SPI1_APB2_PCLK_EN_BIT);
}

static inline void hal_rcc_spi_pclk_di(spi_dev_num_t spi_dev)
{
	RCC_RegDef_t *const pRCC = RCC;
	/* APB1 */
	if (spi_dev == SPI2)
		pRCC->APB1ENR &= ~(1 << SPI2_APB1_PCLK_EN_BIT);
	else if (spi_dev == SPI3)
		pRCC->APB1ENR &= ~(1 << SPI3_APB1_PCLK_EN_BIT);
	/* APB2 */
	else if (spi_dev == SPI1)
		pRCC->APB2ENR &= ~(1 << SPI1_APB2_PCLK_EN_BIT);
	else if (spi_dev == SPI4)
		pRCC->APB2ENR &= ~(1 << SPI1_APB2_PCLK_EN_BIT);
	else if (spi_dev == SPI5)
		pRCC->APB2ENR &= ~(1 << SPI1_APB2_PCLK_EN_BIT);
}

static inline void hal_rcc_gpio_peri_reset(gpio_port_num_t gpio)
{
	RCC_RegDef_t *const pRCC = RCC;
	// Set RCC peripheral reset register to 1 and then clear it back to 0 (not stay in reset state)
	pRCC->AHB1RSTR |= (1 << gpio);
	// Should wait here?
	pRCC->AHB1RSTR &= ~(1 << gpio);
}

static inline void hal_rcc_spi_peri_reset(spi_dev_num_t spi_dev)
{
	RCC_RegDef_t *const pRCC = RCC;
	/* APB1 */
	if (spi_dev == SPI2) {
		pRCC->APB1ENR |= (1 << SPI2_APB1_RST_BIT);
		pRCC->APB1ENR &= ~(1 << SPI2_APB1_RST_BIT);
	} else if (spi_dev == SPI3) {
		pRCC->APB1ENR |= (1 << SPI3_APB1_RST_BIT);
		pRCC->APB1ENR &= ~(1 << SPI3_APB1_RST_BIT);
	/* APB2 */
	} else if (spi_dev == SPI1) {
		pRCC->APB2ENR |= (1 << SPI1_APB2_RST_BIT);
		pRCC->APB2ENR &= ~(1 << SPI1_APB2_RST_BIT);
	} else if (spi_dev == SPI4) {
		pRCC->APB2ENR |= (1 << SPI4_APB2_RST_BIT);
		pRCC->APB2ENR &= ~(1 << SPI4_APB2_RST_BIT);
	} else if (spi_dev == SPI5) {
		pRCC->APB2ENR |= (1 << SPI5_APB2_RST_BIT);
		pRCC->APB2ENR &= ~(1 << SPI5_APB2_RST_BIT);
	}
}

static inline void hal_rcc_syscfg_pclk_conf(hal_enable_disable_t en_dis)
{
	RCC_RegDef_t *const pRCC = RCC;
	// Set RCC peripheral reset register to 1 and then clear it back to 0 (not stay in reset state)
	if (en_dis == HAL_DISABLE) {
		// Should wait here?
		pRCC->APB2ENR &= ~(1 << RCC_PCLK_SYSCFG);
	} else {
		pRCC->APB2ENR |= (1 << RCC_PCLK_SYSCFG);
	}
}

static inline uint8_t hal_rcc_syscfg_pclk_is_enabled(void)
{
	RCC_RegDef_t *const pRCC = RCC;
	uint8_t val = (pRCC->APB2ENR >> RCC_PCLK_SYSCFG) & 0x1U;
	return val;
}

enum USART_APBENR_BIT {
	USART1_APB2ENR_BIT = 4,
	USART2_APB1ENR_BIT = 17,
	USART3_APB1ENR_BIT = 18,
	USART6_APB2ENR_BIT = 5
};

#endif /* INC_STM32F412_RCC_H_ */
