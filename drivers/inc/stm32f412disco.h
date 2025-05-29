/*
 * stm32f412disco.h
 *
 *  Created on: Apr 4, 2025
 *      Author: konstantin
 */

#ifndef INC_STM32F412DISCO_H_
#define INC_STM32F412DISCO_H_

#include <stdint.h>
#include "arm_cortex_m4.h"

//#define HAL_ENABLE 1
//#define HAL_DISABLE 0



#define HAL_SET (HAL_ENABLE)
#define HAL_RESET (HAL_DISABLE)
#define HAL_GPIO_PIN_SET (HAL_SET)
#define HAL_GPIO_PIN_RESET (HAL_RESET)

/* Memory base addrs */
#define MEM_FLASH (0x08000000U)
#define MEM_SRAM  (0x20000000U)
#define MEM_SYSBOOT (0x1FFF0000U)

/* BUS BASE ADDRS */

#define APB1_BASE (0x40000000U)
#define APB2_BASE (0x40010000U)
#define AHB1_BASE (0x40020000U)
#define AHB2_BASE (0x50000000U)
#define AHB3_BASE (0x60000000U)
#define CORTEX_M4_INTERNAL_PERIPHERAL_BASE (0xE0000000U)

/* GPIOs BASE ADDRs */
#define GPIOX_OFFSET (0x400U)
#define GPIOA_BASE (AHB1_BASE)
#define GPIOB_BASE (AHB1_BASE + GPIOX_OFFSET)
#define GPIOC_BASE (AHB1_BASE + 2 * GPIOX_OFFSET)
#define GPIOD_BASE (AHB1_BASE + 3 * GPIOX_OFFSET)
#define GPIOE_BASE (AHB1_BASE + 4 * GPIOX_OFFSET)
#define GPIOF_BASE (AHB1_BASE + 5 * GPIOX_OFFSET)
#define GPIOG_BASE (AHB1_BASE + 6 * GPIOX_OFFSET)
#define GPIOH_BASE (AHB1_BASE + 7 * GPIOX_OFFSET)

/* USARTs BASE ADDRs */
#define USART1_BASE (APB2_BASE + 0x1000U)
#define USART2_BASE (APB1_BASE + 0x4400U)
#define USART3_BASE (APB1_BASE + 0x4800U)
#define USART6_BASE (APB2_BASE + 0x1400U)

/* I2Cs BASE ADDRs */
#define I2C1_BASE (APB1_BASE + 0x5400U)
#define I2C2_BASE (APB1_BASE + 0x5800U)
#define I2C3_BASE (APB1_BASE + 0x5C00U)

/* SPI BASE ADDRs */
#define SPI1_BASE (APB2_BASE + 0x13000)
#define SPI2_BASE (APB1_BASE + 0x3800)
#define SPI3_BASE (APB1_BASE + 0x3C00)
#define SPI4_BASE (APB2_BASE + 0x13400)
#define SPI5_BASE (APB2_BASE + 0x15000)


/* External interrupt controller */
#define EXTI_BASE (APB2_BASE + 0x3C00U)
/* ?? */
#define SYSCFG_BASE (APB2_BASE + 0x3800U)


typedef enum {
	I2C1n = 1,
	I2C2n = 2,
	I2C3n = 3
} I2C_num_t;

typedef enum {
	GPIOA_PORT = 0,
	GPIOB_PORT = 1,
	GPIOC_PORT = 2,
	GPIOD_PORT = 3,
	GPIOE_PORT = 4,
	GPIOH_PORT = 5,
	GPIOG_PORT = 6,
	GPIOF_PORT = 7,
	NUM_OF_GPIOS
} gpio_port_num_t;

#define GPIOA ((GPIO_RegDef_t *)GPIOA_BASE)
#define GPIOB ((GPIO_RegDef_t *)GPIOB_BASE)
#define GPIOC ((GPIO_RegDef_t *)GPIOC_BASE)
#define GPIOD ((GPIO_RegDef_t *)GPIOD_BASE)
#define GPIOE ((GPIO_RegDef_t *)GPIOE_BASE)
#define GPIOF ((GPIO_RegDef_t *)GPIOF_BASE)
#define GPIOG ((GPIO_RegDef_t *)GPIOG_BASE)
#define GPIOH ((GPIO_RegDef_t *)GPIOH_BASE)

/* Clock enable/disable macro */
#define GPIOA_PCLK_EN() (RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN() (RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN() (RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN() (RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN() (RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN() (RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN() (RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN() (RCC->AHB1ENR |= (1 << 7))







#define I2C1_PCLK_EN() (RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN() (RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN() (RCC->APB1ENR |= (1 << 23))

#define USART1_PCLK_EN() (RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_EN() (RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN() (RCC->APB1ENR |= (1 << 18))
#define USART6_PCLK_EN() (RCC->APB2ENR |= (1 << 5))

#define SPI1_PCLK_EN() (RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN() (RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN() (RCC->APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN() (RCC->APB2ENR |= (1 << 13))
#define SPI5_PCLK_EN() (RCC->APB2ENR |= (1 << 20))

#define SYSCFG_PCLK_EN() (RCC->APB2ENR |= (1 << 14))

/* Disable */
#define GPIOA_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 7))

#define I2C1_PCLK_DI() (RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI() (RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI() (RCC->APB1ENR &= ~(1 << 23))

#define USART1_PCLK_DI() (RCC->APB2ENR &= ~(1 << 4))
#define USART2_PCLK_DI() (RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DI() (RCC->APB1ENR &= ~(1 << 18))
#define USART6_PCLK_DI() (RCC->APB2ENR &= ~(1 << 5))

#define SPI1_PCLK_DI() (RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI() (RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI() (RCC->APB1ENR &= ~(1 << 15))
#define SPI4_PCLK_DI() (RCC->APB2ENR &= ~(1 << 13))
#define SPI5_PCLK_DI() (RCC->APB2ENR &= ~(1 << 20))

#define SYSCFG_PCLK_DI() (RCC->APB2ENR &= ~(1 << 14))

 /* =================================================== */

#endif /* INC_STM32F412DISCO_H_ */
