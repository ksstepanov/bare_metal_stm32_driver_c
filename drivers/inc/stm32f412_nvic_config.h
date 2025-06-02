/*
 * stm32f412_nvic_config.h
 *
 *  Created on: May 30, 2025
 *      Author: konstantin
 */

#ifndef INC_STM32F412_NVIC_CONFIG_H_
#define INC_STM32F412_NVIC_CONFIG_H_
#include "arm_cortex_m4_nvic.h"
#include "stm32f412disco_gpio.h"
#include "stm32f412_spi.h"

/* System exceptions and IRQs */
typedef enum nvic_interrupt_number {
	_NVIC_BEGIN = 0,
	NVIC_SYS_EX_RESET = 1,
	NVIC_SYS_EX_NMI_Handler,
	NVIC_SYS_EX_HardFault_Handler,
	NVIC_SYS_EX_MemManage_Handler,
	NVIC_SYS_EX_BusFault_Handler,
	NVIC_SYS_EX_UsageFault_Handler,
	NVIC_SYS_EX_RESERVED0,
	NVIC_SYS_EX_RESERVED1,
	NVIC_SYS_EX_RESERVED2,
	NVIC_SYS_EX_RESERVED3,
	NVIC_SYS_EX_SVC_Handler,
	NVIC_SYS_EX_DebugMon_Handler,
	NVIC_SYS_EX_RESERVED4,
	NVIC_SYS_EX_PendSV_Handler,
	NVIC_SYS_EX_SysTick_Handler,
	_NVIC_SYS_EX_LABEL_TOTAL,

	/* 97 IRQs */
	_NVIC_IRQ_LABEL_START = -1,
	NVIC_IRQ_WWDG,
	NVIC_IRQ_PVD,
	NVIC_IRQ_TAMP_STAMP,
	NVIC_IRQ_RTC_WKUP,
	NVIC_IRQ_FLASH,
	NVIC_IRQ_RCC,
	NVIC_IRQ_EXTI0,
	NVIC_IRQ_EXTI1,
	NVIC_IRQ_EXTI2,
	NVIC_IRQ_EXTI3,
	NVIC_IRQ_EXTI4,
	NVIC_IRQ_DMA1_Stream0,
	NVIC_IRQ_DMA1_Stream1,
	NVIC_IRQ_DMA1_Stream2,
	NVIC_IRQ_DMA1_Stream3,
	NVIC_IRQ_DMA1_Stream4,
	NVIC_IRQ_DMA1_Stream5,
	NVIC_IRQ_DMA1_Stream6,
	NVIC_IRQ_ADC,
	NVIC_IRQ_CAN1_TX,
	NVIC_IRQ_CAN1_RX0,
	NVIC_IRQ_CAN1_RX1,
	NVIC_IRQ_CAN1_SCE,
	NVIC_IRQ_EXTI9_5,
	NVIC_IRQ_TIM1_BRK_TIM9,
	NVIC_IRQ_TIM1_UP_TIM10,
	NVIC_IRQ_TIM1_TRG_COM_TIM11,
	NVIC_IRQ_TIM1_CC,
	NVIC_IRQ_TIM2,
	NVIC_IRQ_TIM3,
	NVIC_IRQ_TIM4,
	NVIC_IRQ_I2C1_EV,
	NVIC_IRQ_I2C1_ER,
	NVIC_IRQ_I2C2_EV,
	NVIC_IRQ_I2C2_ER,
	NVIC_IRQ_SPI1,
	NVIC_IRQ_SPI2,
	NVIC_IRQ_USART1,
	NVIC_IRQ_USART2,
	NVIC_IRQ_USART3,
	NVIC_IRQ_EXTI15_10,
	NVIC_IRQ_RTC_Alarm,
	NVIC_IRQ_OTG_FS_WKUP,
	NVIC_IRQ_TIM12,
	NVIC_IRQ_TIM13,
	NVIC_IRQ_TIM14,
	NVIC_IRQ_TIM8_CC,
	NVIC_IRQ_DMA1_Stream7,
	NVIC_IRQ_FSMC,
	NVIC_IRQ_SDIO,
	NVIC_IRQ_TIM5,
	NVIC_IRQ_SPI3,
	NVIC_IRQ_RESERVED6,
	NVIC_IRQ_RESERVED7,
	NVIC_IRQ_TIM6_DACUNDER,
	NVIC_IRQ_TIM7_IRQHandler,
	NVIC_IRQ_DMA2_Stream0,
	NVIC_IRQ_DMA2_Stream1,
	NVIC_IRQ_DMA2_Stream2,
	NVIC_IRQ_DMA2_Stream3,
	NVIC_IRQ_DMA2_Stream4,
	NVIC_IRQ_DFSDM1_FLT0,
	NVIC_IRQ_DFSDM1_FLT1,
	NVIC_IRQ_CAN2_TX,
	NVIC_IRQ_CAN2_RX0,
	NVIC_IRQ_CAN2_RX1,
	NVIC_IRQ_CAN2_SCE,
	NVIC_IRQ_OTG_FS,
	NVIC_IRQ_DMA2_Stream5,
	NVIC_IRQ_DMA2_Stream6,
	NVIC_IRQ_DMA2_Stream7,
	NVIC_IRQ_USART6,
	NVIC_IRQ_I2C3_EV,
	NVIC_IRQ_I2C3_ER,
	NVIC_IRQ_RESERVED8,
	NVIC_IRQ_RESERVED9,
	NVIC_IRQ_RESERVED10,
	NVIC_IRQ_RESERVED11,
	NVIC_IRQ_RESERVED12,
	NVIC_IRQ_RESERVED13,
	NVIC_IRQ_HASH_RNG,
	NVIC_IRQ_FPU_Global,
	NVIC_IRQ_RESERVED14,
	NVIC_IRQ_RESERVED15,
	NVIC_IRQ_SPI4,
	NVIC_IRQ_SPI5,
	NVIC_IRQ_RESERVED16,
	NVIC_IRQ_RESERVED17,
	NVIC_IRQ_RESERVED18,
	NVIC_IRQ_RESERVED19,
	NVIC_IRQ_RESERVED20,
	NVIC_IRQ_RESERVED21,
	NVIC_IRQ_Quad_SP,
	NVIC_IRQ_RESERVED22,
	NVIC_IRQ_RESERVED23,
	NVIC_IRQ_I2CFMP1_event,
	NVIC_IRQ_I2CFMP1_error,
	_NVIC_IRQ_LABEL_TOTAL,
} nvic_interrupt_number_t;

/* Nor implemented NVIC: PR (priority register bits)
 * From 8 bits per IRQ 4 LSB are not implemented by STM: ...|0000XXXX|..., where X indicate not implemented bit
 */
#define IPR_NOT_IMPLEM_BITS_PER_ISR (4)
/* MAX NVIC PR (priority) register value (4 MSBs are 1): */
#define IPR_REG_MAX_PRIO_VAL (0xF)

/**
 * @brief: Enable or disable IRQ in NVIC. Based on Cortex-M4 Generic user guide
 */
static inline void hal_nvic_irq_enable_disable(nvic_interrupt_number_t irq_num, hal_enable_disable_t en_dis)
{
	assert(irq_num >=NVIC_IRQ_WWDG && irq_num < _NVIC_IRQ_LABEL_TOTAL);
	NVIC_RegDef_t *pNVIC = NVIC;
	uint8_t register_num = irq_num / ISR_PER_NON_PRIO_REG;
	uint8_t register_offset = irq_num % ISR_PER_NON_PRIO_REG;

	if (en_dis == HAL_DISABLE) {
		pNVIC->ICER[register_num] &= ~(1 << register_offset);
	} else {
	    pNVIC->ISER[register_num] |= (1 << register_offset);
	}
}

/*
 * @brief     set up SW interrupt directly in NVIC register or clear pending interrupt request
 * @param[in] irq_num IRQ number (not system exception! )
 * @param[in] en_dis enable (set) or disable (clear) the interrupt
 */
static inline void hal_nvic_irq_sw_control(nvic_interrupt_number_t irq_num, hal_enable_disable_t en_dis)
{
	assert(irq_num >=NVIC_IRQ_WWDG && irq_num < _NVIC_IRQ_LABEL_TOTAL);
	NVIC_RegDef_t *pNVIC = NVIC;
	uint8_t register_num = irq_num / ISR_PER_NON_PRIO_REG;
	uint8_t register_offset = irq_num % ISR_PER_NON_PRIO_REG;

	if (en_dis == HAL_DISABLE) {
		pNVIC->ICPR[register_num] |= ~(1 << register_offset); // Note! ICPR register bits are cleared by writing 1  not 0!
	} else {
	    pNVIC->ISPR[register_num] |= (1 << register_offset);
	}
}

/*
 * @brief     set up SW interrupt directly in NVIC register or clear pending interrupt request
 * @param[in] irq_num IRQ number (not system exception! )
 * @param[in] priority value in range [0 .. 0xF], where 0 is highest priority
 */
static inline void hal_nvic_irq_priority_config(nvic_interrupt_number_t irq_num, uint32_t priority)
{
	assert(irq_num >=NVIC_IRQ_WWDG && irq_num < _NVIC_IRQ_LABEL_TOTAL);
	assert(priority < IPR_REG_MAX_PRIO_VAL);

	NVIC_RegDef_t *pNVIC = NVIC;
	uint8_t register_num = irq_num / ISR_PER_PRIO_REG;
	uint8_t register_offset = irq_num % ISR_PER_PRIO_REG;
	/* Shift calculation example. Let priority = 4 bit value, where p0,p1,p2,p3 are its bits.
	 * Next is how it should be configured for IRQ3:
	 * (binary representation of IPR0 register:
	 * 31                  23       15       7        0
	 * |p3,p2,p1,p0,X,X,X,X|????XXXX|????XXXX|????XXXX|
	 */
	uint8_t shift = IPR_BITS_PER_ISR * register_offset + (IPR_BITS_PER_ISR - IPR_NOT_IMPLEM_BITS_PER_ISR);

	pNVIC->IPR[register_num] &= ~(IPR_REG_MAX_PRIO_VAL << shift);
	pNVIC->IPR[register_num] |= priority << shift;
}

/*
 * @brief     Map spi device from spi_dev_num_t to NVIC IRQ number
 * @param[in] spi_dev - SPI peripheral number
 */
static inline nvic_interrupt_number_t hal_nvic_spi_dev_num_to_irq(uint8_t spi_dev)
{
	assert(spi_dev <= SPI_TOTAL_NUM);

	if (spi_dev == SPI1)
		return NVIC_IRQ_SPI1;
	else if (spi_dev == SPI2)
		return NVIC_IRQ_SPI2;
	else if (spi_dev == SPI3)
		return NVIC_IRQ_SPI3;
	else if (spi_dev == SPI4)
		return NVIC_IRQ_SPI4;
	else /* (spi_dev == SPI5)*/
		return NVIC_IRQ_SPI5;
}

#endif /* INC_STM32F412_NVIC_CONFIG_H_ */
