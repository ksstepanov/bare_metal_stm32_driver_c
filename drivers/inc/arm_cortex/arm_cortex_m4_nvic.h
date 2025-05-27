/*
 * stm32f412_nvic.h
 *
 *  Created on: May 26, 2025
 *      Author: konstantin
 */

#ifndef INC_STM32F412_NVIC_H_
#define INC_STM32F412_NVIC_H_
#include "arm_cortex_m4.h"
#include <assert.h>

/*
 * The nested vector interrupt controller NVIC includes the following features:
 * 52 maskable interrupt channels (not including the 16 interrupt lines of Cortex®-M4 with FPU)
 * 16 programmable priority levels (4 bits of interrupt priority are used)
 * low-latency exception and interrupt handling
 * power management control
 * implementation of system control registers
 * The NVIC and the processor core interface are closely coupled, which enables low latency
 * interrupt processing and efficient processing of late arriving interrupts.
 * All interrupts including the core exceptions are managed by the NVIC. For more information
 * on exceptions and NVIC programming, refer to programming manual PM0214.
 */

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

typedef struct {
	/* [0xE000E100-0xE000E11C] Interrupt Set-enable Registers
	 * [31:0]SETENAInterrupt set-enable bits.
	 * Write:
	 * 0 = no effect
	 * 1 = enable interrupt.
	 * Read:
	 * 0 = interrupt disabled
	 * 1 = interrupt enabled.
	 */
	reg_t ISER[8];

	uint8_t reserved0[96];

	/* [0XE000E180-0xE000E19C] Interrupt Clear-enable Registers
	 * [31:0]CLRENAInterrupt clear-enable bits.
	 * Write:
	 * 0 = no effect
	 * 1 = disable interrupt.
	 * Read:
	 * 0 = interrupt disabled
	 * 1 = interrupt enabled.
	 */
	reg_t ICER[8];

	uint8_t reserved1[96];

	/* [0XE000E200-0xE000E21C] Interrupt Set-pending Registers
	 * [31:0]SETPENDInterrupt set-pending bits.
	 * Write:
	 * 0 = no effect
	 * 1 = changes interrupt state to pending.
	 * Read:
	 * 0 = interrupt is not pending
	 * 1 = interrupt is pending.
	 */
	reg_t ISPR[8];

	uint8_t reserved2[96];

	/* [0XE000E280-0xE000E29C] Interrupt Clear-pending Registers
	 * [31:0]CLRPENDInterrupt clear-pending bits.
	 * Write:
	 * 0 = no effect
	 * 1 = removes pending state an interrupt.
	 * Read:
	 * 0 = interrupt is not pending
	 * 1 = interrupt is pending.
	 */
	reg_t ICPR[8];

	uint8_t reserved3[96];

	/* [0xE000E300-0xE000E31C] Interrupt Active Bit Registers
	 * [31:0]ACTIVEInterrupt active flags, indicate:
	 * 0 = interrupt not active
	 * 1 = interrupt active.
	 */
	reg_t IABR[8];

	uint8_t reserved4[224];

	/* [0xE000E400-0xE000E4EF] Interrupt Priority Registers
	 * [31:24]Priority, byte offset 3
	 * [23:16]Priority, byte offset 2
	 * [15:8]Priority, byte offset 1
	 * Each implementation-defined priority field can hold a priority value, 0-255. The
	 * lower the value, the greater the priority of the corresponding interrupt. Register
	 * priority value fields are eight bits wide, and non-implemented low-order bits read as
	 * zero and ignore writes.
	 * [7:0]Priority, byte offset 0
	 */
	reg_t IPR[60];

	uint8_t reserved5[2576];

	/* [0xE000EF00] Software Trigger Interrupt Register
	 * [8:0]INTIDInterrupt ID of the interrupt to trigger, in
	 * the range 0-239. For example, a value of
	 * 0x03 specifies interrupt IRQ3.
	 */
	reg_t STIR;
} NVIC_RegDef_t;

#define NVIC ((NVIC_RegDef_t *)ARM_NVIC_BASE)

/* number of IRQs in single register */
#define ISR_PER_NON_PRIO_REG (WORD_BITS) /* for ALL NVIC registers except IPR, STIR */
/* Each reg: |8 bits IRQ3|8bits IRQ2|8bits IRQ1|8bits IRQ0| */
#define ISR_PER_PRIO_REG (4)             /* for NVIC IPR register */

#define IPR_BITS_PER_ISR (WORD_BITS / ISR_PER_PRIO_REG)
/* From 8 bits per IRQ 4 LSB are not implemented: ...|0000XXXX|..., where X indicate not implemented bit */
#define IPR_NOT_IMPLEM_BITS_PER_ISR (4)

#define IPR_REG_MAX_PRIO_VAL (0xF)


static inline nvic_interrupt_number_t hal_nvic_exti_line_to_irq(uint8_t exti_line)
{
	assert(exti_line <= 15);
	if (exti_line < 5) {
		return exti_line + NVIC_IRQ_EXTI0;
	} else if (exti_line >= 5 && exti_line < 10) {
		return NVIC_IRQ_EXTI9_5;
	} else { /*  exti_line >= 10 && exti_line <= 15: ok not to check because of the assert */
		return NVIC_IRQ_EXTI15_10;
	}
}

static inline void hal_nvic_irq_exti_enable_disable(nvic_interrupt_number_t irq_num, hal_enable_disable_t en_dis)
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

static inline void hal_nvic_irq_priority_config(nvic_interrupt_number_t irq_num, uint32_t priority)
{
	assert(irq_num >=NVIC_IRQ_WWDG && irq_num < _NVIC_IRQ_LABEL_TOTAL);
	//assert(priority); //FIXME

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

#endif /* INC_STM32F412_NVIC_H_ */
