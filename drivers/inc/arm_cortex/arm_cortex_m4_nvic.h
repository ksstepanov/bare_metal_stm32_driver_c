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


#endif /* INC_STM32F412_NVIC_H_ */
