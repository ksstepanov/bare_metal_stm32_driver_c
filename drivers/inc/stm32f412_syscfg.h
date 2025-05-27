/*
 * stm32f412_syscfg.h
 *
 *  Created on: May 26, 2025
 *      Author: konstantin
 */

#ifndef INC_STM32F412_SYSCFG_H_
#define INC_STM32F412_SYSCFG_H_
#include "stm32f412disco.h"
#include "stm32f412disco_gpio.h"
#include "assert.h"

/*
 * The system configuration controller is mainly used to remap the memory accessible in the
 * code area and manage the external interrupt line connection to the GPIOs.
 */

typedef struct {
	/* [0x00] memory remap register
	 * Bits 1:0 MEM_MODE: Memory mapping selection
	 * Set and cleared by software. This bit controls the memory internal mapping at
	 * address 0x0000 0000. After reset these bits take the value selected by the Boot
	 * pins.
	 * 00: Main Flash memory mapped at 0x0000 0000
	 * 01: System Flash memory mapped at 0x0000 0000
	 * 10: reserved
	 * 11: Embedded SRAM mapped at 0x0000 0000
	 */
	reg_t MEMRMP;

	/* [0x04] peripheral mode configuration register
	 * Bit 16 ADC1DC2:
	 * 0: No effect.
	 * 1: Refer to AN4073 on how to use this bit
	 */
	reg_t PMC;

	/* [0x08, 0x0C, 0x10, 0x14] EXTICR1-4 external interrupt configuration registers
	 * Bits 15:0 EXTIx[3:0]: EXTI x configuration (x = 0 to 3 for CR1, 4 to 7 for CR2, 8 to 11 for CR3, 12 to 15 for CR4)
	 * These bits are written by software to select the source input for the EXTIx
	 * external interrupt.
	 * 0000: PA[x] pin
	 * 0001: PB[x] pin
	 * 0010: PC[x] pin
	 * 0011: PD[x] pin
	 * 0100: PE[x] pin
	 * 0101: PF[x] pin
	 * 0110: PG[x] pin
	 * 0111: PH[x] pin (Reserved for EXTI3 and EXTI2 configurations)
	 */
	reg_t EXTICR[4];

	/* [0x18]
	 * FR[18:0] [22:21]: Falling trigger event configuration bit of line x
	 */
	reg_t reserved;

	/* [0x1C] configuration register 2
	 * Bit 2 PVDL: PVD lock
	 * This bit is set by software. It can be cleared only by a system reset. It enables and
	 * locks the PVD connection to TIM1/8 Break input. It also locks (write protection) the
	 * PVDE and PVDS[2:0] bits of PWR_CR register.
	 * 0: PVD interrupt not connected to TIM1/8 Break input. PVDE and PVDS[2:0] can be read and modified
	 * 1: PVD interrupt connected to TIM1/8 Break input. PVDE and PVDS[2:0] are read- nly
	 * Bit 1 Reserved, must be kept at reset value.
	 * Bit 0 CLL: core lockup lock
	 * This bit is set and cleared by software. It enables and locks the LOCKUP (Hardfault)
	 * output of the Cortex®-M4 with FPU core with TIM1/8 Break input.
	 * 0: Cortex®-M4 with FPU LOCKUP output not connected to TIM1/8 Break input
	 * 1: Cortex®-M4 with FPU LOCKUP output connected to TIM1/8 Break input
	 */
	reg_t CFGR2;

	/* [0x20] Compensation cell control register
	 * Bit 8 READY: Compensation cell ready flag
	 * 0: I/O compensation cell not ready
	 * 1: O compensation cell ready
	 * Bits 7:2 Reserved, must be kept at reset value.
	 * Bit 0 CMP_PD: Compensation cell power-down
	 * 0: I/O compensation cell power-down mode
	 * 1: I/O compensation cell enabled
	 */
	reg_t CMPCR;

	/* [0x24] configuration register
	 * Bit 1 I2CFMP1_SDA
	 * Set and cleared by software. When this bit is set, it forces FM+ drive capability on
	 * I2CFMP1_SDA pin selected through GPIO port mode register and GPIO alternate function selection bits.
	 * Bit 0 I2CFMP1_SCL
	 * Set and cleared by software. When this bit is set, it forces FM+ drive capability on
	 * I2CFMP1_SCL pin selected through GPIO port mode register and GPIO alternate function selection bits.
	 */
	reg_t CFGR;
} SYSCFG_RegDef_t;

#define SYSCFG ((SYSCFG_RegDef_t *)SYSCFG_BASE)

static inline void hal_syscfg_exti_gpio_int_conf(gpio_port_num_t gpio_port, gpio_pin_t pin, hal_enable_disable_t en_dis)
{
	assert(hal_rcc_syscfg_pclk_is_enabled());
	SYSCFG_RegDef_t *pSYSCFG = SYSCFG;
	uint8_t exticr_reg_offset = pin % 4;
	uint8_t exticr_reg = pin / 4;
	pSYSCFG->EXTICR[exticr_reg] &= ~(0xFU << exticr_reg_offset * 4);

	if (en_dis == HAL_ENABLE) {
		pSYSCFG->EXTICR[exticr_reg] |= gpio_port << exticr_reg_offset * 4;
	}
}


#endif /* INC_STM32F412_SYSCFG_H_ */
