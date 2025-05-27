/*
 * arm_cortex_m4.h
 *
 *  Created on: May 26, 2025
 *      Author: konstantin
 */

#ifndef INC_ARM_CORTEX_ARM_CORTEX_M4_H_
#define INC_ARM_CORTEX_ARM_CORTEX_M4_H_
#include <stdint.h>
#define __vo volatile
typedef volatile uint32_t reg_t;
typedef enum {
	HAL_DISABLE = 0,
	HAL_ENABLE = 1
} hal_enable_disable_t;

#define ARM_NVIC_BASE (0xE000E100)

#define WORD_BITS (32)
#endif /* INC_ARM_CORTEX_ARM_CORTEX_M4_H_ */
