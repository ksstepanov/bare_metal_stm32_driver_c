/*
 * stm32f412disco_gpio.h
 *
 *  Created on: Apr 5, 2025
 *      Author: konstantin
 */

#ifndef INC_STM32F412DISCO_GPIO_H_
#define INC_STM32F412DISCO_GPIO_H_
#include "stm32f412disco.h"
#include "stm32f412_gpio_pins.h"

typedef enum {
	GPIOA_PORT = 0,
	GPIOB_PORT = 1,
	GPIOC_PORT = 2,
	GPIOD_PORT = 3,
	GPIOE_PORT = 4,
	GPIOF_PORT = 5,
	GPIOG_PORT = 6,
	GPIOH_PORT = 7,
	GPIO_PORTS_NUM
} gpio_port_num_t;

typedef enum {
	GPIO_PIN_UNUSED = -1, // to skip any settings for pin index
	GPIO_PIN_IDX0 = 0,
	GPIO_PIN_IDX1 = 1,
	GPIO_PIN_IDX2 = 2,
	GPIO_PIN_IDX3 = 3,
	GPIO_PIN_IDX4 = 4,
	GPIO_PIN_IDX5 = 5,
	GPIO_PIN_IDX6 = 6,
	GPIO_PIN_IDX7 = 7,
	GPIO_PIN_IDX8 = 8,
	GPIO_PIN_IDX9 = 9,
	GPIO_PIN_IDX10 = 10,
	GPIO_PIN_IDX11 = 11,
	GPIO_PIN_IDX12 = 12,
	GPIO_PIN_IDX13 = 13,
	GPIO_PIN_IDX14 = 14,
	GPIO_PIN_IDX15 = 15,
	GPIO_PIN_IDXS_NUM
} gpio_pin_index_t;

typedef enum {
	GPIO_PIN_RESET = 0,
	GPIO_PIN_SET = 1
} gpio_pin_val_t;

/* non -interrupt modes */
typedef enum {
	GPIO_INPUT = 0,
	GPIO_OUTPUT = 1,
	GPIO_ALT_FUNC = 2,
	GPIO_ANALOG = 3
} gpio_mode_t;

/* interrupt modes */
typedef enum {
	GPIO_IN_INT_TRIG_DISABLED = 0,
	GPIO_IN_INT_TRIG_FALLING,
	GPIO_IN_INT_TRIG_RISING,
	GPIO_IN_INT_TRIG_RISING_FALLING,
} gpio_input_int_trigger_t;

typedef enum {
	GPIO_OUT_DEFAULT = 0,
	GPIO_OUT_PUSH_PULL = 0,
	GPIO_OUT_OPEN_DRAIN
} gpio_output_type_t;

typedef enum {
	GPIO_OUT_SP_LOW = 0,
	GPIO_OUT_SP_MEDIUM,
	GPIO_OUT_SP_HIGH,
	GPIO_OUT_SP_VHIGH,
} gpio_output_speed_t;

typedef enum {
	GPIO_PIN_NO_PU_PD = 0,
	GPIO_PIN_PU,
	GPIO_PIN_PD
} gpio_pu_pd_resistor_t;

typedef enum {
	GPIO_AF_DEFAULT_VAL = 0,
	GPIO_AF0 = 0,
	GPIO_AF1 = 1,
	GPIO_AF2 = 2,
	GPIO_AF3 = 3,
	GPIO_AF4 = 4,
	GPIO_AF5 = 5,
	GPIO_AF6 = 6,
	GPIO_AF7 = 7,
	GPIO_AF8 = 8,
	GPIO_AF9 = 9,
	GPIO_AF10 = 10,
	GPIO_AF12 = 12,
	GPIO_AF15 = 15
} gpio_af_mode_t;

typedef struct {
	/* [0x00]
	 *  These bits are written by software to configure the I/O direction mode.
		00: Input (reset state)
		01: General purpose output mode
		10: Alternate function mode
		11: Analog mode */
	__vo uint32_t MODER;
	/* [0x04]
	 *  OTy: Port x configuration bits (y = 0..15)
		These bits are written by software to configure the output type of the I/O port.
		0: Output push-pull (reset state)
		1: Output open-drain */
	__vo uint32_t OTYPER;
	/* [0x08]
	 *  These bits are written by software to configure the I/O output speed.
		00: Low speed
		01: Medium speed
		10: Fast speed
		11: High speed */
	__vo uint32_t OSPEEDR;
	/* [0x0C]
	 *  These bits are written by software to configure the I/O pull-up or pull-down:
	 * 	00: No pull-up, pull-down
		01: Pull-up
		10: Pull-down */
	__vo uint32_t PUPDR;
	/* [0x10]
	 * Input data: Bits 15:0 IDRy: Port input data (y = 0..15) */
	__vo uint32_t IDR;
	/* [0x14]
	 * Output data: Bits 15:0 ODRy: Port output data (y = 0..15) */
	__vo uint32_t ODR;
	/* [0x18] GPIO port bit set/reset register
	 * Bits 31:16 BRy: Port x reset bit y (y = 0..15)
	 * Bits 15:0 BSy: Port x set bit y (y= 0..15) */
	__vo uint32_t BSRR;
	/* [0x1c] Lock register (?) */
	__vo uint32_t LCKR;
	/* [0x20-0x24] AF0 - AF16 4 bit configuration fields, 16 * 4 = 64 = 2 * 32-bit registers
	 * 			  	AFR[0] - GPIO AF low Register for pins 0 - 7
	 * 			  	AFR[1] - GPIO AF high Register for pins 8 - 15 */
	__vo uint32_t AFR[2];
} GPIO_RegDef_t;

typedef struct {
	//uint8_t portNumber;      /* from gpio_num_t */
	uint8_t pinNumber;       /* from gpio_pin_t */
	uint8_t pinMode;         /* from gpio_mode_t */
	uint8_t pinSpeed;        /* from gpio_output_speed_t */
	uint8_t pinPuPdControl;  /* from gpio_pu_pd_resistor_t */
	uint8_t pinOpType;       /* from gpio_output_type_t */
	uint8_t pinAFmode;       /* from ? */
	uint8_t pinInIntTrig;    /* GPIO input interrupt trigger from gpio_input_int_trigger_t */
} GPIO_PinCfg_t;

typedef struct {
	/* Base addr of GPIO peripheral */
	GPIO_RegDef_t *pGPIO_Base;
	/* GPIO pin configuration settings */
	GPIO_PinCfg_t PinCfg;
} GPIO_Handle_t;


/**
 * @brief     Enable or disable clock
 * @param[in] GPIO port from gpio_port_num_t
 * @param[in] Enable or disable
 * */
//void hal_gpio_perip_clk_ctrl(gpio_port_num_t port_number, hal_enable_disable_t val);

/**
 * @brief      Init GPIO X
 * @param[in]  Gpio config structure
 * @param[out] GPIO X handling structure
 * */
void hal_gpio_init(const GPIO_PinCfg_t *pGPIO_conf, GPIO_Handle_t *pOut);

/**
 * @brief     Put all GPIO X registers to default reset state. Uses RCC reset register.
 * @param[in] Base address of the GPIO peripheral
 * */
void hal_gpio_deinit(gpio_port_num_t gpio);

/* Input/ Output */
/**
 * @brief     Read value from GPIO pin
 * @param[in] Base address of the GPIO peripheral
 * @param[in] GPIO pin number
 * @return    pin value
 * */
uint8_t hal_gpio_read_input_pin(const GPIO_Handle_t *pGPIO);

/**
 * @brief     Read value from all GPIO pins
 * @param[in] Base address of the GPIO peripheral
 * @return    pins values grouped in 16-bit
 * */
uint16_t hal_gpio_read_input_port(GPIO_RegDef_t *pGPIOx);

/**
 * @brief     Write value to GPIO pin
 * @param[in] Base address of the GPIO peripheral
 * @param[in] GPIO pin number
 * @param[in] pin value
 * */
void hal_gpio_write_output_pin(const GPIO_Handle_t *pGPIO, gpio_pin_val_t val);

/**
 * @brief     Write value to all 16 GPIO pins
 * @param[in] Base address of the GPIO peripheral
 * @param[in] pin values (gpio_pin_val_t) grouped in 16-bit
 * @param[in] pin value
 * */
void hal_gpio_write_output_port(GPIO_RegDef_t *pGPIOx, uint16_t val);


void hal_gpio_toggle_output_pin(const GPIO_Handle_t *pGPIO);

/* Interrupts */
void hal_gpio_IRQ_config(const GPIO_Handle_t *pGPIO, uint8_t irq_priority, uint8_t en_di);


void hal_gpio_IRQ_handle(const GPIO_Handle_t *pGPIO);

static inline gpio_port_num_t hal_gpio_pin_to_port(gpio_pin_t pin)
{
	return pin / GPIO_PIN_IDXS_NUM;
}

static inline gpio_pin_index_t hal_gpio_pin_to_pin_index_within_port(gpio_pin_t pin)
{
	return pin % GPIO_PIN_IDXS_NUM;
}

#endif /* INC_STM32F412DISCO_GPIO_H_ */
