/*
 * stm32f401xx_gpio.driver.h
 *
 *  Created on: Sep 21, 2020
 *      Author: jgonzalezlopes
 */

#ifndef DRIVERS_INC_STM32F401XX_GPIO_DRIVER_H_
#define DRIVERS_INC_STM32F401XX_GPIO_DRIVER_H_

#include "stm32f401xx.h"
#include "stm32f401xx_exti.driver.h"

#ifdef __cplusplus
 extern "C" {
#endif

/************************ PIN CONFIGURATION OF THE REGISTERS *********************************/
/*
 * @GPIO_PIN_MODES
 * GPIO MODE Possible values
 */
#define	GPIOx_MODE_INPUT				0x00UL
#define	GPIOx_MODE_OUTPUT				0x01UL
#define	GPIOx_MODE_ALTFN				0x02UL
#define	GPIOx_MODE_ANALOG				0x03UL
/// Custom modes!
#define GPIOx_MODE_IN_FALLEDGETRIG		0x04UL /*Failing edge trigger*/
#define GPIOx_MODE_IN_RAISEEDGETRIG		0x05UL /*Raising edge trigger*/
#define GPIOx_MODE_IN_FREDGETRIG		0x06UL	/*Failling/rasing edge trigger*/
/*
 * @GPIO_PIN_PTYPE
 * GPIO OTYPE possible values
 */
#define	GPIO_OTYPE_PUSHPULL		RESET
#define	GPIO_OTYPE_OPENDRAIN	SET
/*
 * @GPIO_PIN_SPEED
 * GPIO OSPEEDR possible values
 */
#define GPIO_OSPEEDR_LOW		0x0UL	/*Slow slew speed for the digital wave*/
#define GPIO_OSPEEDR_MEDIUM		0x1UL	/*Medium slew speed for the digital wave*/
#define GPIO_OSPEEDR_HIGH		0x2UL	/*High slew speed for the digital wave*/
#define GPIO_OSPEEDR_VHIGH		0x3UL	/*Very high slew speed for the digital wave*/
/*
 * GPIO PUPDR possible values
 * @GPIO_PIN_PUPDCTRL
 */
#define GPIO_PUPDR_NOPUPD		0x0UL	/*No pull-up/pull-down*/
#define GPIO_PUPDR_PU			0x1UL	/*Pull up resistor*/
#define GPIO_PUPDR_PD			0x2UL  	/*Pull down resistor*/
/*
 * @GPIO_PIN_AFR
 * GPIO Alternate function possible values (for both High and Low AFs)
 */
#define	GPIO_AFR_AF0			(0 << 0)											/*0b 0000*/
#define	GPIO_AFR_AF1			(1 << 0)											/*0b 0001*/
#define	GPIO_AFR_AF2			(1 << 1)											/*0b 0010*/
#define	GPIO_AFR_AF3			( GPIO_AFR_AF2 | GPIO_AFR_AF1 )						/*0b 0011*/
#define	GPIO_AFR_AF4			(1 << 2)											/*0b 0100*/
#define	GPIO_AFR_AF5			( GPIO_AFR_AF4 | GPIO_AFR_AF1 )						/*0b 0101*/
#define	GPIO_AFR_AF6			( GPIO_AFR_AF4 | GPIO_AFR_AF2 )						/*0b 0110*/
#define	GPIO_AFR_AF7			( GPIO_AFR_AF4 | GPIO_AFR_AF3 )						/*0b 0111*/
#define	GPIO_AFR_AF8			(1 << 3)											/*0b 1000*/
#define	GPIO_AFR_AF9			( GPIO_AFR_AF8 | GPIO_AFR_AF1 )						/*0b 1001*/
#define	GPIO_AFR_AF10			( GPIO_AFR_AF8 | GPIO_AFR_AF2 )						/*0b 1010*/
#define	GPIO_AFR_AF11			( GPIO_AFR_AF8 | GPIO_AFR_AF3 )						/*0b 1011*/
#define	GPIO_AFR_AF12			( GPIO_AFR_AF8 | GPIO_AFR_AF4 )						/*0b 1100*/
#define	GPIO_AFR_AF13			( GPIO_AFR_AF12 | GPIO_AFR_AF1 )					/*0b 1101*/
#define	GPIO_AFR_AF14			( GPIO_AFR_AF12 | GPIO_AFR_AF2 )					/*0b 1110*/
#define	GPIO_AFR_AF15			( GPIO_AFR_AF8 | GPIO_AFR_AF7 ) 					/*0b 1111*/

/*
 * Macro that return the pin configuration based on the GPIO
 * configuration definition passed as parameter. Usage:
 * <code>
 * /// Returns pin Px0
 * GPIO_ConfigDef *cfgreg = ...;
 * GPIO_PinConfigDef pincfg = __GET_PIN_CONFIG(cfgreg, GPIOx_PIN_00);
 * </code>
 */
#define	__GET_PIN_CONFIG(p, n)		( PTR(p).GPIOxPinConfig[ (n) ] )

#define __CPY_PIN_CONFIG(pfrom, pto, pinnum)	do {\
													memcpy ((pto), (pfrom), sizeof(PTR(pfrom)));\
													PTR(pto).PinNumber = pinnum;\
												} while(0)

/**
 * Internal PIN configuration for the GPIO
 */
typedef struct {
	__RW uint8_t	PinNumber;
	__RW uint8_t	PinMode;				/*!< possible values for @GPIO_PIN_MODES >*/
	__RW uint8_t	PinSpeed;				/*!< possible values for @GPIO_PIN_SPEED >*/
	__RW uint8_t	PinPullUpPullDownCtrl;	/*!< possible values for @GPIO_PIN_PUPDCTRL >*/
	__RW uint8_t	PinOType;				/*!< possible values for @GPIO_PIN_PTYPE >*/
	__RW uint8_t	PinAltFuncMode;			/*!< possible values for @GPIO_PIN_AFR >*/

	__RW irq_generic_handler_fn_t	IRQFN;	/*!< the IRQ handler for the PIN >*/
} GPIO_PinConfigDef;

/**
 * GPIO Configuration structure
 */
typedef struct {
	GPIO_RegDef	*pGPIOx;					/*Pointer to the active GPIO port to which the pin bellows*/
	GPIO_PinConfigDef GPIOxPinConfig[32];	/*Individual Pin configuration*/
} GPIO_ConfigDef;

/*********************************************************************************
 *
 * 		APIs supported by the driver related with the GPIO interface
 *
 **********************************************************************************/
/*
 * GPIO peripheral control clock
 */
void __gpio_pclk_control(GPIO_ConfigDef* pGpioCfgDef, uint8_t gpioEnDi);

/*
 * Initializes the GPIO port
 */
GPIO_ConfigDef* __gpio_init(GPIO_RegDef	*pGpioCfgDef, uint8_t initAllPins);
/*
 * De-initialize the GPIO port
 */
void __gpio_deinit(GPIO_ConfigDef *pGpioCfgDef);

/*
 * Initialize the GPIO pin
 */
void __gpio_init_pin (GPIO_ConfigDef *pCfgDef, GPIO_PinConfigDef pinConfig);

/*
 * Add a new PIN configuration to the GPIO
 */
void __gpio_add_pincfg(GPIO_ConfigDef *pGpioCfgDef, GPIO_PinConfigDef pinConfig);

/*
 * Read from the GPIO PIN
 */
uint8_t __gpio_read_pin(GPIO_PinConfigDef pinConfig, GPIO_ConfigDef *pGpioCfgDef);
/*
 * Read from the GPIO PORT
 */
uint16_t __gpio_read_port(GPIO_ConfigDef *pGpioCfgDef);
/*
 * Write a digital signal to the output pin
 */
void __gpio_write_pin(uint8_t data, GPIO_PinConfigDef pinConfig, GPIO_ConfigDef *pGpioCfgDef);
/*
 * Write a digital signal to the output port
 */
void __gpio_write_port(uint16_t bsrr, GPIO_ConfigDef *pGpioCfgDef);
/**
 * Toggle the output pin
 */
void __gpio_toggle_pin(GPIO_PinConfigDef pinConfig, GPIO_ConfigDef *pGpioCfgDef);

#ifdef __cplusplus
}
#endif

#endif /* DRIVERS_INC_STM32F401XX_GPIO_DRIVER_H_ */
