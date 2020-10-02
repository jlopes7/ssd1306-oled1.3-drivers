/*
 * stm32f401xx_exti.driver.h
 *
 *  Created on: Sep 24, 2020
 *      Author: jgonzalezlopes
 */

#ifndef STM32F401XX_EXTI_DRIVER_H_
#define STM32F401XX_EXTI_DRIVER_H_

#include "stm32f401xx.h"

#ifdef __cplusplus
 extern "C" {
#endif

/*************** EXTI constants & macros **************************************************/
/*!<@SYSCFG_EXTICR1>*/
#define	EXTI_CR_GPIOA			0x0UL
#define	EXTI_CR_GPIOB			0x1UL
#define	EXTI_CR_GPIOC			0x2UL
#define	EXTI_CR_GPIOD			0x3UL
#define	EXTI_CR_GPIOE			0x4UL
#define	EXTI_CR_GPIOH			0x7UL

/**
 * @brief	- Return the correct external interrupt configuration register for the EXTI
 * 				connection to the NVIC
 * @return	- The numeric representation (4-bit) of the peripheral port to be used/connected
 */
#define __GET_CR_PORT(p)		( p == __GPIOA ? EXTI_CR_GPIOA : \
								( p == __GPIOB ? EXTI_CR_GPIOB : \
								( p == __GPIOC ? EXTI_CR_GPIOC : \
								( p == __GPIOD ? EXTI_CR_GPIOD : \
								( p == __GPIOE ? EXTI_CR_GPIOH : EXTI_CR_GPIOA/*this is an error!!!!*/ ) ) ) ) )

/*************** EXTI definition structure **************************************************/
/**
 * @brief	- The EXTI internal peripheral register structure
 */
typedef struct {
	__RW uint32_t	IMR;	/*Interrupt mask register (EXTI_IMR). Offset: 0x00*/
	__RW uint32_t	EMR;	/*Event mask register (EXTI_EMR). Offset: 0x04*/
	__RW uint32_t	RTSR;	/*Rising trigger selection register (EXTI_RTSR). Offset: 0x08*/
	__RW uint32_t	FTSR;	/*Falling trigger selection register (EXTI_FTSR). Offset: 0x0C*/
	__RW uint32_t	SWIER;	/*Software interrupt event register (EXTI_SWIER). Offset: 0x10*/
	__RW uint32_t	PR;		/*Pending register (EXTI_PR). Offset: 0x14*/
} EXTI_RegDef;

/*
 * @brief	- Pointer reference to the EXTI interface to NVIC
 */
#define	__EXTI		( (EXTI_RegDef*) EXTI_BASEADDR )

#ifdef __cplusplus
}
#endif

#endif /* STM32F401XX_EXTI_DRIVER_H_ */
