/*
 * stm32f401xx.c
 *
 *  Created on: Sep 21, 2020
 *      Author: jgonzalezlopes
 */

#include "stm32f401xx_exti.driver.h"
#include "stm32f401xx_systick.driver.h"

__RW irq_generic_handler_fn_t __registered_fns[88];

/**
 * Return the alternate function low level registers
 */
__RW uint32_t* GPIOGetAFRLow(GPIO_RegDef* pGpio) {
	return &PTR(pGpio).AFR[LOW];
}

/**
 * Return the alternate function high level registers
 */
__RW uint32_t* GPIOGetAFRHigh(GPIO_RegDef* pGpio) {
	return &PTR(pGpio).AFR[HIGH];
}

/**************************************************************************
 * @fn			- __irq_config
 * @brief		- Configure the processor side (NVIC) for the interrupts
 *
 * @param[in]	- the IRQ number (hierarchy) to be configured
 * @param[in]	- the IRQ priority for the hierarchy number
 * @param[in]	- enable/disable the IRQ
 * @param[in]	- a function pointer to be registered to the IRQ number
 *
 * @return		- none
 * @Note		- none
 */
void __irq_config(uint8_t IRQNumber, uint32_t IRQPriority, uint8_t IRQEnableDisable, irq_generic_handler_fn_t fn) {
	uint8_t hidx 	  = IRQNumber / 32UL,
			hbitshift = IRQNumber % 32UL;
	uint8_t pidx	  = IRQNumber / 4UL,
			/*
			 * Priority bits are implemented in the most-significant bits of the priority configuration registers in the NVIC,
			 * that's why we always need to shift 4 bits from the lower portion of a byte to the higher 4 bits of a byte: 16 ~ 240
			 */
			pbitshift = ((IRQNumber % 0x4UL) * 0x8UL) + /*small priorities need to shift 4-bits*/(0x8UL - NUM_PRIORITY_BITS_IMPL);
														/*Initial 4-bits on every 8-bits priority config. are not available*/
														/*The initial 4 bits are reserved to internal system interrupts: 0x0000 0000 ~ 0x0000 003C*/

	/// Register the function to be triggered
	__registered_fns[ IRQNumber ] = fn;

	/// Configure the priority !
	PTR(__NVIC).IPR[ pidx ] |= (IRQPriority << pbitshift);

	/// Configure the hierarchy !
	if (IRQEnableDisable & ENABLE) {
		PTR(__NVIC).ISER [ hidx ] |= ( 1 << hbitshift );
	}
	else {
		PTR(__NVIC).ICER [ hidx ] |= ( 1 << hbitshift );
	}
}

/**************************************************************************
 * @fn			- __aux_irq_generic_handler
 * @brief		- Auxliary IRQ function handler for all the IRQ numbers
 *
 * @param [in]	- the IRQ number to be used
 *
 * @return		- none
 * @Note		- none
 */
void __aux_irq_generic_handler(uint8_t IRQNumber, uint8_t pinNumber) {
	_delay_ms(200); // Wait about 200ms to avoid debouncing of the button
	/// Clear the pending register in the EXTI so the handler are not called twice !!
	if ( PTR(__EXTI).PR & (0x1 << pinNumber) ) {
		PTR(__EXTI).PR |= (0x1 << pinNumber);
	}

	//// Callback function needs to be called!!!
	if (__registered_fns [ IRQNumber ]) {
		__registered_fns [ IRQNumber ] (pinNumber, 0x0);
	}
}
/**************************************************************************
 * @fn			- EXTI0_IRQHandler
 * @brief		- Default IRQ function handler for EXTI0
 *
 * @return		- none
 * @Note		- none
 */
void EXTI0_IRQHandler(void) {
	__aux_irq_generic_handler(IRQ_HIERARCHY_EXTI0, GPIOx_PIN_00);
}
/**************************************************************************
 * @fn			- EXTI1_IRQHandler
 * @brief		- Default IRQ function handler for EXTI1
 *
 * @return		- none
 * @Note		- none
 */
void EXTI1_IRQHandler(void) {
	__aux_irq_generic_handler(IRQ_HIERARCHY_EXTI1, GPIOx_PIN_01);
}
/**************************************************************************
 * @fn			- EXTI2_IRQHandler
 * @brief		- Default IRQ function handler for EXTI2
 *
 * @return		- none
 * @Note		- none
 */
void EXTI2_IRQHandler(void) {
	__aux_irq_generic_handler(IRQ_HIERARCHY_EXTI2, GPIOx_PIN_02);
}
/**************************************************************************
 * @fn			- EXTI3_IRQHandler
 * @brief		- Default IRQ function handler for EXTI3
 *
 * @return		- none
 * @Note		- none
 */
void EXTI3_IRQHandler(void) {
	__aux_irq_generic_handler(IRQ_HIERARCHY_EXTI3, GPIOx_PIN_03);
}
/**************************************************************************
 * @fn			- EXTI4_IRQHandler
 * @brief		- Default IRQ function handler for EXTI4
 *
 * @return		- none
 * @Note		- none
 */
void EXTI4_IRQHandler(void) {
	__aux_irq_generic_handler(IRQ_HIERARCHY_EXTI4, GPIOx_PIN_04);
}
/**************************************************************************
 * @fn			- EXTI9_5_IRQHandler
 * @brief		- Default IRQ function handler for EXTI9_5
 *
 * @return		- none
 * @Note		- none
 */
void EXTI9_5_IRQHandler(void) {
	__aux_irq_generic_handler(IRQ_HIERARCHY_EXTI9_5, GPIOx_PIN_05);
	__aux_irq_generic_handler(IRQ_HIERARCHY_EXTI9_5, GPIOx_PIN_06);
	__aux_irq_generic_handler(IRQ_HIERARCHY_EXTI9_5, GPIOx_PIN_07);
	__aux_irq_generic_handler(IRQ_HIERARCHY_EXTI9_5, GPIOx_PIN_08);
	__aux_irq_generic_handler(IRQ_HIERARCHY_EXTI9_5, GPIOx_PIN_09);
}
/**************************************************************************
 * @fn			- EXTI15_10_IRQHandler
 * @brief		- Default IRQ function handler for EXTI15_10
 *
 * @return		- none
 * @Note		- none
 */
void EXTI15_10_IRQHandler(void) {
	__aux_irq_generic_handler(IRQ_HIERARCHY_EXTI15_10, GPIOx_PIN_10);
	__aux_irq_generic_handler(IRQ_HIERARCHY_EXTI15_10, GPIOx_PIN_11);
	__aux_irq_generic_handler(IRQ_HIERARCHY_EXTI15_10, GPIOx_PIN_12);
	__aux_irq_generic_handler(IRQ_HIERARCHY_EXTI15_10, GPIOx_PIN_13);
	__aux_irq_generic_handler(IRQ_HIERARCHY_EXTI15_10, GPIOx_PIN_14);
	__aux_irq_generic_handler(IRQ_HIERARCHY_EXTI15_10, GPIOx_PIN_15);
}
