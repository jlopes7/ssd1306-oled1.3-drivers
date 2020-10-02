/*
 * stm32f401xx_adc.driver.c
 *
 *  Created on: Sep 24, 2020
 *      Author: jgonzalezlopes
 */

#include "stm32f401xx_adc.driver.h"

/**************************************************************************
 * @fn			- __adc_init
 * @brief		- ADC initialization
 *
 * @param [in]  - The number of the PIN to sample the analog data
 *
 * @return		- none
 * @Note		- none
 */
void __adc_init(uint16_t pinNumber) {
	uint8_t idx = pinNumber / 6,
			pinmap = (pinNumber % 6) * 0x5UL;

	__NONOTUSEDWARN_ pinmap;

	__ADC1_PCLK_EN();

	/// Needs to add in reverse form
	if (idx == 0x0) idx = 0x2;
	else if (idx == 0x2) idx = 0x0;

	PTR(__ADC1).CR2  = DISABLE;	// Disable ADC
	PTR(__ADC1).SQR[ idx ] = 0x1;//(0x1 << pinmap);		// Nth conversion in regular sequence. Fixed to 0b 00001 (need to change this in the future)
	PTR(__ADC1).CR2 |= ENABLE;	// Enable ADC
}

/**************************************************************************
 * @fn			- __adc_read
 * @brief		- ADC read an entry from the pin
 *
 * @return		- the 16-bit data read from the analog input
 * @Note		- none
 */
uint32_t __adc_read(void) {
	PTR(__ADC1).CR2 |= ADC_CR2_STARTCONV_REGCH_SET; //0x40000000
	// Wait until the end of convertion is finished/completed
	while ( !(PTR(__ADC1).SR & ADC_SR_CHCONVERSATION_COMPLETE) ) ;// wait until an amount is read

	return PTR(__ADC1).DR;
}
