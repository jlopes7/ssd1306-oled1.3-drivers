/*
 * stm32401xx_systick.driver.c
 *
 *  Created on: Sep 23, 2020
 *      Author: jgonzalezlopes
 */

#include "stm32f401xx_systick.driver.h"

/**************************************************************************
 * @fn			- __aux_clkconfig
 * @brief		- Internal (private) MCU SysTick clock configuration function
 *
 * @param[in]	- the cycles to run the counting
 * @param[in]	- the time to wait, or number of cycles to wait for
 *
 * @return		- none
 * @Note		- none
 */
void __aux_clkconfig(uint32_t reloadCycles, size_t time) {
	int count = 0;

	/// By default we will set the clock to be "internally controlled"
	PTR(__SysTick).CSR 		= ( SYSTICK_CSR_ENABLE | SYSTICK_CSR_CLKSOURCE_PROC ) ;
	PTR(__SysTick).RELOAD	= reloadCycles;
	PTR(__SysTick).VAL		= 0x0; // Clear the value

	for (;count < time;count++) { /// Counts per configured SysTick cycles: nano seconds
		while ( (PTR(__SysTick).CSR & SYSTICK_COUNTFLAG_TIMECOUNTED)== 0x0 ) ;
	}
}

/**************************************************************************
 * @fn			- _delay_ns
 * @brief		- Waits the number of cycles in nanoseconds
 *
 * @param[in]	- the number of cycles separated in nanoseconds
 *
 * @return		- none
 * @Note		- none
 */
void _delay_ns(size_t ns) {
	__aux_clkconfig(SYSTICK_NANO_CYCLES, ns);
}

/**************************************************************************
 * @fn			- _delay_ms
 * @brief		- Waits the number of cycles in milliseconds
 *
 * @param[in]	- the number of cycles separated in nano seconds
 *
 * @return		- none
 * @Note		- none
 */
void _delay_ms(size_t ms) {
	__aux_clkconfig(SYSTICK_MILLI_CYCLES, ms);
}

/**************************************************************************
 * @fn			- _delay_s
 * @brief		- Waits the number of cycles in seconds
 *
 * @param[in]	- the number of cycles separated in seconds
 *
 * @return		- none
 * @Note		- none
 */
void _delay_s(size_t s) {
	__aux_clkconfig(SYSTICK_SEC_CYCLES, s);
}
