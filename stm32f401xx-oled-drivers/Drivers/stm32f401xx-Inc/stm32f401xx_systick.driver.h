/*
 * stm32f401xx_systick.driver.h
 *
 *  Created on: Sep 23, 2020
 *      Author: jgonzalezlopes
 */

#ifndef DRIVERS_INC_STM32F401XX_SYSTICK_DRIVER_H_
#define DRIVERS_INC_STM32F401XX_SYSTICK_DRIVER_H_

#include "stm32f401xx.h"

#ifdef __cplusplus
 extern "C" {
#endif

/******************* Systick configuration definition ***************************************/
#define SYSTICK_CSR_ENABLE				(1<<0)
#define SYSTICK_CSR_DISABLE				(0<<0)
#define SYSTICK_CSR_TICKINT_ASSERT		(1<<1)
#define SYSTICK_CSR_TICKINT_NOASSERT	(0<<1)
#define SYSTICK_CSR_CLKSOURCE_EXT		(0<<2)
#define SYSTICK_CSR_CLKSOURCE_PROC		(1<<2)
#define SYSTICK_COUNTFLAG_TIMECOUNTED	(1<<16)

#define SYSTICK_CALIB_NOREFCLK			(0<<31)
#define SYSTICK_CALIB_REFCLK			(1<<31)
#define SYSTICK_CALIB_SKEWEXACT			(0<<30)
#define SYSTICK_CALIB_SKEWINEXACTNGIVEN	(1<<30)

/*
 * Cycles for the MCU considering a Cortex of 16MHz
 */
#define	SYSTICK_SEC_CYCLES				(16000000UL -1UL)
#define	SYSTICK_MILLI_CYCLES			(16000UL -1UL)
#define	SYSTICK_NANO_CYCLES				(16UL -1UL)

/******************* Memory Definition & Mapping ********************************************/
#define	SYSTICK_BASEADDR	( CORTEXM4_SCS_BASEADDR | 0x0010UL )

/**
 * SysTick struct mapping definition
 */
#define	__SysTick			((Systick_RegDef*) SYSTICK_BASEADDR)

/******************* Systick struct definition **********************************************/
/*
 * Systick internal register definition and configuration
 */
typedef struct {
	__RW uint32_t	CSR;		/*Control and Status register	[SYST_CSR]- Offset: 0x10*/
	__RW uint32_t	RELOAD;		/*Reload value register			[SYST_RVR]- Offset: 0x14*/
	__RW uint32_t	VAL;		/*Current value register 		[SYST_CVR]- Offset: 0x18*/
	__RW uint32_t	CALIB;		/*Calibration value register	[SYST_CALIB]- Offset: 0x1C*/
} Systick_RegDef;

/******************* Function Prototype **********************************************/
void _delay_ns(size_t ns);
void _delay_ms(size_t ms);
void _delay_s(size_t s);

#ifdef __cplusplus
}
#endif

#endif /* DRIVERS_INC_STM32F401XX_SYSTICK_DRIVER_H_ */
