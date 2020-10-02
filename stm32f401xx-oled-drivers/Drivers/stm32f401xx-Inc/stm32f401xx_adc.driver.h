/*
 * stm32f401xx_adc.driver.h
 *
 *  Created on: Sep 23, 2020
 *      Author: jgonzalezlopes
 */

#ifndef STM32F401XX_ADC_DRIVER_H_
#define STM32F401XX_ADC_DRIVER_H_

#include "stm32f401xx.h"

#ifdef __cplusplus
 extern "C" {
#endif

/************************ COMMON MACROS **********************************************/
// Enable peripheral clock for the ports (AHB1): A/B/C/D/E/H
#define __ADC1_PCLK_EN()			do { \
										PTR(__RCC).APB2ENR |= ADC_EN; \
									} while (0)

/************************ ADC ADDRESS REFERENCES *************************************/
#define	__ADC1				((ADC_RegDef*) ADC1_BASEADDR)

/************************ ADC REG SPECIFIC DEFINITION *************************************/
///Bit 8 ADC1EN: ADC1 clock enable
#define ADC_EN				(1<<8)

//Bit 0 ~ 4 AWDCH[4:0]: Analog watchdog channel select bits
#define ADC_CR1_IN_CH0		0x00UL
#define ADC_CR1_IN_CH1		0x01UL
#define ADC_CR1_IN_CH2		0x02UL
#define ADC_CR1_IN_CH3		0x03UL
#define ADC_CR1_IN_CH4		0x04UL
#define ADC_CR1_IN_CH5		0x05UL
#define ADC_CR1_IN_CH6		0x06UL
#define ADC_CR1_IN_CH7		0x07UL
#define ADC_CR1_IN_CH8		0x08UL
#define ADC_CR1_IN_CH9		0x09UL
#define ADC_CR1_IN_CH10		0x0AUL
#define ADC_CR1_IN_CH11		0x0BUL
#define ADC_CR1_IN_CH12		0x0CUL
#define ADC_CR1_IN_CH13		0x0DUL
#define ADC_CR1_IN_CH14		0x0EUL
#define ADC_CR1_IN_CH15		0x0FUL
#define ADC_CR1_IN_CH16		0x10UL
#define ADC_CR1_IN_CH17		0x11UL
#define ADC_CR1_IN_CH18		0x12UL

/// @CR1
///Bit 5 EOCIE: Interrupt enable for EOC
#define ADC_CR1_INTERRUPT_EOC_EN			ENABLE
#define ADC_CR1_INTERRUPT_EOC_DIS			DISABLE
///Bit 6 AWDIE: Analog watchdog interrupt enable
#define ADC_CR1_INTERRUPT_WATCHDOG_EN		ENABLE
#define ADC_CR1_INTERRUPT_WATCHDOG_DIS		DISABLE
///Bit 7 JEOCIE: Interrupt enable for injected channels
#define ADC_CR1_INTERRUPT_INJECTCH_EN		ENABLE
#define ADC_CR1_INTERRUPT_INJECTCH_DIS		DISABLE
///Bit 8 SCAN: Scan mode
#define ADC_CR1_SCAN_EN						ENABLE
#define ADC_CR1_SCAN_DIS					DISABLE
///Bit 9 AWDSGL: Enable the watchdog on a single channel in scan mode
#define ADC_CR1_ANALOG_WATCHDOG_EN			ENABLE
#define ADC_CR1_ANALOG_WATCHDOG_DIS			DISABLE
///Bit 10 JAUTO: Automatic injected group conversion
#define ADC_CR1_AUTOINJECT_GRPCONV_EN		ENABLE
#define ADC_CR1_AUTOINJECT_GRPCONV_DIS		DISABLE
///Bit 11 DISCEN: Discontinuous mode on regular channels
#define ADC_CR1_DISMODE_REGCH_EN			ENABLE
#define ADC_CR1_DISMODE_REGCH_DIS			DISABLE
///Bit 12 JDISCEN: Discontinuous mode on injected channels
#define ADC_CR1_DISMODE_INJECTCH_EN			ENABLE
#define ADC_CR1_DISMODE_INJECTCH_DIS		DISABLE
///Bits 15:13 DISCNUM[2:0]: Discontinuous mode channel count
#define ADC_CR1_DISCONTNUM_1CH					0x0 /*1 channel*/
#define ADC_CR1_DISCONTNUM_2CH					0x1 /*2 channels*/
#define ADC_CR1_DISCONTNUM_3CH					0x2 /*3 channels*/
#define ADC_CR1_DISCONTNUM_4CH					0x3 /*4 channels*/
#define ADC_CR1_DISCONTNUM_5CH					0x4 /*5 channels*/
#define ADC_CR1_DISCONTNUM_6CH					0x5 /*6 channels*/
#define ADC_CR1_DISCONTNUM_7CH					0x6 /*7 channels*/
#define ADC_CR1_DISCONTNUM_8CH					0x7 /*8 channels*/
///Bit 22 JAWDEN: Analog watchdog enable on injected channels
#define ADC_CR1_ANALOG_WATCHDOG_INJCH_EN	ENABLE
#define ADC_CR1_ANALOG_WATCHDOG_INJCH_DIS	DISABLE
///Bit 23 AWDEN: Analog watchdog enable on regular channels
#define ADC_CR1_ANALOG_WATCHDOG_REGULARCH_EN	ENABLE
#define ADC_CR1_ANALOG_WATCHDOG_REGULAR_DIS		DISABLE
///Bits 25:24 RES[1:0]: Resolution
#define ADC_CR1_RESOLUTION_12BIT			0x0
#define ADC_CR1_RESOLUTION_10BIT			0x1
#define ADC_CR1_RESOLUTION_8BIT				0x2
#define ADC_CR1_RESOLUTION_6BIT				0x3
///Bit 26 OVRIE: Overrun interrupt enable
#define ADC_CR1_OVERRUN_INTERRUP_EN			ENABLE
#define ADC_CR1_OVERRUN_INTERRUP_DIS		DISABLE

/// @CR2
///Bits 27:24 EXTSEL[3:0]: External event select for regular group
#define	ADC_CR2_TIM1CC1_EVENT				0x0	/*0000: Timer 1 CC1 event*/
#define	ADC_CR2_TIM1CC2_EVENT				0x1 /*0001: Timer 1 CC2 event*/
#define	ADC_CR2_TIM1CC3_EVENT				0x2 /*0010: Timer 1 CC3 event*/
#define	ADC_CR2_TIM2CC2_EVENT				0x3 /*0011: Timer 2 CC2 event*/
#define	ADC_CR2_TIM2CC3_EVENT				0x4 /*0100: Timer 2 CC3 event*/
#define	ADC_CR2_TIM2CC4_EVENT				0x5 /*0101: Timer 2 CC4 event*/
#define	ADC_CR2_TIM2TRGO_EVENT				0x6 /*0110: Timer 2 TRGO event*/
#define	ADC_CR2_TIM3CC1_EVENT				0x7 /*0111: Timer 3 CC1 event*/
#define	ADC_CR2_TIM3TRGO_EVENT				0x8 /*1000: Timer 3 TRGO event*/
#define	ADC_CR2_TIM4CC4_EVENT				0x9 /*1001: Timer 4 CC4 event*/
#define	ADC_CR2_TIM5CC1_EVENT				0xA /*1010: Timer 5 CC1 event*/
#define	ADC_CR2_TIM5CC2_EVENT				0xB /*1011: Timer 5 CC2 event*/
#define	ADC_CR2_TIM5CC3_EVENT				0xC /*1100: Timer 5 CC3 event*/
#define	ADC_CR2_EXTILN11_EVENT				0xF /*1111: EXTI line11*/
///Bits 29:28 EXTEN: External trigger enable for regular channels
#define	ADC_CR2_TRIGDETECT_DIS				0x0 /*00: Trigger detection disabled*/
#define	ADC_CR2_TRIGDETECT_RISEEDGE			0x1 /*01: Trigger detection on the rising edge*/
#define	ADC_CR2_TRIGDETECT_FALLEDGE			0x2 /*10: Trigger detection on the falling edge*/
#define	ADC_CR2_TRIGDETECT_BOTH				0x3 /*11: Trigger detection on both the rising and falling edges*/
///Bit 30 SWSTART: Start conversion of regular channels. Note: This bit can be set only when ADON = 1 otherwise no conversion is launched.
#define	ADC_CR2_STARTCONV_REGCH_SET			(SET<<30UL)		/*1: Starts conversion of regular channels*/
#define	ADC_CR2_STARTCONV_REGCH_RESET		(RESET<<30UL)	/*0: Reset state*/

/// @SMPR1
///Bits 26:0 SMPx[2:0]: Channel x sampling time selection
#define ADC_SMP1_3_CYCLES					0x0
#define ADC_SMP1_15_CYCLES					0x1
#define ADC_SMP1_28_CYCLES					0x2
#define ADC_SMP1_56_CYCLES					0x3
#define ADC_SMP1_84_CYCLES					0x4
#define ADC_SMP1_112_CYCLES					0x5
#define ADC_SMP1_144_CYCLES					0x6
#define ADC_SMP1_480_CYCLES					0x7

/// @SMPR2
///SMPx[2:0]: Channel x sampling time selection
#define ADC_SMP2_3_CYCLES					0x0
#define ADC_SMP2_15_CYCLES					0x1
#define ADC_SMP2_28_CYCLES					0x2
#define ADC_SMP2_56_CYCLES					0x3
#define ADC_SMP2_84_CYCLES					0x4
#define ADC_SMP2_112_CYCLES					0x5
#define ADC_SMP2_144_CYCLES					0x6
#define ADC_SMP2_480_CYCLES					0x7

/// ADC status register (ADC_SR)
///Bit 1 EOC: Regular channel end of conversion
#define ADC_SR_CHCONVERSATION_INCOMPLETE	~0x2
#define ADC_SR_CHCONVERSATION_COMPLETE		0x2

/************************ ADC REGISTER DEFINITIONS *************************************/
/**
 * @brief Analog (ADC) data register configuration (Analog to Digital Converter)
 */
typedef struct
{
  __RW uint32_t SR;     /*!< ADC status register,                         Address offset: 0x00 */
  __RW uint32_t CR1;    /*!< ADC control register 1,                      Address offset: 0x04 */
  __RW uint32_t CR2;    /*!< ADC control register 2,                      Address offset: 0x08 */
  __RW uint32_t SMPR1;  /*!< ADC sample time register 1,                  Address offset: 0x0C */
  __RW uint32_t SMPR2;  /*!< ADC sample time register 2,                  Address offset: 0x10 */
  __RW uint32_t JOFR[4];  /*!< ADC injected channel data offset register 1, Address offset: 0x14 */
  	  	  	  	  	  	  /*!< ADC injected channel data offset register 2, Address offset: 0x18 */
  						  /*!< ADC injected channel data offset register 3, Address offset: 0x1C */
  	  	  	  	  	  	  /*!< ADC injected channel data offset register 4, Address offset: 0x20 */
  __RW uint32_t HTR;    /*!< ADC watchdog higher threshold register,      Address offset: 0x24 */
  __RW uint32_t LTR;    /*!< ADC watchdog lower threshold register,       Address offset: 0x28 */
  __RW uint32_t SQR[3]; /*!< ADC regular sequence register 1,             Address offset: 0x2C */
  	  	  	  	  	  	/*!< ADC regular sequence register 2,             Address offset: 0x30 */
  	  	  	  	  	  	/*!< ADC regular sequence register 3,             Address offset: 0x34 */
  __RW uint32_t JSQR;   /*!< ADC injected sequence register,              Address offset: 0x38*/
  __RW uint32_t JDR[4]; /*!< ADC injected data register 1,                Address offset: 0x3C */
  	  	  	  	  	  	/*!< ADC injected data register 2,                Address offset: 0x40 */
  	  	  	  	  	    /*!< ADC injected data register 3,                Address offset: 0x44 */
  						/*!< ADC injected data register 4,                Address offset: 0x48 */
  __RW uint32_t DR;     /*!< ADC regular data register,                   Address offset: 0x4C */
} ADC_RegDef;

/*********************** FUNCTION PROTOTYPES *********************************************/
void __adc_init(uint16_t pinNumber);
uint32_t __adc_read(void);

#ifdef __cplusplus
}
#endif

#endif /* STM32F401XX_ADC_DRIVER_H_ */
