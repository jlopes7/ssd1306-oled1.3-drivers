/*
 * stm32f401xx.h
 *
 *  Created on: Sep 21, 2020
 *      Author: jgonzalezlopes
 */

#ifndef DRIVERS_INC_STM32F401XX_H_
#define DRIVERS_INC_STM32F401XX_H_

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <stddef.h>
#include <memory.h>
#include <string.h>

#ifdef __cplusplus
 extern "C" {
#endif

#define LOW			0x0UL
#define	HIGH		0x1UL

#define YES			HIGH
#define NO			LOW
#define ENABLE		HIGH
#define	DISABLE		LOW
#define SET			ENABLE
#define RESET		DISABLE
#define GPIO_PIN_SET	SET
#define GPIO_PIN_RESET	RESET

#define	NUM_PRIORITY_BITS_IMPL		0x4UL

#define __WEAK						__attribute__(weak)
#define __NONOTUSEDWARN_			(void)

/*
 * @brief	- Function type definition for handling hierarchy numbers
 */
typedef	void	(*irq_generic_handler_fn_t)(uint32_t, void*);

/**************************************** MCU INTERNAL BUFFER DISTRIBUTION ***************************************/
/*
 * Private Cortex M4 Internal Bus addresses
 */
#define	CORTEXM4_INTERNAL_BASEADDR	0xE0000000UL	/*Cortex M4 Internal Peripherals base address - goes all the way to 0xE0040000*/
#define CORTEXM4_SCS_BASEADDR		( CORTEXM4_INTERNAL_BASEADDR | 0xE000UL ) /*Internal System Control Space address*/
#define CORTEXM4_FPB_BASEADDR		( CORTEXM4_INTERNAL_BASEADDR | 0x2000UL ) /*Internal Flashpatch and Breakpoint address*/
#define CORTEXM4_DWT_BASEADDR		( CORTEXM4_INTERNAL_BASEADDR | 0x1000UL ) /*Data Watchpoint and Trace address*/
#define CORTEXM4_ITM_BASEADDR		( CORTEXM4_INTERNAL_BASEADDR | 0x0000UL ) /*Instrumentation Trace Macrocell address*/
#define CORTEXM4_NVIC_BASEADDR		( CORTEXM4_SCS_BASEADDR 	 | 0x0100UL ) /*Nested Vectored Interrupt Controller*/

/*
 * ARM Cortex M4 processor NVIC ISERx register addresses
 */

/*
 * Define the base address of the Flash and SRAM memories
 */
#define	FLASH_BASEADDR				0x08000000UL	/*Base Flash address (Main Memory in the datasheet) defined in the MCU*/
#define	SRAM1_BASEADDR				0x20000000UL	/*Base SRAM address defined in the MCU*/
#define	ROM_BASEADDR				0x1FFF0000UL	/*System memory address in the MCU*/

/*
 * Define the base addresses for the Bus domains for the STM32F4xx MCUs
 */
#define	PERIPHERAL_BASE			0x40000000UL
#define AHB1_BASE				0x40020000UL
#define AHB2_BASE				0x50000000UL
#define APB1_BASE				PERIPHERAL_BASE
#define APB2_BASE				0x40010000UL

/*
 * RCC and DMA1 + DM2 definitions
 */
#define RCC_BASEADDR			(AHB1_BASE + 0x3800UL)
#define CRC_BASEADDR			(AHB1_BASE + 0x3000UL)
#define DMA1_BASEADDR			(AHB1_BASE + 0x6000UL)
#define DMA2_BASEADDR			(AHB1_BASE + 0x6400UL)

/*
 * Define the base addresses for all the peripherals which are
 * hanging on the AHB1 bus
 */
#define	GPIOA_BASEADDR			(AHB1_BASE + 0x0000UL)
#define	GPIOB_BASEADDR			(AHB1_BASE + 0x0400UL)
#define	GPIOC_BASEADDR			(AHB1_BASE + 0x0800UL)
#define	GPIOD_BASEADDR			(AHB1_BASE + 0x0C00UL)
#define	GPIOE_BASEADDR			(AHB1_BASE + 0x1000UL)
#define	GPIOH_BASEADDR			(AHB1_BASE + 0x1C00UL)

/*
 * Define the base addresses for all the peripherals which are
 * hanging on the APB1 bus - they are self explanatory
 */
#define TIM2_BASEADDR			(APB1_BASE + 0x0000UL)
#define TIM3_BASEADDR			(APB1_BASE + 0x0400UL)
#define TIM4_BASEADDR			(APB1_BASE + 0x0800UL)
#define TIM5_BASEADDR			(APB1_BASE + 0x0C00UL)
#define RTCBKP_REG_BASEADDR		(APB1_BASE + 0x2800UL)
#define WWDG_BASEADDR			(APB1_BASE + 0x2C00UL)
#define IWDG_BASEADDR			(APB1_BASE + 0x3000UL)
#define I2S2EXT_BASEADDR		(APB1_BASE + 0x3400UL)
#define SPI2_I2S2_BASEADDR		(APB1_BASE + 0x3800UL)
#define SPI3_I2S3_BASEADDR		(APB1_BASE + 0x3C00UL)
#define I2S3EXT_BASEADDR		(APB1_BASE + 0x4000UL)
#define USART2_BASEADDR			(APB1_BASE + 0x4400UL)
#define I2C1_BASEADDR			(APB1_BASE + 0x5400UL)
#define I2C2_BASEADDR			(APB1_BASE + 0x5800UL)
#define I2C3_BASEADDR			(APB1_BASE + 0x5C00UL)
#define PWR_BASEADDR			(APB1_BASE + 0x7000UL)

/*
 * Define the base addresses for all the peripherals which are
 * hanging on the APB2 bus - they are self explanatory
 */
#define TIM1_BASEADDR			(APB2_BASE + 0x0000UL)
#define USART1_BASEADDR			(APB2_BASE + 0x1000UL)
#define USART6_BASEADDR			(APB2_BASE + 0x1400UL)
#define ADC1_BASEADDR			(APB2_BASE + 0x2000UL)
#define SDIO_BASEADDR			(APB2_BASE + 0x2C00UL)
#define SPI1_BASEADDR			(APB2_BASE + 0x3000UL)
#define SPI4_BASEADDR			(APB2_BASE + 0x3400UL)
#define SYSCFG_BASEADDR			(APB2_BASE + 0x3800UL)
#define EXTI_BASEADDR			(APB2_BASE + 0x3C00UL)
#define TIM9_BASEADDR			(APB2_BASE + 0x4000UL)
#define TIM10_BASEADDR			(APB2_BASE + 0x4400UL)
#define TIM11_BASEADDR			(APB2_BASE + 0x4800UL)

/*
 * Pointers for the Flash and ROM
 */
#define	FLASH						((uint32_t*) FLASH_BASEADDR)
#define ROM							((uint32_t*) SRAM1_BASEADDR)

/*
 * Configure the NVIC hierarchy numbers (position in the NCIV table) for
 * the STM32F401xx board
 */
#define	IRQ_HIERARCHY_EXTI0			0x06UL
#define	IRQ_HIERARCHY_EXTI1			0x07UL
#define	IRQ_HIERARCHY_EXTI2			0x08UL
#define	IRQ_HIERARCHY_EXTI3			0x09UL
#define	IRQ_HIERARCHY_EXTI4			0x0AUL
#define	IRQ_HIERARCHY_EXTI9_5		0x17UL
#define	IRQ_HIERARCHY_EXTI15_10		0x28UL

#define IRQ_HIERARCHY_SPI1			0x23UL
#define IRQ_HIERARCHY_SPI2			0x24UL
#define IRQ_HIERARCHY_SPI3			0x33UL
#define IRQ_HIERARCHY_SPI4			0x54UL

/*
 * Configure the NVIC priority numbers (position in the NCIV table) for
 * the STM32F401xx board (1 byte each = 8-bits)
 */
#define	IRQ_PRIORITY_EXTI0			0x0DUL
#define	IRQ_PRIORITY_EXTI1			0x0EUL
#define	IRQ_PRIORITY_EXTI2			0x0FUL
#define	IRQ_PRIORITY_EXTI3			0x10UL
#define	IRQ_PRIORITY_EXTI4			0x11UL
#define	IRQ_PRIORITY_EXTI9_5		0x1EUL
#define	IRQ_PRIORITY_EXTI15_10		0x2FUL

#define IRQ_PRIORITY_SPI1			0x2AUL
#define IRQ_PRIORITY_SPI2			0x2BUL
#define IRQ_PRIORITY_SPI3			0x3AUL
#define IRQ_PRIORITY_SPI4			0x5BUL

/*
 * NVIC IRQ Priorities definitions based on the Cortex-M4 4-bit
 * configuration of the MCU (STM32F401xx)
 */
#define NVIC_IRQ_PRIORITY_00		0x0UL
#define NVIC_IRQ_PRIORITY_01		0x1UL
#define NVIC_IRQ_PRIORITY_02		0x2UL
#define NVIC_IRQ_PRIORITY_03		0x3UL
#define NVIC_IRQ_PRIORITY_04		0x4UL
#define NVIC_IRQ_PRIORITY_05		0x5UL
#define NVIC_IRQ_PRIORITY_06		0x6UL
#define NVIC_IRQ_PRIORITY_07		0x7UL
#define NVIC_IRQ_PRIORITY_08		0x8UL
#define NVIC_IRQ_PRIORITY_09		0x9UL
#define NVIC_IRQ_PRIORITY_10		0xAUL
#define NVIC_IRQ_PRIORITY_11		0xBUL
#define NVIC_IRQ_PRIORITY_12		0xCUL
#define NVIC_IRQ_PRIORITY_13		0xDUL
#define NVIC_IRQ_PRIORITY_14		0xEUL
#define NVIC_IRQ_PRIORITY_15		0xFUL

/* IO definitions (access restrictions to peripheral registers) */
/**
    \defgroup CMSIS_glob_defs CMSIS Global Defines

    <strong>IO Type Qualifiers</strong> are used
    \li to specify the access to peripheral variables.
    \li for automatic generation of peripheral register debug information.
*/
#ifdef __cplusplus
  #define   __R     volatile             /*!< Defines 'read only' permissions */
#else
  #define   __R     		 			 /*!< Defines 'read only' permissions */
#endif
#define     __RW    volatile             /*!< Defines 'read / write' permissions */

/*@} end of group Cortex_M0 */

/***************************** PERIPHERAL REGISTER DEFINITION STRUCTURES ***********************************/
/*
 * Definition peripheral structure for the RCC register
 */
typedef struct {
	__RW uint32_t CR;					/*Clock control register. 		Offset: 0x00*/
	__RW uint32_t PLLCFGR;				/*PLL configuration register. 	Offset: 0x04*/
	__RW uint32_t CFGR;					/*Clock configuration register. Offset: 0x08*/
	__RW uint32_t CIR;					/*Clock interrupt register.		Offset: 0x0C*/
	__RW uint32_t AHB1RSTR;				/*AHB1 peripheral reset register. Offset: 0x10*/
	__RW uint32_t AHB2RSTR;				/*AHB2 peripheral reset register. Offset: 0x14*/
	__R  uint32_t RESERVED0[2];			/*Reserved, 0x18 ~ 0x20*/
	__RW uint32_t APB1RSTR;				/*APB1 peripheral reset register. Offset: 0x20*/
	__RW uint32_t APB2RSTR;				/*APB2 peripheral reset register. Offset: 0x24*/
	__R  uint32_t RESERVED1[2];			/*Reserved, 0x2C ~ 0x30*/
	__RW uint32_t AHB1ENR;				/*AHB1 peripheral clock enable register. Offset: 0x30*/
	__RW uint32_t AHB2ENR;				/*AHB2 peripheral clock enable register. Offset: 0x34*/
	__R  uint32_t RESERVED2[2];			/*Reserved, 0x38 ~ 0x40*/
	__RW uint32_t APB1ENR;				/*APB1 peripheral clock enable register. Offset: 0x40*/
	__RW uint32_t APB2ENR;				/*APB2 peripheral clock enable register. Offset: 0x44*/
	__R  uint32_t RESERVED3[2];			/*Reserved, 0x48 ~ 0x50*/
	__RW uint32_t AHB1LPENR;			/*AHB1 peripheral clock enable in low power mode register. Offset: 0x50*/
	__RW uint32_t AHB2LPENR;			/*AHB2 peripheral clock enable in low power mode register. Offset: 0x54*/
	__R  uint32_t RESERVED4[2];			/*Reserved, 0x58 ~ 0x60*/
	__RW uint32_t APB1LPENR;			/*APB1 peripheral clock enable in low power mode register. Offset: 0x60*/
	__RW uint32_t APB2LPENR;			/*APB2 peripheral clock enable in low power mode register. Offset: 0x64*/
	__R  uint32_t RESERVED5[2];			/*Reserved, 0x68 ~ 0x70*/
	__RW uint32_t BDCR;					/*RCC Backup domain control register. Offset: 0x70*/
	__RW uint32_t CSR;					/*RCC clock control & status register. Offset: 0x74*/
	__R  uint32_t RESERVED6[2];			/*Reserved, 0x78 ~ 0x80*/
	__RW uint32_t SSCGR;				/*RCC spread spectrum clock generation register. Offset: 0x80*/
	__RW uint32_t PLLI2SCFGR;			/*PLLI2S configuration register. Offset: 0x84*/
	__R  uint32_t RESERVED7[1];			/*Reserved, 0x88 ~ 0x8C*/
	__RW uint32_t DCKCFGR;				/*RCC Dedicated Clocks Configuration Register. Offset: 0x8C*/
} RCC_RegDef;

/*
 * Definition peripheral structure for the GPIO register
 * You use the given macros for accessing the AFR pins:
 * 	__GPIOA->AFR[LOW] = ...l
 */
typedef struct {
	__RW uint32_t MODER;				/*GPIO port mode register. Offset: 0x00*/
	__RW uint32_t OTYPER;				/*GPIO port output type register. Offset: 0x04*/
	__RW uint32_t OSPEEDR;				/*GPIO port output speed register. Offset: 0x08*/
	__RW uint32_t PUPDR;				/*port pull-up/pull-down register. Offset: 0x0C*/
	__RW uint32_t IDR;					/*port input data register. Offset: 0x10*/
	__RW uint32_t ODR;					/*port output data register. Offset: 0x14*/
	__RW uint32_t BSRR;					/*port bit set/reset register. Offset: 0x18*/
	__RW uint32_t LCKR;					/*port configuration lock register. Offset: 0x1C*/
	__RW uint32_t AFR[2];				/*GPIO alternate function: low set to 0, high set to 1. Offset: 0x20 ~ 0x28*/
} GPIO_RegDef;

/**
 * Definition for the peripheral structure for the SYSCFG register
 */
typedef struct {
	__RW uint32_t MEMRMP;				/*SYSCFG memory remap register (SYSCFG_MEMRMP). Offset: 0x00*/
	__RW uint32_t PMC;					/*SYSCFG peripheral mode configuration register (SYSCFG_PMC). Offset: 0x04*/
	__RW uint32_t EXTICR[4];			/*SYSCFG external interrupt configuration registers [1:4](SYSCFG_EXTICR1/2/3/4). Offset: 0x08 ~ 0x14*/
	__RW uint32_t CMPCR;				/*Compensation cell control register (SYSCFG_CMPCR). Address offset: 0x20*/
} SYSCFG_RegDef;

typedef struct {
	__RW uint32_t ISER[8U];				/* Interrupt Set-enable Registers: (0xE000E100-0xE000E11C)
											The NVIC_ISER0-NVIC_ISER7 registers enable interrupts,
											and show which interrupts are enabled. Each hierarchy
											number is represented by one of the bits defined in this
											register. Set "1" in write to mode to enable it. */
		 uint32_t RESERVED0[24U];
	__RW uint32_t ICER[8U];				/* Interrupt Clear-enable Registers: (0xE000E180-0xE000E19C)
	 	 	 	 	 	 	 	 	 	 	 The NVIC_ICER0-NVIC_ICER7 registers disable interrupts,
	 	 	 	 	 	 	 	 	 	 	 and show which interrupts are enabled. Each hierarchy
	 	 	 	 	 	 	 	 	 	 	 number is mapped to a bit to clear the IRQ in one
	 	 	 	 	 	 	 	 	 	 	 of the entries defined in the list. Set to one in Write
	 	 	 	 	 	 	 	 	 	 	 mode to disable it. */
		 uint32_t RSERVED1[24U];
	__RW uint32_t ISPR[8U];				/* Interrupt Set-pending Registers: (0XE000E200-0xE000E21C)
	 	 	 	 	 	 	 	 	 	 	 The NVIC_ISPR0-NVIC_ISPR7 registers force interrupts into
	 	 	 	 	 	 	 	 	 	 	 the pending state, and show which interrupts are pending.
	 	 	 	 	 	 	 	 	 	 	 Setting the hierarchy number bit to "1" sets that hierarchy to
	 	 	 	 	 	 	 	 	 	 	 pending. */
		 uint32_t RESERVED2[24U];
	__RW uint32_t ICPR[8U];				/* Interrupt Clear-pending Registers: (0XE000E280-0xE000E29C)
	 	 	 	 	 	 	 	 	 	 	 The NVIC_ICPR0-NCVIC_ICPR7 registers remove the pending
	 	 	 	 	 	 	 	 	 	 	 state from interrupts, and show which interrupts are pending.
	 	 	 	 	 	 	 	 	 	 	 Setting the hierarchy number bit to "1" clears that pending hierarchy.*/
		 uint32_t RESERVED3[24U];
	__RW uint32_t IABR[8U];				/* Interrupt Active Bit Registers: (0xE000E300-0xE000E31C)
	 	 	 	 	 	 	 	 	 	 	 The NVIC_IABR0-NVIC_IABR7 registers indicate which interrupts are active.
	 	 	 	 	 	 	 	 	 	 	 - "1" in the bit means the hierarchy number is active
	 	 	 	 	 	 	 	 	 	 	 - "0" in the bit means the hierarchy number is not active */
		 uint32_t RESERVED4[56U];
	__RW uint32_t IPR[240U];			/* Interrupt Priority Registers: (0xE000E400-0xE000E4EF)
	 	 	 	 	 	 	 	 	 	 	 The NVIC_IPR0-NVIC_IPR59 registers provide an 8-bit priority field for
	 	 	 	 	 	 	 	 	 	 	 each interrupt and each register holds four priority fields. These
	 	 	 	 	 	 	 	 	 	 	 registers are byte-accessible. */
		 uint32_t RESERVED5[644U];
	__RW uint32_t STIR;					/* Software Trigger Interrupt Register: (0xE000EF00)
	 	 	 	 	 	 	 	 	 	 	Write to the STIR to generate an interrupt from software.
	 	 	 	 	 	 	 	 	 	 	When the USERSETMPEND bit in the SCR is set to 1, unprivileged
	 	 	 	 	 	 	 	 	 	 	software can access the STIR. */
} NVIC_RegDef;

/**
 * @brief	- Structure type to retain the register map configuration for the
 * 			  SPI peripheral registers on STM32F401xx.
 */
typedef struct {
	__RW uint32_t	CR1;		/*!< SPI control register 1 (SPI_CR1)(not used in I2S mode). 			Address offset: 0x00 */
	__RW uint32_t	CR2;		/*!< SPI control register 2 (SPI_CR2). 									Address offset: 0x04 */
	__RW uint32_t	SR;			/*!< SPI status register (SPI_SR). 										Address offset: 0x08 */
	__RW uint32_t	DR;			/*!< SPI data register (SPI_DR). 										Address offset: 0x0C */
	__RW uint32_t	CRCPR;		/*!< SPI CRC polynomial register (SPI_CRCPR) - (not used in I2Smode). 	Address offset: 0x10 */
	__RW uint32_t	RXCRCR;		/*!< SPI RX CRC register (SPI_RXCRCR) - (not used in I2S mode). 		Address offset: 0x14 */
	__RW uint32_t	TXCRCR;		/*!< SPI TX CRC register (SPI_TXCRCR) - (not used in I2S mode). 		Address offset: 0x18 */
	__RW uint32_t	I2SCFGR;	/*!< SPI_I2S configuration register (SPI_I2SCFGR). 						Address offset: 0x1C */
	__RW uint32_t	I2SPR;		/*!< SPI_I2S prescaler register (SPI_I2SPR). 							Address offset: 0x20 */
} SPI_RegDef;


/***************************** BASE POINTER DEFINITIONS FOR THE PERIPHERALS ***********************************/
/*
 * Register pointer definitions:
 * - RCC, NVIC, SYSCFG, GPIOA/B/C/D/E/H, SPI1/2/3/4
 */
#define	__RCC		((RCC_RegDef*) RCC_BASEADDR)
#define	__NVIC		((NVIC_RegDef*) CORTEXM4_NVIC_BASEADDR)
#define __SYSCFG	((SYSCFG_RegDef*) SYSCFG_BASEADDR)

#define	__GPIOA		((GPIO_RegDef*) GPIOA_BASEADDR)
#define	__GPIOB		((GPIO_RegDef*) GPIOB_BASEADDR)
#define	__GPIOC		((GPIO_RegDef*) GPIOC_BASEADDR)
#define	__GPIOD		((GPIO_RegDef*) GPIOD_BASEADDR)
#define	__GPIOE		((GPIO_RegDef*) GPIOE_BASEADDR)
#define	__GPIOH		((GPIO_RegDef*) GPIOH_BASEADDR)

#define __SPI1		((SPI_RegDef*) SPI1_BASEADDR)
#define __SPI2I2S2	((SPI_RegDef*) SPI2_I2S2_BASEADDR)
#define __SPI3I2S3	((SPI_RegDef*) SPI3_I2S3_BASEADDR)
#define __SPI4		((SPI_RegDef*) SPI4_BASEADDR)

/*
 * GPIO PIN definition
 */
#define	GPIOx_PIN_00				0x0
#define	GPIOx_PIN_01				0x1
#define	GPIOx_PIN_02				0x2
#define	GPIOx_PIN_03				0x3
#define	GPIOx_PIN_04				0x4
#define	GPIOx_PIN_05				0x5
#define	GPIOx_PIN_06				0x6
#define	GPIOx_PIN_07				0x7
#define	GPIOx_PIN_08				0x8
#define	GPIOx_PIN_09				0x9
#define	GPIOx_PIN_10				0xA
#define	GPIOx_PIN_11				0xB
#define	GPIOx_PIN_12				0xC
#define	GPIOx_PIN_13				0xD
#define	GPIOx_PIN_14				0xE
#define	GPIOx_PIN_15				0xF

/*
 * De-pointer macro
 */
#define	PTR(x)		((*x))

#define	SYSCFG_MAINFLASH_MEMORY			0x0UL	/*00: Main Flash memory mapped at 0x0000 0000*/
#define	SYSCFG_SYSTEMFLASH_MEMORY		0x1UL	/*01: System Flash memory mapped at 0x0000 0000*/
#define	SYSCFG_EMBEDDEDSRAM_MEMORY		0x3UL	/*11: Embedded SRAM mapped at 0x0000 0000*/

/***************************** PERIPHERAL MACROS & CLOCKS ***********************************/
// Enable peripheral clock for the ports (AHB1): A/B/C/D/E/H
#define __GPIOA_PCLK_EN()			do { \
										PTR(__RCC).AHB1ENR |= (1<<0); \
									} while (0)
#define __GPIOB_PCLK_EN()			do { \
										PTR(__RCC).AHB1ENR |= (1<<1); \
									} while (0)
#define __GPIOC_PCLK_EN()			do { \
										PTR(__RCC).AHB1ENR |= (1<<2); \
									} while (0)
#define __GPIOD_PCLK_EN()			do { \
										PTR(__RCC).AHB1ENR |= (1<<3); \
									} while (0)
#define __GPIOE_PCLK_EN()			do { \
										PTR(__RCC).AHB1ENR |= (1<<4); \
									} while (0)
#define __GPIOH_PCLK_EN()			do { \
										PTR(__RCC).AHB1ENR |= (1<<7); \
									} while (0)
#define __CRC_PCLK_EN()				do { \
										PTR(__RCC).AHB1ENR |= (1<<12); \
									} while (0)
#define __DMA1_PCLK_EN()			do { \
										PTR(__RCC).AHB1ENR |= (1<<21); \
									} while (0)
#define __DMA2_PCLK_EN()			do { \
										PTR(__RCC).AHB1ENR |= (1<<22); \
									} while (0)
// Enable peripheral clock for the ports (AHB2)
#define __OTGFS_PCLK_EN()			do { \
										PTR(__RCC).AHB2ENR |= (1<<7); \
									} while (0)
// Enable peripheral clock for the ports (APB1): TIM2..5, WWDG, SPI2..3, USART2, I2C1..3, PWR
#define __TIM2_PCLK_EN()			do { \
										PTR(__RCC).APB1ENR |= (1<<0); \
									} while (0)
#define __TIM3_PCLK_EN()			do { \
										PTR(__RCC).APB1ENR |= (1<<1); \
									} while (0)
#define __TIM4_PCLK_EN()			do { \
										PTR(__RCC).APB1ENR |= (1<<2); \
									} while (0)
#define __TIM5_PCLK_EN()			do { \
										PTR(__RCC).APB1ENR |= (1<<3); \
									} while (0)
#define __WWDG_PCLK_EN()			do { \
										PTR(__RCC).APB1ENR |= (1<<11); \
									} while (0)
#define __SPI2_PCLK_EN()			do { \
										PTR(__RCC).APB1ENR |= (1<<14); \
									} while (0)
#define __SPI3_PCLK_EN()			do { \
										PTR(__RCC).APB1ENR |= (1<<15); \
									} while (0)
#define __USART2_PCLK_EN()			do { \
										PTR(__RCC).APB1ENR |= (1<<17); \
									} while (0)
#define __I2C1_PCLK_EN()			do { \
										PTR(__RCC).APB1ENR |= (1<<21); \
									} while (0)
#define __I2C2_PCLK_EN()			do { \
										PTR(__RCC).APB1ENR |= (1<<22); \
									} while (0)
#define __I2C3_PCLK_EN()			do { \
										PTR(__RCC).APB1ENR |= (1<<23); \
									} while (0)
#define __PWR_PCLK_EN()			do { \
									PTR(__RCC).APB1ENR |= (1<<28); \
								} while (0)
// Enable peripheral clock for the ports (APB2): TIM1, USART1/6, ADC1, SDIO, SPI1, SPI4EN, SYSCFG, TIM9..11
#define __TIM1_PCLK_EN()			do { \
										PTR(__RCC).APB2ENR |= (1<<1); \
									} while (0)
#define __USART1_PCLK_EN()			do { \
										PTR(__RCC).APB2ENR |= (1<<4); \
									} while (0)
#define __USART6_PCLK_EN()			do { \
										PTR(__RCC).APB2ENR |= (1<<5); \
									} while (0)
#define __SDIO_PCLK_EN()			do { \
										PTR(__RCC).APB2ENR |= (1<<11); \
									} while (0)
#define __SPI1_PCLK_EN()			do { \
										PTR(__RCC).APB2ENR |= (1<<12); \
									} while (0)
#define __SPI4_PCLK_EN()			do { \
										PTR(__RCC).APB2ENR |= (1<<13); \
									} while (0)
#define __SYSCFG_PCLK_EN()			do { \
										PTR(__RCC).APB2ENR |= (1<<14); \
									} while (0)
#define __TIM9_PCLK_EN()			do { \
										PTR(__RCC).APB2ENR |= (1<<16); \
									} while (0)
#define __TIM10_PCLK_EN()			do { \
										PTR(__RCC).APB2ENR |= (1<<17); \
									} while (0)
#define __TIM11_PCLK_EN()			do { \
										PTR(__RCC).APB2ENR |= (1<<18); \
									} while (0)

/************************************ DISABLE MACROS ***********************************/
// Disable peripheral clock for the ports (AHB1): A/B/C/D/E/H
#define __GPIOA_PCLK_DIS()			do { \
										PTR(__RCC).AHB1ENR &= ~(1<<0); \
									} while (0)
#define __GPIOB_PCLK_DIS()			do { \
										PTR(__RCC).AHB1ENR &= ~(1<<1); \
									} while (0)
#define __GPIOC_PCLK_DIS()			do { \
										PTR(__RCC).AHB1ENR &= ~(1<<2); \
									} while (0)
#define __GPIOD_PCLK_DIS()			do { \
										PTR(__RCC).AHB1ENR &= ~(1<<3); \
									} while (0)
#define __GPIOE_PCLK_DIS()			do { \
										PTR(__RCC).AHB1ENR &= ~(1<<4); \
									} while (0)
#define __GPIOH_PCLK_DIS()			do { \
										PTR(__RCC).AHB1ENR &= ~(1<<7); \
									} while (0)
#define __CRC_PCLK_DIS()			do { \
										PTR(__RCC).AHB1ENR &= ~(1<<12); \
									} while (0)
#define __DMA1_PCLK_DIS()			do { \
										PTR(__RCC).AHB1ENR &= ~(1<<21); \
									} while (0)
#define __DMA2_PCLK_DIS()			do { \
										PTR(__RCC).AHB1ENR &= ~(1<<22); \
									} while (0)
// Disable peripheral clock for the ports (AHB2)
#define __OTGFS_PCLK_DIS()			do { \
										PTR(__RCC).AHB2ENR &= ~(1<<7); \
									} while (0)
// Disable peripheral clock for the ports (APB1): TIM2..5, WWDG, SPI2..3, USART2, I2C1..3, PWR
#define __TIM2_PCLK_DIS()			do { \
										PTR(__RCC).APB1ENR &= ~(1<<0); \
									} while (0)
#define __TIM3_PCLK_DIS()			do { \
										PTR(__RCC).APB1ENR &= ~(1<<1); \
									} while (0)
#define __TIM4_PCLK_DIS()			do { \
										PTR(__RCC).APB1ENR &= ~(1<<2); \
									} while (0)
#define __TIM5_PCLK_DIS()			do { \
										PTR(__RCC).APB1ENR &= ~(1<<3); \
									} while (0)
#define __WWDG_PCLK_DIS()			do { \
										PTR(__RCC).APB1ENR &= ~(1<<11); \
									} while (0)
#define __SPI2_PCLK_DIS()			do { \
										PTR(__RCC).APB1ENR &= ~(1<<14); \
									} while (0)
#define __SPI3_PCLK_DIS()			do { \
										PTR(__RCC).APB1ENR &= ~(1<<15); \
									} while (0)
#define __USART2_PCLK_DIS()			do { \
										PTR(__RCC).APB1ENR &= ~(1<<17); \
									} while (0)
#define __I2C1_PCLK_DIS()			do { \
										PTR(__RCC).APB1ENR &= ~(1<<21); \
									} while (0)
#define __I2C2_PCLK_DIS()			do { \
										PTR(__RCC).APB1ENR &= ~(1<<22); \
									} while (0)
#define __I2C3_PCLK_DIS()			do { \
										PTR(__RCC).APB1ENR &= ~(1<<23); \
									} while (0)
#define __PWR_PCLK_DIS()			do { \
									PTR(__RCC).APB1ENR &= ~(1<<28); \
								} while (0)
// Disable peripheral clock for the ports (APB2): TIM1, USART1/6, ADC1, SDIO, SPI1, SPI4EN, SYSCFG, TIM9..11
#define __TIM1_PCLK_DIS()			do { \
										PTR(__RCC).APB2ENR &= ~(1<<1); \
									} while (0)
#define __USART1_PCLK_DIS()			do { \
										PTR(__RCC).APB2ENR &= ~(1<<4); \
									} while (0)
#define __USART6_PCLK_DIS()			do { \
										PTR(__RCC).APB2ENR &= ~(1<<5); \
									} while (0)
#define __ADC1_PCLK_DIS()			do { \
										PTR(__RCC).APB2ENR &= ~(1<<8); \
									} while (0)
#define __SDIO_PCLK_DIS()			do { \
										PTR(__RCC).APB2ENR &= ~(1<<11); \
									} while (0)
#define __SPI1_PCLK_DIS()			do { \
										PTR(__RCC).APB2ENR &= ~(1<<12); \
									} while (0)
#define __SPI4_PCLK_DIS()			do { \
										PTR(__RCC).APB2ENR &= ~(1<<13); \
									} while (0)
#define __SYSCFG_PCLK_DIS()			do { \
										PTR(__RCC).APB2ENR &= ~(1<<14); \
									} while (0)
#define __TIM9_PCLK_DIS()			do { \
										PTR(__RCC).APB2ENR &= ~(1<<16); \
									} while (0)
#define __TIM10_PCLK_DIS()			do { \
										PTR(__RCC).APB2ENR &= ~(1<<17); \
									} while (0)
#define __TIM11_PCLK_DIS()			do { \
										PTR(__RCC).APB2ENR &= ~(1<<18); \
									} while (0)

/************************************ RESET MACROS ***********************************/
// Reset the GPIO for ports: A/B/C/D/E/H
#define __GPIOA_PCLK_RESET()		do { \
										PTR(__RCC).AHB1RSTR |=  (1<<0); \
										PTR(__RCC).AHB1RSTR &= ~(1<<0); \
									} while (0)
#define __GPIOB_PCLK_RESET()		do { \
										PTR(__RCC).AHB1RSTR |=  (1<<1); \
										PTR(__RCC).AHB1RSTR &= ~(1<<1); \
									} while (0)
#define __GPIOC_PCLK_RESET()		do { \
										PTR(__RCC).AHB1RSTR |=  (1<<2); \
										PTR(__RCC).AHB1RSTR &= ~(1<<2); \
									} while (0)
#define __GPIOD_PCLK_RESET()		do { \
										PTR(__RCC).AHB1RSTR |=  (1<<3); \
										PTR(__RCC).AHB1RSTR &= ~(1<<3); \
									} while (0)
#define __GPIOE_PCLK_RESET()		do { \
										PTR(__RCC).AHB1RSTR |=  (1<<4); \
										PTR(__RCC).AHB1RSTR &= ~(1<<4); \
									} while (0)
#define __GPIOH_PCLK_RESET()		do { \
										PTR(__RCC).AHB1RSTR |=  (1<<7); \
										PTR(__RCC).AHB1RSTR &= ~(1<<7); \
									} while (0)

// Reset SPIs 1/2/3/4
#define	__SPI1_PCLK_RESET()			do { \
										PTR(__RCC).APB2RSTR |=  (1<<12); \
										PTR(__RCC).APB2RSTR &= ~(1<<12); \
									} while (0)
#define	__SPI2_PCLK_RESET()			do { \
										PTR(__RCC).APB1RSTR |=  (1<<14); \
										PTR(__RCC).APB1RSTR &= ~(1<<14); \
									} while (0)
#define	__SPI3_PCLK_RESET()			do { \
										PTR(__RCC).APB1RSTR |=  (1<<15); \
										PTR(__RCC).APB1RSTR &= ~(1<<15); \
									} while (0)
#define	__SPI4_PCLK_RESET()			do { \
										PTR(__RCC).APB2RSTR |=  (1<<13); \
										PTR(__RCC).APB2RSTR &= ~(1<<13); \
									} while (0)

/**
 * @brief	- Selects the IRQ hierarchy number based on the PIN configuration
 */
#define	__IRQNUM_FROM_PIN(n)		( n == GPIOx_PIN_00 ? IRQ_HIERARCHY_EXTI0 : \
									( n == GPIOx_PIN_01 ? IRQ_HIERARCHY_EXTI1 : \
									( n == GPIOx_PIN_02 ? IRQ_HIERARCHY_EXTI2 : \
									( n == GPIOx_PIN_03 ? IRQ_HIERARCHY_EXTI3 : \
									( n == GPIOx_PIN_04 ? IRQ_HIERARCHY_EXTI4 : \
									( n >= GPIOx_PIN_05 && n < GPIOx_PIN_10 ? IRQ_HIERARCHY_EXTI9_5 : \
									( n >= GPIOx_PIN_10 && n <= GPIOx_PIN_15 ? IRQ_HIERARCHY_EXTI15_10 : 0x0/*this is an exception!!!*/) ))))))


/***************************** MAIN FUNCTION PROTOTYPES ***********************************/
__RW uint32_t* GPIOGetAFRLow(GPIO_RegDef*);
__RW uint32_t* GPIOGetAFRHigh(GPIO_RegDef*);
/*
 * Configure the interrupts
 */
void __irq_config(uint8_t IRQNumber, uint32_t IRQPriority, uint8_t IRQEnableDisable, irq_generic_handler_fn_t fn);
void __irq_handle(uint8_t pinNumber);

#ifdef __cplusplus
}
#endif

#endif /* DRIVERS_INC_STM32F401XX_H_ */
