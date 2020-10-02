/*
 * stm32f401xx_gpio.driver.c
 *
 *  Created on: Sep 21, 2020
 *      Author: jgonzalezlopes
 */

#include "stm32f401xx_gpio.driver.h"

/**************************************************************************
 * @fn			- __gpio_pclk_control
 * @brief		- GPIO peripheral control clock (enable/disable)
 *
 * @param[in]	- base address of the GPIO Address
 * @param[in]	- ENABLE or DISABLE the macros
 *
 * @return		- none
 * @Note		- none
 */
void __gpio_pclk_control(GPIO_ConfigDef* pGpioCfgDef, uint8_t gpioEnDi) {
	// Enable the PIN
	if (gpioEnDi & ENABLE) {
		if (__GPIOA == PTR(pGpioCfgDef).pGPIOx) __GPIOA_PCLK_EN();
		else if (__GPIOB == PTR(pGpioCfgDef).pGPIOx) __GPIOB_PCLK_EN();
		else if (__GPIOC == PTR(pGpioCfgDef).pGPIOx) __GPIOC_PCLK_EN();
		else if (__GPIOD == PTR(pGpioCfgDef).pGPIOx) __GPIOD_PCLK_EN();
		else if (__GPIOE == PTR(pGpioCfgDef).pGPIOx) __GPIOE_PCLK_EN();
		else if (__GPIOH == PTR(pGpioCfgDef).pGPIOx) __GPIOH_PCLK_EN();
	}
	else {
		if (__GPIOA == PTR(pGpioCfgDef).pGPIOx) __GPIOA_PCLK_DIS();
		else if (__GPIOB == PTR(pGpioCfgDef).pGPIOx) __GPIOB_PCLK_DIS();
		else if (__GPIOC == PTR(pGpioCfgDef).pGPIOx) __GPIOC_PCLK_DIS();
		else if (__GPIOD == PTR(pGpioCfgDef).pGPIOx) __GPIOD_PCLK_DIS();
		else if (__GPIOE == PTR(pGpioCfgDef).pGPIOx) __GPIOE_PCLK_DIS();
		else if (__GPIOH == PTR(pGpioCfgDef).pGPIOx) __GPIOH_PCLK_DIS();
	}
}

/**************************************************************************
 * @fn			- __aux_gpio_init_pin
 * @brief		- GPIO private function to initialize every GPIO pin with a default PIN configuration
 *
 * @param[in]	- the pin definition to be used
 * @param[in]	- the pin configuration to be initialized
 *
 * @return		- none
 * @Note		- private function
 */
void __gpio_init_pin (GPIO_ConfigDef *pCfgDef, GPIO_PinConfigDef pinConfig) {
	// Registers that takes two bits /////////////////////////////////////////////////
	uint8_t bitnum = pinConfig.PinNumber * 0x2UL;
	PTR(pCfgDef->pGPIOx).MODER 	&= ~(0x3 << bitnum);
	PTR(pCfgDef->pGPIOx).MODER 	|= (pinConfig.PinMode << bitnum);

	PTR(pCfgDef->pGPIOx).OSPEEDR &= ~(0x3 << bitnum);
	PTR(pCfgDef->pGPIOx).OSPEEDR |=  (pinConfig.PinSpeed << bitnum); // Low speed for all the pins

	PTR(pCfgDef->pGPIOx).PUPDR &= ~(0x3 << bitnum);
	PTR(pCfgDef->pGPIOx).PUPDR |=  (pinConfig.PinPullUpPullDownCtrl << bitnum); // Low speed for all the pins
	// Registers that takes two bits /////////////////////////////////////////////////

	PTR(pCfgDef->pGPIOx).OTYPER	&= ~(0x1 << pinConfig.PinNumber);
	PTR(pCfgDef->pGPIOx).OTYPER	|= (pinConfig.PinOType << pinConfig.PinNumber);	/// Will leave the default value for push pull to all the pins

	/// Alternate function support & configuration
	if ( pinConfig.PinMode == GPIOx_MODE_ALTFN ) {
		bitnum = (pinConfig.PinNumber % 8) * 0x4UL;

		/// Selects the alternating function based on configuration: AF0(Low) ~ AF15(High)
		if ( pinConfig.PinNumber > GPIOx_PIN_07 ) { /// For AFRH
			PTR(pCfgDef->pGPIOx).AFR[HIGH] &= ~(0x7 << bitnum);
			PTR(pCfgDef->pGPIOx).AFR[HIGH] |=  (pinConfig.PinAltFuncMode << bitnum);
		}
		else { /// For AFRL
			PTR(pCfgDef->pGPIOx).AFR[LOW] &= ~(0x7 << bitnum);
			PTR(pCfgDef->pGPIOx).AFR[LOW] |=  (pinConfig.PinAltFuncMode << bitnum);
		}
	}

	/// Specific mode configuration
	if (pinConfig.PinMode <= 0x3) { // Non iterrupt mode
		// TODO:
	}
	/// Interrupt mode
	else {
		/*
		 * PERIPHERAL SIDE CONFIGURATION
		 */
		uint8_t	idx 	= pinConfig.PinNumber / 4UL,
				basepos = (pinConfig.PinNumber % 4UL) * 0x4UL,
				portcfg = __GET_CR_PORT(PTR(pCfgDef).pGPIOx);

		__SYSCFG_PCLK_EN(); /// Enable the SYSCFG clock on APB2

		/// 1. Configure the triggering point (FE, RT, or both) - when is going to happen
		if ( pinConfig.PinMode & GPIOx_MODE_IN_FALLEDGETRIG ) {
			PTR(__EXTI).FTSR |= ( 1 << pinConfig.PinNumber ); // set!
			/// Clear the corresponding RTSR bit (only one should be enable)
			PTR(__EXTI).RTSR &= ~( 1 << pinConfig.PinNumber );
		}
		else if ( pinConfig.PinMode & GPIOx_MODE_IN_RAISEEDGETRIG ) {
			PTR(__EXTI).RTSR |= ( 1 << pinConfig.PinNumber ); // set!
			/// Clear the corresponding FTSR bit (only one should be enable)
			PTR(__EXTI).FTSR &= ~( 1 << pinConfig.PinNumber );
		}
		else if ( pinConfig.PinMode & GPIOx_MODE_IN_FREDGETRIG ) {
			PTR(__EXTI).RTSR |= ( 1 << pinConfig.PinNumber ); // set!
			PTR(__EXTI).FTSR |= ( 1 << pinConfig.PinNumber ); // set!
		}

		/// 2. Configure the GPIO port selection in SYSCFG_EXTI
		PTR(__SYSCFG).EXTICR[ idx ] |= ( portcfg << basepos );

		/// 3. Enable the EXTI interrupt using IMR (Interrupt mask register)
		PTR(__EXTI).IMR |= ( 1 << pinConfig.PinNumber );

		/*
		 * Now, we have to initialize the IRQ handling...
		 */
		__irq_config(__IRQNUM_FROM_PIN(pinConfig.PinNumber),
					 NVIC_IRQ_PRIORITY_15 /*Default priority value: 0b 1111*/,
					 ENABLE, pinConfig.IRQFN);
	}

	// Save the PIN configuration
	PTR(pCfgDef).GPIOxPinConfig[ pinConfig.PinNumber ] = pinConfig;
}

/**************************************************************************
 * @fn			- __gpio_init
 * @brief		- GPIO peripheral port initialization
 *
 * @param[in]	- base address of the GPIOx port
 * @param[in]	- "0" to not initialize all the pins, "1" otherwise
 *
 * @return		- the new GPIO configuration definition
 * @Note		- none
 */
GPIO_ConfigDef* __gpio_init(GPIO_RegDef	*pGpioCfgDef, uint8_t initAllPins) {
	int8_t count, len=(GPIOx_PIN_15+0x1UL);
	GPIO_ConfigDef *pCfgDef = (GPIO_ConfigDef*) malloc(sizeof(GPIO_ConfigDef));

	memset((void*) pCfgDef, 0, sizeof(GPIO_ConfigDef));
	PTR(pCfgDef).pGPIOx = pGpioCfgDef;

	// Enable the clock for the AHB1
	__gpio_pclk_control(pCfgDef, ENABLE);

	/// Pin number initialization
	for (count=0;count < len; count++) {
		PTR(pCfgDef).GPIOxPinConfig[ count ].PinNumber = count;

		memset ( PTR(pCfgDef).GPIOxPinConfig[ count ].IRQFN, 0, sizeof(irq_generic_handler_fn_t) );/*no handlers for the IRQ by default*/
//		PTR(pCfgDef).GPIOxPinConfig[ count ].IRQFN = 0x0; /*no handlers for the IRQ by default*/
	}

	if (initAllPins & SET) {
		/// Lets custom initialize all the pins in the configuration
		for (count=0;count < len; count++) {
			PTR(pCfgDef).GPIOxPinConfig[ count ].PinAltFuncMode = 0x00; // AF0, no actually configuration (set to system by default)
//			PTR(pCfgDef).GPIOxPinConfig[ count ].PinMode = mode;
			PTR(pCfgDef).GPIOxPinConfig[ count ].PinNumber = count;
			PTR(pCfgDef).GPIOxPinConfig[ count ].PinOType = GPIO_OTYPE_PUSHPULL;	// Will leave the default value for push pull to all the pins
			PTR(pCfgDef).GPIOxPinConfig[ count ].PinPullUpPullDownCtrl = GPIO_PUPDR_PU; // Internal push-up resistor
			PTR(pCfgDef).GPIOxPinConfig[ count ].PinSpeed = GPIO_OSPEEDR_LOW; // Low speed for all the pins

			// Normal port configuration on the buffer
			__gpio_init_pin(pCfgDef, PTR(pCfgDef).GPIOxPinConfig[ count ]);
		}
	}

	return pCfgDef;
}

/**************************************************************************
 * @fn			- __gpio_deinit
 * @brief		- GPIO peripheral port deinitialization/reset. After calling this function,
 * 				  the GPIO will have to re-initialized again
 *
 * @param[in]	- base address of the GPIOx port
 *
 * @return		- none
 * @Note		- none
 */
void __gpio_deinit(GPIO_ConfigDef* pGpioCfgDef) {
	// Disable GPIO
	if (__GPIOA == PTR(pGpioCfgDef).pGPIOx) __GPIOA_PCLK_RESET();
	else if (__GPIOB == PTR(pGpioCfgDef).pGPIOx) __GPIOB_PCLK_RESET();
	else if (__GPIOC == PTR(pGpioCfgDef).pGPIOx) __GPIOC_PCLK_RESET();
	else if (__GPIOD == PTR(pGpioCfgDef).pGPIOx) __GPIOD_PCLK_RESET();
	else if (__GPIOE == PTR(pGpioCfgDef).pGPIOx) __GPIOE_PCLK_RESET();
	else if (__GPIOH == PTR(pGpioCfgDef).pGPIOx) __GPIOH_PCLK_RESET();

	/// Release the memory block for the allocated amount
	free ((void*) pGpioCfgDef);
}

/**************************************************************************
 * @fn			- __gpio_read_pin
 * @brief		- read one bit from the given pin and port passed as parameter
 *
 * @param[in]	- the pin configuration to be used for reading the digital signal
 * @param[in]	- the currently defined GPIO configuration
 *
 * @return		- the bit read from the PIN: "0" or "1"
 * @Note		- none
 */
uint8_t __gpio_read_pin(GPIO_PinConfigDef pinConfig, GPIO_ConfigDef *pGpioCfgDef) {
	uint8_t val;

	// Read just one bit from the port in one MCU cycle
	/// It will always going to be "0" or "1"
	val = (uint8_t) ( ( PTR(pGpioCfgDef->pGPIOx).IDR >> pinConfig.PinNumber ) & 0x00000001 ); // we only care about the less significant bit

	return val;
}

/**************************************************************************
 * @fn			- __gpio_read_port
 * @brief		- Read the entire 16-bit pins from the given port
 *
 * @param[in]	- the currently defined GPIO configuration
 *
 * @return		- the 16-bit pins from the given peripheral port
 * @Note		- none
 */
uint16_t __gpio_read_port(GPIO_ConfigDef *pGpioCfgDef) {
	uint16_t val;

	// Read just one bit from the port in one MCU cycle
	/// It will always going to be "0" or "1"
	val = (uint16_t) PTR(pGpioCfgDef->pGPIOx).IDR;

	return val;
}

/**************************************************************************
 * @fn			- __gpio_write_pin
 * @brief		- Write the given bit ("0" or "1") to given GPIO port passed as parameter. Example:
 * 					<code>
 * 						__gpio_write_pin(SET, pinConfig, pGpioCfg); /// writes "1" to the pin
 * 						...
 * 						__gpio_write_pin(RESET, pinConfig, pGpioCfg); /// writes "0" to the pin
 * 					</code>
 *
 * @param[in]	- the bit to be written: either "0" or "1" - <em>other values are not allowed</em>
 * @param[in]	- the PIN configuration and definition
 * @param[in]	- the GPIO peripheral configuration
 *
 * @return		- none
 * @Note		- none
 */
void __gpio_write_pin(uint8_t data, GPIO_PinConfigDef pinConfig, GPIO_ConfigDef *pGpioCfgDef) {
	if (data & SET) { /// Writes "1"
		PTR(pGpioCfgDef->pGPIOx).ODR |= ( 0x1 << pinConfig.PinNumber );
	}
	else { /// Writes "0"
		PTR(pGpioCfgDef->pGPIOx).ODR &= ~( 0x1 << pinConfig.PinNumber );
	}
}

/**************************************************************************
 * @fn			- __gpio_write_port
 * @brief		- Write the given bit ("0" or "1") to given GPIO port passed as parameter. Example:
 * 					<code>
 * 						__gpio_write_pin(SET, pinConfig, pGpioCfg); /// writes "1" to the pin
 * 						...
 * 						__gpio_write_pin(RESET, pinConfig, pGpioCfg); /// writes "0" to the pin
 * 					</code>
 *
 * @param[in]	- the bit to be written: either "0" or "1" - <em>other values are not allowed</em>
 * @param[in]	- the PIN configuration and definition
 * @param[in]	- the GPIO peripheral configuration
 *
 * @return		- none
 * @Note		- none
 */
void __gpio_write_port(uint16_t bsrr, GPIO_ConfigDef *pGpioCfgDef) {
	/// Write all the bits to the ODR register (aka to the port ;)
	PTR(pGpioCfgDef->pGPIOx).ODR = bsrr;
}

/**************************************************************************
 * @fn			- __gpio_toggle_pin
 * @brief		- Toggle the bit from the given pin configuration passed as parameter
 *
 * @param[in]	- the pin configuration & definition
 * @param[in]	- the GPIO peripheral configuration
 *
 * @return		- none
 * @Note		- none
 */
void __gpio_toggle_pin(GPIO_PinConfigDef pinConfig, GPIO_ConfigDef *pGpioCfgDef) {
	PTR(pGpioCfgDef->pGPIOx).ODR ^= ( 0x1 << pinConfig.PinNumber );
}

