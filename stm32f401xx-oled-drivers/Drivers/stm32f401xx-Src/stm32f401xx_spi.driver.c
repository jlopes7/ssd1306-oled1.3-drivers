/*
 * stm32f401xx_spi.driver.c
 *
 *  Created on: Sep 27, 2020
 *      Author: jgonzalezlopes
 */

#include "stm32f401xx_spi.driver.h"


/*************** Local Variables a function prototypes *************************/
static SPI_ConfigDef	__asyncSPIConfigDef[4]; /*one for each SPI operation*/

static void SPIx_Handler (SPI_RegDef* pSPI);/*private internal function*/

static void __async_internal_send(SPI_ConfigDef *pSPICfg);
static void __async_internal_receive(SPI_ConfigDef *pSPICfg);
static void __async_internal_errie(SPI_ConfigDef *pSPICfg, uint8_t errieCode);

/***************  Local/Internal Macros and constants  *************************/
#define __GET_ASYNC_SPICONFIG(pSPI)		( pSPI == __SPI1 	 ? &__asyncSPIConfigDef[0x0] :\
										( pSPI == __SPI2I2S2 ? &__asyncSPIConfigDef[0x1] :\
										( pSPI == __SPI3I2S3 ? &__asyncSPIConfigDef[0x2] :\
										( pSPI == __SPI4     ? &__asyncSPIConfigDef[0x3] : &__asyncSPIConfigDef[0x0]/*this is a problem and should not work*/ ))) )

/*******************************************************************************
 * @fn			- __spi_pclk_control
 * @brief		- Initialize the clock control for the given SPI configuration definition
 *
 * @param[in]	- The SPI register definition
 * @param[in]	- If it should enable or disable the clock control for the given SPI peripheral
 */
void __spi_pclk_control(SPI_RegDef* pSPICfgDef, uint8_t spiEnDi) {
	if ( spiEnDi & ENABLE ) {
		if ( pSPICfgDef == __SPI1 ) __SPI1_PCLK_EN();
		else if ( pSPICfgDef == __SPI2I2S2 ) __SPI2_PCLK_EN();
		else if ( pSPICfgDef == __SPI3I2S3 ) __SPI3_PCLK_EN();
		else if ( pSPICfgDef == __SPI4 ) __SPI4_PCLK_EN();
	}
	else {
		if ( pSPICfgDef == __SPI1 ) __SPI1_PCLK_DIS();
		else if ( pSPICfgDef == __SPI2I2S2 ) __SPI2_PCLK_DIS();
		else if ( pSPICfgDef == __SPI3I2S3 ) __SPI3_PCLK_DIS();
		else if ( pSPICfgDef == __SPI4 ) __SPI4_PCLK_DIS();
	}
}

/******************************************************************************
 * @fn			- __spi_init
 * @brief		- Initialize the SPI peripheral
 *
 * @param[in]	- the MCU internal register structure to be initialized
 * @param[in]	- the peripheral definition and configuration to be used
 * @param[in]	- identifies if the initialization should use a new configuration created from memory or shared
 * 				  version for each SPI peripheral. Available constants are: \SPI_ASYNC (0x1) or \SPI_NORMAL (0x2)
 *
 * @return		- a newly created configuration handler for the SPI peripheral
 * @Note		- none
 */
SPI_ConfigDef*__spi_init (SPI_RegDef *pSPIx, SPI_PeripheralDef *pPDef, uint8_t operationType) {
	SPI_ConfigDef *pSPICfg;

	/// Verifies if it's going to use the async configuration or the SRAM based SPI buffer
	if ( operationType & SPI_ASYNC ) {
		pSPICfg = __GET_ASYNC_SPICONFIG(pSPIx);
	}
	else {
		pSPICfg = malloc(sizeof(SPI_ConfigDef));
	}

	__spi_pclk_control(pSPIx, ENABLE);

	memset(pSPICfg, 0, sizeof(SPI_ConfigDef));/// Reset all the values...
	PTR(pSPICfg).pSPIx = pSPIx;

	/// Let's configure the device mode (using the CR1 register):  master or slave
	PTR(pSPIx).CR1 |= PTR(pPDef).DeviceMode;

	/// Configuration of the BUS mode of the SPI (still using the CR1 register):
	//// 1st. we clear the configuration based on the conditions
	if ( (PTR(pPDef).BusConfig & SPI_BUS_CONFIG_FD) || (PTR(pPDef).BusConfig & SPI_BUS_CONFIG_SIMPLEX_RXONLY)) {
		PTR(pSPIx).CR1 &= ~(1<<15); // 1: 1-line bidirectional data mode selected needs to be reseted to "0" (2-line unidirectional data mode selected: full duplex or simplex)
	}
	//// 2nd. we set the new bus configuration
	PTR(pSPIx).CR1 |= PTR(pPDef).BusConfig;

	//// 3rd. define the clock configuration
	PTR(pSPIx).CR1 &= ~(0x38U); // 111 & 000: reset all the values
	PTR(pSPIx).CR1 |=  PTR(pPDef).SClkSpeed;

	/// 4th. Specifies SSM to be software enabled
	PTR(pSPIx).CR1 &= ~(SPI_CR1_SWSLAVEMGMT_EN);
	PTR(pSPIx).CR1 |=  PTR(pPDef).SSM;

	/// 5th. define the Data frame format
	PTR(pSPIx).CR1 &= ~(SPI_CR1_DFF_16BITS);
	PTR(pSPIx).CR1 |=  PTR(pPDef).DataFrame;

	/// 6th. Define the polarity and phase of the clock for the SPI peripheral
	PTR(pSPIx).CR1 &= ~(SPI_CR1_PHASE_2NDCLKDATACATCH | SPI_CR1_POLARITY_HIGHONIDLE); // Reset is: 11 & 00
	PTR(pSPIx).CR1 |= (PTR(pPDef).CPhase | PTR(pPDef).CPolarity);

	/// 7th. SSI enabled - this may make the NSS signal internally high and avoid MODF error
	if ( PTR(pPDef).SSIEnabled ) {
		__SPI_SSI_CONFIG(pSPICfg, ENABLE);
	}

	/// 8th. Verify if it's going to run in a multimaster configuration
	if ( PTR(pPDef).SSOEEnabled ) {
		__SPI_SSOE_CONFIG(pSPICfg, ENABLE);
	}

	/// 9th. MSB or LSB selection
	if ( PTR(pPDef).LSBFirstEnabled ) {
		__SPI_SSOE_CONFIG(pSPICfg, ENABLE);
	}

	/// 10th. Sets the CRC polynomial
	if ( PTR(pPDef).CRCPolynomial ) {
		PTR(pSPIx).CRCPR = PTR(pPDef).CRCPolynomial;
	}

	PTR(pSPICfg).spiBasePeripheral = PTR(pPDef);

	/// return the newly created and configured SPI handler structure
	return pSPICfg;
}

/*******************************************************************************
 * @fn			- __spi_deinit
 * @brief		- De-initialize/reset the SPI peripheral
 *
 * @param[in] 	- the SPI peripheral configuration
 *
 * @return		- none
 * @Note		- none
 */
void __spi_deinit(SPI_ConfigDef *pSPIConfig) {
	if ( pSPIConfig->pSPIx == __SPI1 ) __SPI1_PCLK_RESET();
	else if ( pSPIConfig->pSPIx == __SPI2I2S2 ) __SPI2_PCLK_RESET();
	else if ( pSPIConfig->pSPIx == __SPI3I2S3 ) __SPI3_PCLK_RESET();
	else if ( pSPIConfig->pSPIx == __SPI4 ) __SPI4_PCLK_RESET();

	__SPI_PERIPHERAL_DISABLE(pSPIConfig);

	if ( pSPIConfig != __GET_ASYNC_SPICONFIG(PTR(pSPIConfig).pSPIx) ) {
		free((void*) pSPIConfig);
	}
}

/*******************************************************************************
 * @fn			- __spi_send
 * @brief		- Send the data buffer passed as parameter. It sends all the data passed as parameter
 *
 * @param[in] 	- the data to be written to the TX buffer
 * @param[in]	- the SPI configuration handle to be used
 *
 * @return		- none
 * @Note		- this function will only return when all the data is transmitted by the bit shift register (when "len" = 0). Blocking call
 */
void __spi_send	(uint8_t *pTxBuffer, SPI_ConfigDef *pSPIConfig) {
	uint8_t dff16BitsDef = PTR(pSPIConfig).spiBasePeripheral.DataFrame & SPI_CR1_DFF_16BITS; // Configured data frame format (DFF)
	uint32_t count = 0, len = (sizeof(pTxBuffer) / sizeof(uint8_t));// Length of the bytes to be transmitted

	/// We need to enable the SPI peripheral before actually start using it
	/// This needs to be done as the last part because not all the configuration
	///  was defined yet. Basically we set Bit 6 to "1" in the CR1 register.
	__SPI_PERIPHERAL_ENABLE(pSPIConfig);
	while ( len > count ) {
		uint8_t is_count_mod_2 = (count % 2) == 0; // positive counter number (this is only used for 16-bit communication)
		uint16_t data;

		/*
		 * we need to do some calculation for a 16 bits transaction
		 */
		if (len == 1) {
			data = PTR(pTxBuffer);
			count++;
		}
		else {
			if ( dff16BitsDef && is_count_mod_2 && (count+1) < len ) {
				uint8_t next_count = ++count;
				data = /*LSB*/pTxBuffer[ count ] | /*shift to MSB==>*/(((uint16_t)pTxBuffer[ next_count ]) << 8);

				count++;/// Process two bytes at a time
			}
			else {
				data = pTxBuffer[ count++ ] & 0x00FF; // Only LMB (first 8-bits) are filled in the register
			}
		}

		if ( is_count_mod_2 || !dff16BitsDef ) {// || /*last odd element*/ (((count+1) % 2) == 0 && (count+1) == len)
			/// Wait until data is transmitted
			__SPI_WaitTXRX_EmptyBuffer(PTR(pSPIConfig).pSPIx, SPI_TRANSMITTER_FLAG);

			PTR(pSPIConfig).pSPIx->DR = data;

			// Wait until SPI is not busy
			__SPI_Wait_Busy(PTR(pSPIConfig).pSPIx);
		}
	}
	/// We need to disable the SPI peripheral after using it.
	/// This needs to be done because the channels need to be put to high after
	///  that finish transmitting
	__SPI_PERIPHERAL_DISABLE(pSPIConfig);
}

/*******************************************************************************
 * @fn			- __spi_send_partial
 * @brief		- Send the data buffer passed as parameter. It sends all the number of bytes/halfwords defined by the parameter "len"
 *
 * @param[in] 	- the data to be written to the TX buffer
 * @param[in]	- the initial byte position to read from (for half words this amount is multiplied by 2)
 * @param[in]	- the length of data in bytes to be sent in the stream (for half words this amount is multiplied by 2)
 * @param[in]	- the SPI configuration handle to be used
 * @param[in]	- flag to identify if the SPI communication channel needs to be initiated
 * @param[in]	- flag to identify if the SPI communication channel needs to be finished
 *
 * @return		- the number of bytes remaining in the buffer
 * @Note		- this function will only return when all the data is transmitted by the bit shift register (when "len" = 0). Blocking call
 */
uint32_t __spi_send_partial(uint8_t *pTxBuffer, uint32_t fromb, uint32_t tob, SPI_ConfigDef *pSPIConfig, uint8_t initCOM, uint8_t endCOM) {
	uint8_t dff16BitsDef = PTR(pSPIConfig).spiBasePeripheral.DataFrame & SPI_CR1_DFF_16BITS; // Configured data frame format (DFF)
	uint8_t modifier = ( dff16BitsDef ? 2UL : 1UL);
	uint32_t len = tob;//(sizeof(pTxBuffer) / (dff16BitsDef ? sizeof(uint16_t) : sizeof(uint8_t)));// Length in bytes or half-words
	uint32_t counter = 0U;

	/// Calculation from the number of bytes to be transmitted
	if (tob > len) tob = len;

	// Safe operation
	if ( fromb >= len ) return counter;

	//Place the buffer in the right position
	pTxBuffer += fromb;

	/// We need to enable the SPI peripheral before actually start using it
	/// This needs to be done as the last part because not all the configuration
	///  was defined yet. Basically we set Bit 6 to "1" in the CR1 register.
	if (initCOM) __SPI_PERIPHERAL_ENABLE(pSPIConfig);

	while ( tob > 0 ) {
		// We need to wait until transfer buffer is free
		__SPI_WaitTXRX_EmptyBuffer(pSPIConfig->pSPIx, SPI_TRANSMITTER_FLAG);

		if ( dff16BitsDef ) {
			PTR(pSPIConfig->pSPIx).DR = PTR((uint16_t*) pTxBuffer);
			(uint16_t*)pTxBuffer++;
		}
		else {
			PTR(pSPIConfig->pSPIx).DR = PTR(pTxBuffer);
			pTxBuffer++;
		}

		// Wait until SPI is not busy
		__SPI_Wait_Busy(PTR(pSPIConfig).pSPIx);

		tob     -= modifier;
		counter += modifier;
	}

	/// We need to disable the SPI peripheral after using it.
	/// This needs to be done because the channels need to be put to high after
	///  that finish transmitting
	if (endCOM) __SPI_PERIPHERAL_DISABLE(pSPIConfig);

	return counter;
}

/*******************************************************************************
 * @fn			- __spi_send_partial_async
 * @brief		- Send the data buffer passed as parameter. It sends all the number of bytes/halfwords defined by the parameter "len". This
 * 				  uses the async characteristics of the IRQ to transmit the data, and it defines a non-blocking point for the MCU operations.
 * 				  Before this method can be called, a call to \__spi_configure_irq needs to be done first to enable the TX interrupt.
 *
 * @param[in] 	- the data to be written to the TX buffer
 * @param[in]	- the initial byte position to read from (for half words this amount is multiplied by 2)
 * @param[in]	- the length of data in bytes to be sent in the stream (for half words this amount is multiplied by 2)
 * @param[in]	- the SPI configuration handle to be used
 *
 * @return		- the current application state of the transmission buffer
 * @Note		- this method does not actually write anything to the stream, but instead register a transmission buffer to be populated
 * 				  by the specifics of the SPIs interrupts. Write is done the IRQ function handler.
 */
uint32_t __spi_send_partial_async(uint8_t *pTxBuffer, uint32_t fromb, uint32_t tob, SPI_ConfigDef *pSPIConfig) {
	uint8_t state = PTR(pSPIConfig).SPITxRxAsyncDef.txState;

	if ( !(state & SPI_APP_TXBUSY) ) {/// we can only change the buffer if we are not sending data to the slave/master
		uint8_t dff16BitsDef = PTR(pSPIConfig).spiBasePeripheral.DataFrame & SPI_CR1_DFF_16BITS; // Configured data frame format (DFF)
		uint8_t modifier = ( dff16BitsDef ? 2UL : 1UL);
		uint8_t counter = 0x0, len = tob - fromb;

		/// copy the bytes to the structure
		for ( ; counter < len ; counter+=modifier ) len -= modifier;

		// 1st. Saves the data to be transmitted in the handle
		pTxBuffer += fromb; // move the memory to position if this is the case
		PTR(pSPIConfig).SPITxRxAsyncDef.pTxBuffer = pTxBuffer;
		PTR(pSPIConfig).SPITxRxAsyncDef.txLen	  = counter;
		PTR(pSPIConfig).SPITxRxAsyncDef._originalRxLen = counter;

		// 2nd. Marks the state (using SR in this case) as BUSY in TX
		PTR(pSPIConfig).SPITxRxAsyncDef.txState = SPI_APP_TXBUSY;

		// 3rd. Activate the TXEIE flag for TXE interrupts
		__SPI_CR2_Enable(pSPIConfig, SPI_CR2_TXBUFF_EMPTY_INT_EN);
	}

	return state;
}

/*******************************************************************************
 * @fn			- __spi_receive
 * @brief		- Receives the data from MISO Pin and saves it in the given input data buffer. This is
 * 				  a blocking method call. This method will block the execution until data is placed in the
 * 				  DR register from the SPI peripheral.
 *
 * @param[in] 	- the buffer to be used for saving the data
 * @param[in]	- the length of data in bytes to be read from the stream (for half words this amount is multiplied by 2). Maximum 8bits read = 256 at a time
 * @param[in]	- the SPI configuration handle to be used
 * @param[in]	- flag to identify if the SPI communication channel needs to be initiated
 * @param[in]	- flag to identify if the SPI communication channel needs to be finished
 *
 * @return		- the number of bytes remaining in the buffer
 * @Note		- this function will only return when all the data is transmitted by the bit shift register (when "len" = 0). Blocking call
 */
void __spi_receive(uint8_t *pRxBuffer, uint8_t len, SPI_ConfigDef *pSPIConfig, uint8_t initCOM, uint8_t endCOM) {
	uint8_t dff16BitsDef = PTR(pSPIConfig).spiBasePeripheral.DataFrame & SPI_CR1_DFF_16BITS; // Configured data frame format (DFF)
	uint8_t modifier = ( dff16BitsDef ? 2UL : 1UL);
	uint32_t counter = 0;

	while (len > 0) {
		/// Wait receiver but is free - blocking instruction
		__SPI_WaitTXRX_EmptyBuffer(pSPIConfig->pSPIx, SPI_RECEIVER_FLAG);

		if ( dff16BitsDef ) {/// 16-bit read
			PTR((uint16_t*) pRxBuffer) = PTR(pSPIConfig->pSPIx).DR;
		}
		else {
			PTR(pRxBuffer) = PTR(pSPIConfig->pSPIx).DR;
		}

		pRxBuffer+=modifier;

		// Wait until SPI is not busy
		__SPI_Wait_Busy(PTR(pSPIConfig).pSPIx);

		len		  -= modifier;
		counter	  += modifier;
	}

	/// Reset the internal shift counter of the read buffer
	pRxBuffer -= counter;
}

/*******************************************************************************
 * @fn			- __spi_receive_async
 * @brief		- Maps the given buffer to the number of bytes/halfwords defined by the parameter "len". This
 * 				  uses the async characteristics of the IRQ to receive the data, and it defines a non-blocking point for the MCU operations.
 * 				  Before this method can be called, a call to \__spi_configure_irq needs to be done first to enable the RX interrupt.
 *
 * @param[in] 	- the data to be read into the RX buffer
 * @param[in]	- the initial byte position to read from (for half words this amount is multiplied by 2)
 * @param[in]	- the length of data in bytes to be sent in the stream (for half words this amount is multiplied by 2)
 * @param[in]	- the SPI configuration handle to be used
 *
 * @return		- the current application state of the receiver buffer
 * @Note		- this method does not read anything from the stream, but instead register a read buffer to be populated by the specifics of
 * 		    	  the SPIs interrupts. Read is done the IRQ function.
 */
uint32_t __spi_receive_async (uint8_t *pRxBuffer, uint8_t len, SPI_ConfigDef *pSPIConfig) {
	uint8_t state = PTR(pSPIConfig).SPITxRxAsyncDef.rxState;

	if ( !(state & SPI_APP_RXBUSY) ) {/// we can only change the buffer if we are not receiving data to the slave/master
		// 1st. Saves the data to be transmitted in the handle
		PTR(pSPIConfig).SPITxRxAsyncDef.pRxBuffer = pRxBuffer;
		PTR(pSPIConfig).SPITxRxAsyncDef.rxLen	  = len;
		PTR(pSPIConfig).SPITxRxAsyncDef._originalRxLen = len;

		// 2nd. Marks the state (using SR in this case) as BUSY in TX
		PTR(pSPIConfig).SPITxRxAsyncDef.rxState = SPI_APP_RXBUSY;

		// 3rd. Activate the TXEIE flag for TXE interrupts
		__SPI_CR2_Enable(pSPIConfig, SPI_CR2_RXBUFF_NOTEMPTY_INT_EN);
	}

	return state;
}

/*******************************************************************************
 * @fn			- __spi_configure_irq
 * @brief		- Configures the SPI interrupt with the function to be executed as callback based
 * 				  on one of the valid/available interrupts defined in the system. The SPI interrupts
 * 				  are directly connected to NVIC, so need to configure the EXTI for the GPIOx pins.
 *
 * @param[in] 	- the IRQ type of interrupt
 * @param[in]	- the callback function to be called based on its type
 * @param[in]	- the callback function to be triggered when an event happens
 *
 * @return		- none
 * @Note		- none
 */
void __spi_configure_irq(uint8_t IRQType, SPI_ConfigDef *pSPIConfig, irq_generic_handler_fn_t fn) {
	uint32_t hierarchyNumber = 0UL;

	/// Saves the handler function
	PTR(pSPIConfig).SPITxRxAsyncDef.SPIFN = fn;
	PTR(pSPIConfig).configuredIT = IRQType;

	/// Save the IRQ type and register the IRQ hierarchy in NVIC
	hierarchyNumber = __IRQNUM_FROM_SPIx(pSPIConfig);

	/// we need to select the right interrupts (ITs) to activate (in this case we only activate ERRIE)
	__SPI_CR2_Enable(pSPIConfig, SPI_CR2_ERR_INT_EN);

	/*
	 * Now, we have to initialize the IRQ handling...
	 */
	__irq_config(hierarchyNumber,
				 NVIC_IRQ_PRIORITY_15 /*Default priority value: 0b 1111*/,
				 ENABLE, fn);
}

static void __async_internal_send(SPI_ConfigDef *pSPICfg) {
	uint16_t dff16Bits = PTR(pSPICfg->pSPIx).CR1 & SPI_CR1_DFF_16BITS;
	uint8_t  modifier  = dff16Bits ? 0x2U : 0x1U;
	if ( dff16Bits ) { // 16bits communication
		 PTR(pSPICfg->pSPIx).DR = PTR((uint16_t*) PTR(pSPICfg).SPITxRxAsyncDef.pTxBuffer);
		 (uint16_t*) PTR(pSPICfg).SPITxRxAsyncDef.pTxBuffer++;
	}
	else {
		PTR(pSPICfg->pSPIx).DR = PTR(PTR(pSPICfg).SPITxRxAsyncDef.pTxBuffer);
		PTR(pSPICfg).SPITxRxAsyncDef.pTxBuffer++;
	}

	PTR(pSPICfg).SPITxRxAsyncDef.txLen -= modifier;

	/// we need to stop the interrupt and clean up things after all the data is sent through the pipe...
	if ( !PTR(pSPICfg).SPITxRxAsyncDef.txLen  ) {
		/// Close the SPI transmission and inform the application that TX is over.
		__SPI_CR2_Disable(pSPICfg, SPI_CR2_TXBUFF_EMPTY_INT_EN);

		/// we also need to clean the async data
		PTR(pSPICfg).SPITxRxAsyncDef.txLen = 0x0U;
		PTR(pSPICfg).SPITxRxAsyncDef.pTxBuffer = NULL;
		PTR(pSPICfg).SPITxRxAsyncDef.txState = SPI_APP_TXREADY;

		/// Makes to inform the application that the transmission is over
		if ( PTR(pSPICfg).SPITxRxAsyncDef.SPIFN ) {
			PTR(pSPICfg).SPITxRxAsyncDef.SPIFN(SPI_APP_TX_COMPLETE, (void*)pSPICfg);/// Triggered when all the data is written to the stream
		}
	}
}
static void __async_internal_receive(SPI_ConfigDef *pSPICfg) {
	uint16_t dff16Bits = PTR(pSPICfg->pSPIx).CR1 & SPI_CR1_DFF_16BITS;
	uint8_t  modifier  = dff16Bits ? 0x2U : 0x1U;
	if ( dff16Bits ) { // 16bits communication
		 PTR((uint16_t*) PTR(pSPICfg).SPITxRxAsyncDef.pRxBuffer) = (uint16_t) PTR(pSPICfg->pSPIx).DR;
		 (uint16_t*) PTR(pSPICfg).SPITxRxAsyncDef.pRxBuffer++;
	}
	else {
		PTR(PTR(pSPICfg).SPITxRxAsyncDef.pRxBuffer) = (uint8_t) PTR(pSPICfg->pSPIx).DR;
		PTR(pSPICfg).SPITxRxAsyncDef.pRxBuffer++;
	}

	PTR(pSPICfg).SPITxRxAsyncDef.rxLen -= modifier;

	/// we need to stop the interrupt and clean up things after all the data is sent through the pipe...
	if ( !PTR(pSPICfg).SPITxRxAsyncDef.rxLen  ) {
		/// Close the SPI transmission and inform the application that TX is over.
		__SPI_CR2_Disable(pSPICfg, SPI_CR2_RXBUFF_NOTEMPTY_INT_EN);

		/// Makes a call to the callback to inform the application that a new byte was received by the stream
		if ( PTR(pSPICfg).SPITxRxAsyncDef.SPIFN ) {
			PTR(pSPICfg).SPITxRxAsyncDef.rxLen = PTR(pSPICfg).SPITxRxAsyncDef._originalRxLen;
			PTR(pSPICfg).SPITxRxAsyncDef.SPIFN(SPI_APP_RX_COMPLETE, (void*)&pSPICfg->SPITxRxAsyncDef);/// Triggered when all the data is written to the stream
		}

		/// we also need to clean the async data
		PTR(pSPICfg).SPITxRxAsyncDef.rxLen = 0x0U;
		PTR(pSPICfg).SPITxRxAsyncDef.pRxBuffer = NULL;
		PTR(pSPICfg).SPITxRxAsyncDef.rxState = SPI_APP_TXREADY;
	}
}
static void __async_internal_errie(SPI_ConfigDef *pSPICfg, uint8_t errieCode) {
	PTR(pSPICfg).SPITxRxAsyncDef.errieCode = errieCode;
	uint16_t ovrRxByte, status;

	/// we need to clear the OVR flag
	ovrRxByte = PTR((uint16_t*) PTR(pSPICfg).pSPIx->DR);
	status    = PTR(pSPICfg).pSPIx->SR;
	__NONOTUSEDWARN_ status;

	PTR(pSPICfg).SPITxRxAsyncDef.errieCode = errieCode;
	PTR(pSPICfg).SPITxRxAsyncDef.pRxBuffer = ((uint8_t*) &ovrRxByte);
	PTR(pSPICfg).SPITxRxAsyncDef.rxLen     = 1U;
	/// Informs the application that the an overrun error ocurred
	if ( PTR(pSPICfg).SPITxRxAsyncDef.SPIFN ) {
		PTR(pSPICfg).SPITxRxAsyncDef.SPIFN(SPI_APP_ERRIE_OVR, (void*)pSPICfg);/// Triggered when all the data is written to the stream
	}
}

/**
 * @brief	- configure the action to be taken based on the SPI definition
 */
static void SPIx_Handler (SPI_RegDef* pSPI) {
	SPI_ConfigDef *pSPICfg = __GET_ASYNC_SPICONFIG(pSPI);

	/// Transmission
	uint8_t	itFlag 	  = PTR(pSPI).CR2 & SPI_CR2_TXBUFF_EMPTY_INT_EN,
			itFlagEvt = PTR(pSPI).SR & SPI_SR_TRANSMISSION_BUFF_NOTEMPTY;
	if ( itFlag && itFlagEvt && PTR(pSPICfg).configuredIT & SPI_IRQ_TXBUFFER_EMPTY ) {
		/// Calls back the function handler
		__async_internal_send(pSPICfg);
	}

	/// Reception
	itFlag 	  = PTR(pSPI).CR2 & SPI_CR2_RXBUFF_NOTEMPTY_INT_EN;
	itFlagEvt = PTR(pSPI).SR & SPI_SR_RECEIVER_BUFF_NOTEMPTY;
	if ( itFlag && itFlagEvt && PTR(pSPICfg).configuredIT & SPI_IRQ_RXBUFFER_NOTEMPTY ) {
		/// Calls back the function handler
		__async_internal_receive(pSPICfg);
	}

	/// ERRIE: Error interrupt enable (we are only processing OVR for now)
	itFlag 	  = PTR(pSPI).CR2 & SPI_CR2_ERR_INT_EN;
	if ( itFlag && PTR(pSPICfg).configuredIT & SPI_IRQ_OVERRUN_ERROR ) {
		/// Calls back the function handler
		__async_internal_errie(pSPICfg, SPI_IRQ_OVERRUN_ERROR);
	}
}

void SPI1_IRQHandler(void) {
	SPIx_Handler(__SPI1);
}
void SPI2_IRQHandler(void) {
	SPIx_Handler(__SPI2I2S2);
}
void SPI3_IRQHandler(void) {
	SPIx_Handler(__SPI3I2S3);
}
void SPI4_IRQHandler(void) {
	SPIx_Handler(__SPI4);
}

