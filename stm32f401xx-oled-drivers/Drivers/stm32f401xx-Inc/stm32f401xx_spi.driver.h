/*
 * stm32f401xx_spi.driver.h
 *
 *  Created on: Sep 26, 2020
 *      Author: jgonzalezlopes
 */

#ifndef STM32F401XX_SPI_DRIVER_H_
#define STM32F401XX_SPI_DRIVER_H_

#include "stm32f401xx.h"

#ifdef __cplusplus
 extern "C" {
#endif

/********************** SPI INTERNAL REGISTER MODIFIERS *********************************************************/
/*
 * @SPI_CR1
 */
#define	SPI_CRx_RESET							RESET
#define	SPI_CR1_PHASE_2NDCLKDATACATCH	 		0x00000001U		/*!< Clock phase: The first clock transition is the first data capture edge> */
#define	SPI_CR1_PHASE_1STCLKDATACATCH			SPI_CRx_RESET  	/*!< Clock phase: The second clock transition is the first data capture edge> */
#define SPI_CR1_POLARITY_LOWONIDLE				SPI_CRx_RESET	/*!< Clock polarity: CK to 0 when idle> */
#define SPI_CR1_POLARITY_HIGHONIDLE				0x00000002U		/*!< Clock polarity: CK to 1 when idle> */
#define SPI_CR1_MASTER_SELECTION				0x00000004UL	/*!< 1: Master configuration> */
#define SPI_CR1_SLAVE_SELECTION					SPI_CRx_RESET	/*!< 1: Slave configuration> */

#define	SPI_CR1_SPIENABLE						0x00000040U		/*!< SPI enable: SPI_CR1_SPE=(1<<6)> */
#define	SPI_CR1_SPIDISABLE						SPI_CRx_RESET	/*!< SPI disable: SPI_CR1_SPE=(0<<6)> */
#define SPI_CR1_LSBFIRST						0x00000080U  	/*!< Bit 7 LSBFIRST: Frame format> */
#define SPI_CR1_MSBFIRST						SPI_CRx_RESET  	/*!< Bit 7 MSBFIRST: Frame format> */
#define SPI_CR1_INTERNAL_SLVSELECT				0x00000100U		/*!< Bit 8 SSI: Internal slave select - This bit has an effect only when the SSM bit is set. The value of this bit is forced onto the NSS pin and the IO value of the NSS pin is ignored.> */
#define SPI_CR1_SWSLAVEMGMT_EN					0x00000200U		/*!< Bit 9 SSM: Software slave management - When the SSM bit is set, the NSS pin input is replaced with the value from the SSI bit. (Bit 8)>*/
#define SPI_CR1_SWSLAVEMGMT_DIS					SPI_CRx_RESET	/*!< Bit 9 SSM: Software slave management - When the SSM bit is set, the NSS pin input is replaced with the value from the SSI bit. (Bit 8)>*/
#define SPI_CR1_RECEIVE_ONLY					0x00000400U		/*!< This bit combined with the BIDImode bit selects the direction of transfer in 2-line
																	 unidirectional mode. This bit is also useful in a multislave system in which this particular
																	 slave is not accessed, the output from the accessed slave is not corrupted.> */
#define	SPI_CR1_DFF_8BITS						SPI_CRx_RESET	/*!< 0: 8-bit data frame format is selected for transmission/reception> */
#define	SPI_CR1_DFF_16BITS						0x00000800U		/*!< 0: 16-bit data frame format is selected for transmission/reception> */

/*
 * When the SPI is configured in full duplex or transmitter only modes, CRCNEXT must be written as soon as the last data is written
 * to the SPI_DR register. When the SPI is configured in receiver only mode, CRCNEXT must be set after the second last data reception.
 * This bit should be kept cleared when the transfers are managed by DMA. It is not used in I2S mode.
 */
#define	SPI_CR1_CRCNEXT							0x00001000U		/*!< Next transfer is CRC (CRC phase)> */
#define	SPI_CR1_CRC_DATAPH						SPI_CRx_RESET	/*!< Next transfer is CRC (CRC phase)> */

#define SPI_CR1_HWCRCCALC_ENABLE				0x00002000U		/*!< Bit 13 CRCEN: Hardware CRC calculation enable> */
#define SPI_CR1_HWCRCCALC_DISABLE				SPI_CRx_RESET	/*!< Bit 13 CRCEN: Hardware CRC calculation disable> */
/*
 * This bit combined with the BIDImode bit selects the direction of transfer in bidirectional mode
 * 	0: Output disabled (receive-only mode)
 * 	1: Output enabled (transmit-only mode)
 */
#define SPI_CR1_TRANSMISSION_ONLY				0x00004000U		/*!< Bit 14 BIDIOE: Output enable in bidirectional mode> */
//#define SPI_CR1_RECEIVE_ONLY					SPI_CRx_RESET	/*!< Bit 14 BIDIOE: Output enable in bidirectional mode> */
#define	SPI_CR1_1LINE_BIDIR_DATA_MODE			0x00008000U		/*!< 1: 1-line bidirectional data mode selected> */
#define	SPI_CR1_2LINE_UNIDIR_DATA_MODE			SPI_CRx_RESET	/*!< 0: 2-line unidirectional data mode selected> */

/*
 * @SPI_CR2
 */
 /*
  * Bit 0 RXDMAEN: Rx buffer DMA enable
  */
#define SPI_CR2_RXBUFF_DMA_EN					(1<<0)			/*!< Bit 0 RXDMAEN: Rx buffer DMA enable> */

/*
 * Bit 1 @TXDMAEN: Tx buffer DMA enable
 */
#define SPI_CR2_TXBUFF_DMA_EN					(1<<1)			/*!< 1: Tx buffer empty> */
/*
 * Bit 2 SSOE: SS output enable
 * Note: This bit is not used in I2S mode and SPI TI mode.
 */
#define SPI_CR2_SS_OUTPUT_DIS					SPI_CRx_RESET	/*!< 0: SS output is disabled in master mode and the cell can work in multimaster configuration > */
#define SPI_CR2_SS_OUTPUT_EN					(1<<2)			/*!< 1: SS output is enabled in master mode and when the cell is enabled. The cell cannot work in a multimaster environment. */
/*
 * Bit 3 FRF: Frame format
 * Note: This bit is not used in I2S mode.
 */
#define	SPI_CR2_TI_MODE							(1<<4)			/*!< 1 SPI TI mode > */
#define	SPI_CR2_MOTOROLA_MODE					SPI_CRx_RESET	/*!< 0: SPI Motorola mode > */
/*
 * Bit 5 ERRIE: Error interrupt enable
 * This bit controls the generation of an interrupt when an error condition occurs)(CRCERR, OVR, MODF in SPI mode,
 * FRE in TI mode and UDR, OVR, and FRE in I2S mode).
 */
#define	SPI_CR2_ERR_INT_EN						(1<<5)			/*!< 1: Error interrupt is enabled > */
#define	SPI_CR2_ERR_INT_DIS						SPI_CRx_RESET	/*!< 0: Error interrupt is masked > */
/*
 * Bit 6 RXNEIE: RX buffer not empty interrupt enable
 */
#define	SPI_CR2_RXBUFF_NOTEMPTY_INT_EN			(1<<6)			/*!< 1: RXNE interrupt not masked. Used to generate an interrupt request when the RXNE flag is set. > */
#define	SPI_CR2_RXBUFF_NOTEMPTY_INT_DIS			SPI_CRx_RESET	/*!< 0: No overrun occurred> */
/*
 * Bit 7 TXEIE: Tx buffer empty interrupt enable
 */
#define	SPI_CR2_TXBUFF_EMPTY_INT_EN				(1<<7)			/*!< 1: TXE interrupt not masked. Used to generate an interrupt request when the TXE flag is set. > */
#define	SPI_CR2_TXBUFF_EMPTY_INT_DIS			SPI_CRx_RESET	/*!< 0: TXE interrupt masked > */


/*
 * SPI status register (SPI_SR). Address offset: 0x08; Reset value: 0x0002
 */
#define SPI_SR_RECEIVER_BUFF_NOTEMPTY			(1<<0)			/*!< Bit 0 RXNE: Receive buffer not empty. 1: Rx buffer not empty >*/
#define SPI_SR_TRANSMISSION_BUFF_NOTEMPTY		(1<<1)			/*!< Bit 1 TXE: Transmit buffer empty. 1: Rx buffer not empty >*/
#define	SPI_SR_CHSIDERIGHT_HAS_TRANSORRECEIVED	(1<<2)			/*!< Bit 2 CHSIDE: Channel side.1: Channel Right has to be transmitted or has been received >*/
#define	SPI_SR_CHSIDELEFT_HAS_TRANSORRECEIVED	0x0U			/*!< Bit 2 CHSIDE: Channel side.1: Channel Right has to be transmitted or has been received >*/
#define SPI_SR_UNDERRUN							(1<<3)			/*!< Bit 3 UDR: Underrun flag. 1: Underrun occurred > */
#define SPI_SR_CRC_ERROR						(1<<4)			/*!< Bit 4 CRCERR: CRC error flag. 1: CRC value received does not match the SPI_RXCRCR value>*/
#define SPI_SR_MODE_FAULT						(1<<5)			/*!< Bit 5 MODF: Mode fault. 1: Mode fault occurred - Refer to Section 20.4.8 of the STM32F401xx reference manual: Error flags for the software sequence.>*/
#define SPI_SR_OVERRUN							(1<<6)			/*!< Bit 6 OVR: Overrun flag. 1: Overrun occurred>*/
#define SPI_SR_BUSY								(1<<7)			/*!< Bit 7 BSY: Busy flag. 1: SPI(or I2S)is busy in communication or Tx buffer is not empty - BSY flag must be used with caution: refer to Section 20.3.7 and Section 20.3.8 of the STM32F401xx reference manual.>*/
#define SPI_SR_FFM_ERROR						(1<<8)			/*!< Bit 8 FRE: Frame format error. 1: A frame format error occurred - Note: This flag is used when the SPI operates in TI slave mode or I2S slave mode (refer to Section 20.3.10 of the STM32F401xx reference manual).>*/

#define	SPI_TRANSMITTER_FLAG					SET
#define	SPI_RECEIVER_FLAG						RESET

/*
 * SPI_I2S prescaler register (@SPI_I2SPR). Address offset: 0x20
 * Note: This bit should be configured when the I2S is disabled. It is used only when the I2S is in master mode.
 * This bit is not used in SPI mode.
 */
/*
 * Bit 9 MCKOE: Master clock output enable
 */
#define	SPI_I2SPR_MASTERCLK_OUT_EN		(1<<9)			/*!< 1: Master clock output is enabled> */
#define	SPI_I2SPR_MASTERCLK_OUT_DIS		SPI_CRx_RESET	/*!< 0: Master clock output is disable> */
/*
 * Bit 8 ODD: Odd factor for the prescaler
 * Refer to Section 20.4.4: Clock generator. Not used in SPI mode.
 * Note: This bit should be configured when the I2S is disabled. It is used only when the I2S is in master mode.
 */
#define	SPI_I2SPR_ODD_DIVIDER			(1<<8)			/*!< 1: real divider value is = (I2SDIV * 2)+1> */
#define	SPI_I2SPR_POS_DIVIDER			SPI_CRx_RESET	/*!< 0: real divider value is = I2SDIV *2> */

/*
 * @SPI_DeviceMode
 */
#define	SPI_DEVICE_MODE_MASTER		SPI_CR1_MASTER_SELECTION	/*!<Master select>*/
#define SPI_DEVICE_MODE_SLAVE		SPI_CR1_SLAVE_SELECTION		/*!<Slave select>*/

/*
 * @SPI_BusConfig
 */
#define SPI_BUS_CONFIG_FD				SPI_CR1_2LINE_UNIDIR_DATA_MODE	/*!<Full duplex configuration>*/
#define SPI_BUS_CONFIG_HD				SPI_CR1_1LINE_BIDIR_DATA_MODE	/*!<Half duplex configuration>*/
//no need, just disconnect the MISO line: #define SPI_BUS_CONFIG_SIMPLEX_TXONLY	0x0	/*!<Simplex configuration: transfer only no receive>*/
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY	SPI_CR1_RECEIVE_ONLY			/*!<Simplex configuration: receiver only no transmission>*/

/*
 * @SPI_SClkSpeed
 */
#define SPI_SCLK_BAUDRATE_BY_2					SPI_CRx_RESET	/*!< 000: fPCLK/2> */
#define SPI_SCLK_BAUDRATE_BY_4					0x00000008U		/*!< 001: fPCLK/4> */
#define SPI_SCLK_BAUDRATE_BY_8					0x00000010U		/*!< 010: fPCLK/8> */
#define SPI_SCLK_BAUDRATE_BY_16					0x00000018U		/*!< 011: fPCLK/16> */
#define SPI_SCLK_BAUDRATE_BY_32					0x00000020U		/*!< 100: fPCLK/32> */
#define SPI_SCLK_BAUDRATE_BY_64					0x00000028U		/*!< 101: fPCLK/64> */
#define SPI_SCLK_BAUDRATE_BY_128				0x00000030U		/*!< 110: fPCLK/128> */
#define SPI_SCLK_BAUDRATE_BY_256				0x00000038U		/*!< 111: fPCLK/256 >*/

/*
 * @SPI_SSM
 */
#define	SPI_SSM_SW_EN		SPI_CR1_SWSLAVEMGMT_EN				/*!< Hardware management >*/
#define	SPI_SSM_SW_DIS		SPI_CR1_SWSLAVEMGMT_DIS				/*!< Software management >*/

/*
 * Define the SPI interrupt types available in the IRQ/NVIC hierarchy
 */
#define	SPI_IRQ_TXBUFFER_EMPTY		0x1U	/*!< TXE: Transmit buffer empty flag. Event control bit: RXNEIE (TXEIE) >*/
#define	SPI_IRQ_RXBUFFER_NOTEMPTY	0x2U	/*!< RXNE: Receive buffer not empty flag. Event control bit: RXNEIE (CR2) >*/
#define	SPI_IRQ_MASTERMODE_FAULT	0x3U	/*!< MODF: Master Mode fault event. Event control bit: ERRIE (CR2) >*/
#define	SPI_IRQ_OVERRUN_ERROR		0x4U	/*!< OVR: CRC error flag. Event control bit: ERRIE (CR2) >*/
#define	SPI_IRQ_CRC_ERROR			0x5U	/*!< CRCERR: Overrun error. Event control bit: ERRIE (CR2) >*/
#define	SPI_IRQ_TIDFF_ERROR			0x6U	/*!< FRE: TI frame format error. Event control bit: ERRIE (CR2) >*/

/*
 * Application custom flags
 */
#define SPI_APP_TXREADY				RESET
#define SPI_APP_RXREADY				RESET
#define SPI_APP_TXBUSY				( SPI_SR_BUSY | SPI_SR_TRANSMISSION_BUFF_NOTEMPTY )
#define SPI_APP_RXBUSY				( SPI_SR_BUSY | SPI_SR_RECEIVER_BUFF_NOTEMPTY )
#define SPI_APP_TX_COMPLETE			0x10U
#define SPI_APP_RX_COMPLETE			0x20U
#define SPI_APP_ERRIE_OVR			0x40U

#define	SPI_ASYNC					0x1	/*!< specifies that the SPI configuration is asynchronous - sharing the same/single SPI configuration >*/
#define	SPI_NORMAL					0x0	/*!< specifies that the SPI configuration is synchronous >*/

/********************** SPI REGISTER STRUCTURE DEFINITION *******************************************************/
/**
 * @brief	- Configuration structure for the SPIx peripheral
 */
typedef struct {
	__RW uint16_t	DeviceMode;
	__RW uint16_t	BusConfig;
	__RW uint16_t	SClkSpeed;
	__RW uint16_t	DataFrame;
	__RW uint16_t	CPolarity;
	__RW uint16_t	CPhase;
	__RW uint16_t	SSM;
	__RW uint8_t	FrameFormat; /*!< Frame format configuration in the MCU>*/
	__RW uint16_t	CRCPolynomial;

	/// Enable/Disable properties (@SPI_RC2 bits controls mainly)
	__RW uint8_t	LSBFirstEnabled;			/*!< First bit in the transmition is MSB (most significant bit) >*/
	__RW uint8_t	SSIEnabled;
	__RW uint8_t	SSOEEnabled: ENABLE; 		/*!< Enable or disable a multimaster configuration in the MCU>*/
	__RW uint8_t	RXDMABufferEnabled; 		/*!< Enable or disable the Rx buffer DMA in the MCU>*/
	__RW uint8_t	TXDMABufferEnabled; 		/*!< Enable or disable the Tx buffer DMA in the MCU>*/
	__RW uint8_t	ErrorInterruptEnab; 		/*!< Enable or disable the the error interrupt in the MCU>*/
	__RW uint8_t	RXBufferInterruptEnabled; 	/*!< Enable or disable the RX buffer "not empty" interrupt in the MCU>*/
	__RW uint8_t	TXBufferInterruptEnabled; 	/*!< Enable or disable the TX buffer "empty" interrupt in the MCU>*/

} SPI_PeripheralDef;

/*
 * Internally defined TX/RX async communication structure
 */
typedef struct {
	irq_generic_handler_fn_t SPIFN;	/*!< Saves the generic function handler to be used by IRQ >*/

	/// Internal interrupt control
	uint8_t		*pTxBuffer;		/*!< Saves the current used transmission buffer >*/
	uint8_t		*pRxBuffer;		/*!< Saves the current used receiver buffer >*/
	uint32_t	txLen;			/*!< The length of the transmission buffer >*/
	uint32_t	rxLen;			/*!< The length of the receiver buffer >*/
	uint32_t	_originalRxLen;
	uint8_t		txState;		/*!< The current state of the transmission buffer >*/
	uint8_t		rxState;		/*!< The current state of the receiver buffer >*/

	uint8_t		errieCode;		/*!< In case something going wrong, the correct IRQ ERRIE code will be saved here >*/
} SPI_AsyncTXRXDef;

/**
 * @brief	- Handle structure for the SPIx peripheral
 */
typedef struct {
	SPI_RegDef	*pSPIx;	/*!< This holds the base address of the SPIx(x:0,1,2) peripheral >*/
	SPI_PeripheralDef	spiBasePeripheral;

	SPI_AsyncTXRXDef SPITxRxAsyncDef;
	uint8_t		configuredIT;	/*!< Define the available configured interrupts in the system >*/
} SPI_ConfigDef;

/**************************** SPI MACROS ************************************************************************/
#define	__SPI_WaitTXRX_EmptyBuffer(p, T) 	do { \
												if ( T & SPI_TRANSMITTER_FLAG ) {\
													while ( !((p)->SR & SPI_SR_TRANSMISSION_BUFF_NOTEMPTY ) );\
												}\
												else if ( T & SPI_RECEIVER_FLAG ) {\
													while ( ((p)->SR & SPI_SR_RECEIVER_BUFF_NOTEMPTY ) );\
												}\
											} while(0)
#define	__SPI_Wait_Busy(p)	 				do {\
												while ( ((p)->SR & SPI_SR_BUSY ) );\
											} while(0)
#define	__SPI_PERIPHERAL_ENABLE(p)			do {\
												PTR((p)->pSPIx).CR1 |= SPI_CR1_SPIENABLE;\
											} while (0)
#define	__SPI_PERIPHERAL_DISABLE(p)			do {\
												PTR((p)->pSPIx).CR1 &= ~(SPI_CR1_SPIENABLE);\
											} while (0)
#define __SPI_SSI_CONFIG(p, ed)				do {\
												if (ed) {\
													PTR((p)->pSPIx).CR1 |=  SPI_CR1_INTERNAL_SLVSELECT;\
												} else {\
													PTR((p)->pSPIx).CR1 &= ~SPI_CR1_INTERNAL_SLVSELECT;\
												}\
											} while (0)
#define __SPI_SSOE_CONFIG(p, ed)			do {\
												if (ed) {\
													PTR((p)->pSPIx).CR2 |=  SPI_CR2_SS_OUTPUT_EN;\
												} else {\
													PTR((p)->pSPIx).CR2 &= ~SPI_CR2_SS_OUTPUT_EN;\
												}\
											} while (0)
#define __SPI_LSBFIRST_CONFIG(p, ed)		do {\
												if (ed) {\
													PTR((p)->pSPIx).CR1 |=  SPI_CR1_LSBFIRST;\
												} else {\
													PTR((p)->pSPIx).CR1 &= ~SPI_CR1_LSBFIRST;\
												}\
											} while (0)

/**
 * @brief	- Returns the IRQ hierarchy number based on the configured SPI being used
 */
#define __IRQNUM_FROM_SPIx(pSPICfg)			( PTR((pSPICfg)).pSPIx == __SPI1 ? IRQ_HIERARCHY_SPI1 :\
											( PTR((pSPICfg)).pSPIx == __SPI2I2S2 ? IRQ_HIERARCHY_SPI2 :\
											( PTR((pSPICfg)).pSPIx == __SPI3I2S3 ? IRQ_HIERARCHY_SPI3 : IRQ_HIERARCHY_SPI4 ) ) )

#define __SPI_CR2_Enable(pSPICfg, bit)		do {\
												PTR((pSPICfg)->pSPIx).CR2 |= (bit);\
											} while(0);
#define __SPI_CR2_Disable(pSPICfg, bit)		do {\
												PTR((pSPICfg)->pSPIx).CR2 &= ~(bit);\
											} while(0);

/***************************************************************************************************************
 * 								APIs supported by the SPI driver
 * 		For more information on how to use the functions and base structures, consult the STM32F401xx
 * 		reference manual and datasheet.
 ***************************************************************************************************************/
void 			__spi_pclk_control	(SPI_RegDef* pSPICfgDef, uint8_t spiEnDi);
SPI_ConfigDef*	__spi_init			(SPI_RegDef *pSPIx, SPI_PeripheralDef *pPDef, uint8_t asyncOper);
void			__spi_deinit		(SPI_ConfigDef *pSPIConfig);
void			__spi_send			(uint8_t *pTxBuffer, SPI_ConfigDef *pSPIConfig);
uint32_t		__spi_send_partial	(uint8_t *pTxBuffer, uint32_t fromb, uint32_t tob, SPI_ConfigDef *pSPIConfig, uint8_t initCOM, uint8_t endCOM);
uint32_t 		__spi_send_partial_async(uint8_t *pTxBuffer, uint32_t fromb, uint32_t tob, SPI_ConfigDef *pSPIConfig);
void			__spi_receive		(uint8_t *pRxBuffer, uint8_t len, SPI_ConfigDef *pSPIConfig, uint8_t initCOM, uint8_t endCOM);
uint32_t		__spi_receive_async (uint8_t *pRxBuffer, uint8_t len, SPI_ConfigDef *pSPIConfig);

void __spi_configure_irq(uint8_t IRQType, SPI_ConfigDef *pSPIConfig, irq_generic_handler_fn_t fn);

#ifdef __cplusplus
}
#endif

#endif /* STM32F401XX_SPI_DRIVER_H_ */
