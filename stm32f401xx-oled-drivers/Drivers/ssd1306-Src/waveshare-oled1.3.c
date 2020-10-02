/**
 ******************************************************************************
 * @file           : waveshare-oled1.3.c
 * @author         : J0a0 Gonzalez
 * @date		   : Oct 2, 2020
 * @version	       : 1.0
 * @brief          : TODO
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

#include "waveshare-oled1.3.h"

static GPIO_ConfigDef* __wv_init_gpio();
static SPI_ConfigDef* __wv_init_spi(SPI_RegDef *pSPIPeripheral, GPIO_ConfigDef *pGPIOCfg);
static void ssd1306_write_byte(GPIO_ConfigDef *pGPIOCfg, SPI_ConfigDef *pSPICfg, uint8_t chData, uint8_t chCmd);

static uint8_t spi_read_write_byte(SPI_ConfigDef* pSPIx, uint8_t chByte);
static uint32_t __pow(uint8_t m, uint8_t n);

/******** Private macros ***************************************************/
#define __SSD1306_WRITE_BYTE(pSPICfg, data) spi_read_write_byte((pSPICfg), data)

#define __SET_COL_START_ADDR(pGPIOCfg, pSPICfg) 	do { \
														ssd1306_write_byte((pGPIOCfg), (pSPICfg), 0x00, SSD1306_CMD); \
														ssd1306_write_byte((pGPIOCfg), (pSPICfg), 0x10, SSD1306_CMD); \
													} while(0)

#if !defined(SSD1306)
	#warning Please select first the target OLED device(SH1106 or SSD1306) in your application!
	#define SSD1306  //define SSD1306 by default
#endif

/* Private variables ---------------------------------------------------------*/
static uint8_t s_chDispalyBuffer[128][8];

/**************************************************************************
 * @fn			- ws_sys_start
 * @brief		- Initialization of the WaveShare OLED display
 *
 * @param [in]  - none
 *
 * @return		- The SPI configured object
 * @Note		- none
 */
SSD1306_Config_t ws_sys_start() {
	SSD1306_Config_t config;
	GPIO_ConfigDef *pGPIOCfg;
	SPI_ConfigDef  *pSPICfg;

	//// Give time for the processor to load
	_delay_ms(300);

	/// 1st. Initialize the GPIO port and pins
	pGPIOCfg = __wv_init_gpio();

	/// 2nd. Initialize the SPI communication protocol - we will use SPI1
	pSPICfg  = __wv_init_spi(__SPI1, pGPIOCfg);

	/// 3rd. Slave (SSD1306) initialization
	ssd1306_init(pGPIOCfg, pSPICfg);

	config.pGPIOConfiguration = pGPIOCfg;
	config.pSPIConfiguration  = pSPICfg;

	return config;
}

/************************ Internal functions *********************************/
static GPIO_ConfigDef* __wv_init_gpio() {
	GPIO_ConfigDef *pGPIOCfgA = __gpio_init(__GPIOA, NO); /// USE GPIOA

	// Define the pins
	GPIO_PinConfigDef mosiPin = __GET_PIN_CONFIG(pGPIOCfgA, SPI1_DIN_PIN);
	GPIO_PinConfigDef dcPin = __GET_PIN_CONFIG(pGPIOCfgA, SPI1_DC_PIN);
	GPIO_PinConfigDef sclkPin = __GET_PIN_CONFIG(pGPIOCfgA, SPI1_SCLK_PIN);
	GPIO_PinConfigDef csPin  = __GET_PIN_CONFIG(pGPIOCfgA, SPI1_CS_PIN);
	GPIO_PinConfigDef resPin  = __GET_PIN_CONFIG(pGPIOCfgA, SPI1_RES_PIN);

	// Configure the Pins: MOSI
	mosiPin.PinMode 				= GPIOx_MODE_ALTFN;
	mosiPin.PinAltFuncMode 			= GPIO_AFR_AF5;
	mosiPin.PinOType 				= GPIO_OTYPE_PUSHPULL; // it's ok for SPI, I2C needs to be in open drain
	mosiPin.PinPullUpPullDownCtrl 	= GPIO_PUPDR_PU;
	mosiPin.PinSpeed 				= GPIO_OSPEEDR_MEDIUM; // Speed to about 50MHz
	// Configure the Pins: RES
	resPin.PinMode 					= GPIOx_MODE_OUTPUT;
	resPin.PinOType 				= GPIO_OTYPE_PUSHPULL; // it's ok for SPI, I2C needs to be in open drain
	resPin.PinPullUpPullDownCtrl 	= GPIO_PUPDR_PD;
	resPin.PinSpeed 				= GPIO_OSPEEDR_MEDIUM; // Speed to about 50MHz

	// Configure the Pins: SCLK and MISO - with the same configuration as MOSI
	__CPY_PIN_CONFIG(&resPin, &dcPin, SPI1_DC_PIN);
	__CPY_PIN_CONFIG(&resPin, &csPin, SPI1_CS_PIN);
	__CPY_PIN_CONFIG(&mosiPin, &sclkPin, SPI1_SCLK_PIN);

	/// now we need to initialize the PINs on GPIO
	__gpio_init_pin(pGPIOCfgA, mosiPin);
	__gpio_init_pin(pGPIOCfgA, dcPin);
	__gpio_init_pin(pGPIOCfgA, sclkPin);
	__gpio_init_pin(pGPIOCfgA, csPin);
	__gpio_init_pin(pGPIOCfgA, resPin);

	return pGPIOCfgA;
}
static SPI_ConfigDef* __wv_init_spi(SPI_RegDef *pSPIPeripheral, GPIO_ConfigDef *pGPIOCfg) {
	SPI_ConfigDef *pSPICfg;
	SPI_PeripheralDef spiPeripheralDef;

	spiPeripheralDef.BusConfig = SPI_BUS_CONFIG_FD;
	spiPeripheralDef.DeviceMode = SPI_DEVICE_MODE_MASTER;
	spiPeripheralDef.SClkSpeed = SPI_SCLK_BAUDRATE_BY_8; // Baud rate of /8 (2MHz clock)
	spiPeripheralDef.DataFrame = SPI_CR1_DFF_8BITS; // We use 8-bits tx/rx
	spiPeripheralDef.CPolarity = SPI_CR1_POLARITY_HIGHONIDLE;
	spiPeripheralDef.CPhase	= SPI_CR1_PHASE_2NDCLKDATACATCH;
	spiPeripheralDef.SSM		= SPI_SSM_SW_DIS; // hardware slave mgmt enable for NSS pin
	spiPeripheralDef.SSIEnabled  = DISABLE; // We are not using software slave mgmt, so we don't need to activate SSI
	spiPeripheralDef.SSOEEnabled = ENABLE; // When SSOE is "1", SSM is managed by HW, and NSS is equal "0" when SPE="1" (Active) and equals "1" when SPE="0" (Inactive)
	spiPeripheralDef.CRCPolynomial = 0x7; /*Reset value*/

	pSPICfg = __spi_init(pSPIPeripheral, &spiPeripheralDef, /*Not using Async by configuring the SPI IRQ*/SPI_NORMAL);

	return pSPICfg;
}

/******************************************************************************************************
 ******************************************************************************************************
 ******************************************************************************************************
 ***********		WAVESHARE OLED 1.3 (SSD1306) API SPECIFIC IMPLEMENTATION		*******************
 ******************************************************************************************************
 ******************************************************************************************************
 ******************************************************************************************************
 ******************************************************************************************************/
void ssd1306_draw_bitmap(GPIO_ConfigDef *pGPIOCfg, SPI_ConfigDef *pSPIConfig, uint8_t chXpos, uint8_t chYpos, const uint8_t *pchBmp, uint8_t chWidth, uint8_t chHeight) {
	uint16_t i, j, byteWidth = (chWidth + 7) / 8;

    for(j = 0; j < chHeight; j ++) {
        for(i = 0; i < chWidth; i ++ ) {
            if(*(pchBmp + j * byteWidth + i / 8) & (128 >> (i & 7))) {
                ssd1306_draw_point(chXpos + i, chYpos + j, 1);
            }
        }
    }
}
void ssd1306_draw_3216char(GPIO_ConfigDef *pGPIOCfg, SPI_ConfigDef *pSPIConfig, uint8_t chXpos, uint8_t chYpos, uint8_t chChar) {
	uint8_t i, j;
	uint8_t chTemp = 0, chYpos0 = chYpos, chMode = 0;

	for (i = 0; i < 64; i ++) {
		chTemp = c_chFont3216[chChar - 0x30][i];
		for (j = 0; j < 8; j ++) {
			chMode = chTemp & 0x80? 1 : 0;
			ssd1306_draw_point(chXpos, chYpos, chMode);
			chTemp <<= 1;
			chYpos ++;
			if ((chYpos - chYpos0) == 32) {
				chYpos = chYpos0;
				chXpos ++;
				break;
			}
		}
	}
}
void ssd1306_draw_1616char(GPIO_ConfigDef *pGPIOCfg, SPI_ConfigDef *pSPIConfig, uint8_t chXpos, uint8_t chYpos, uint8_t chChar) {
	uint8_t i, j;
	uint8_t chTemp = 0, chYpos0 = chYpos, chMode = 0;

	for (i = 0; i < 32; i ++) {
		chTemp = c_chFont1612[chChar - 0x30][i];
		for (j = 0; j < 8; j ++) {
			chMode = chTemp & 0x80? 1 : 0;
			ssd1306_draw_point(chXpos, chYpos, chMode);
			chTemp <<= 1;
			chYpos ++;
			if ((chYpos - chYpos0) == 16) {
				chYpos = chYpos0;
				chXpos ++;
				break;
			}
		}
	}
}

/**************************************************************************
 * @fn			- ssd1306_display_string
 * @brief		- Displays a string on the screen
 *
 * @param [in]	- The GPIO port reference
 * @param [in]	- The SPI peripheral configuration reference
 * @param [in]  - Specifies the X position
 * @param [in]  - Specifies the Y position
 * @param [in]  - Pointer to a string to display on the screen
 *
 * @return		- none
 * @Note		- none
 */
void ssd1306_display_string(GPIO_ConfigDef *pGPIOCfg, SPI_ConfigDef *pSPIConfig, uint8_t chXpos, uint8_t chYpos, const uint8_t *pchString, uint8_t chSize, uint8_t chMode) {
    while (*pchString != '\0') {
    	if (chXpos > (SSD1306_WIDTH - chSize / 2)) {
    		chXpos = 0;
			chYpos += chSize;
			if (chYpos > (SSD1306_HEIGHT - chSize)) {
				chYpos = chXpos = 0;
				ssd1306_clear_screen(pGPIOCfg, pSPIConfig, 0x00);
			}
		}

        ssd1306_display_char(pGPIOCfg, pSPIConfig, chXpos, chYpos, *pchString, chSize, chMode);
        chXpos += chSize / 2;
        pchString ++;
    }
}

void ssd1306_display_num(GPIO_ConfigDef *pGPIOCfg, SPI_ConfigDef *pSPIConfig, uint8_t chXpos, uint8_t chYpos, uint32_t chNum, uint8_t chLen, uint8_t chSize) {
	uint8_t i;
	uint8_t chTemp, chShow = 0;

	for(i = 0; i < chLen; i ++) {
		chTemp = (chNum / __pow(10, chLen - i - 1)) % 10;
		if(chShow == 0 && i < (chLen - 1)) {
			if(chTemp == 0) {
				ssd1306_display_char(pGPIOCfg, pSPIConfig, chXpos + (chSize / 2) * i, chYpos, ' ', chSize, 1);
				continue;
			} else {
				chShow = 1;
			}
		}
	 	ssd1306_display_char(pGPIOCfg, pSPIConfig, chXpos + (chSize / 2) * i, chYpos, chTemp + '0', chSize, 1);
	}
}

/**************************************************************************
 * @fn			- ssd1306_display_char
 * @brief		-  Displays one character at the specified position
 *
 * @param [in]	- The GPIO port reference
 * @param [in]	- The SPI peripheral configuration reference
 * @param [in]  - Specifies the X position
 * @param [in]	- Specifies the Y position
 * @param [in]	-
 * @param [in[	-
 *
 * @return		- none
 * @Note		- none
 */
void ssd1306_display_char(GPIO_ConfigDef *pGPIOCfg, SPI_ConfigDef *pSPIConfig, uint8_t chXpos, uint8_t chYpos, uint8_t chChr, uint8_t chSize, uint8_t chMode) {
	uint8_t i, j;
	uint8_t chTemp, chYpos0 = chYpos;

	chChr = chChr - ' ';
    for (i = 0; i < chSize; i ++) {
		if (chSize == 12) {
			if (chMode) {
				chTemp = c_chFont1206[chChr][i];
			} else {
				chTemp = ~c_chFont1206[chChr][i];
			}
		} else {
			if (chMode) {
				chTemp = c_chFont1608[chChr][i];
			} else {
				chTemp = ~c_chFont1608[chChr][i];
			}
		}

        for (j = 0; j < 8; j ++) {
			if (chTemp & 0x80) {
				ssd1306_draw_point(chXpos, chYpos, 1);
			} else {
				ssd1306_draw_point(chXpos, chYpos, 0);
			}
			chTemp <<= 1;
			chYpos ++;

			if ((chYpos - chYpos0) == chSize) {
				chYpos = chYpos0;
				chXpos ++;
				break;
			}
		}
    }
}
static uint32_t __pow(uint8_t m, uint8_t n) {
	uint32_t result = 1;
	while(n --) result *= m;
	return result;
}

/**************************************************************************
 * @fn			- ssd1306_fill_screen
 * @brief		- Fills a rectangle
 *
 * @param [in]	- The GPIO port reference
 * @param [in]	- The SPI peripheral configuration reference
 * @param [in]  - Specifies the X position 1 (X top left position)
 * @param [in]  - Specifies the Y position 1 (Y top left position)
 * @param [in]  - Specifies the X position 2 (X bottom right position)
 * @param [in]  - Specifies the Y position 2 (Y bottom right position)
 *
 * @return		- none
 * @Note		- none
 */
void ssd1306_fill_screen(GPIO_ConfigDef *pGPIOCfg, SPI_ConfigDef *pSPIConfig, uint8_t chXpos1, uint8_t chYpos1, uint8_t chXpos2, uint8_t chYpos2, uint8_t chDot)
{
	uint8_t chXpos, chYpos;

	for (chXpos = chXpos1; chXpos <= chXpos2; chXpos ++) {
		for (chYpos = chYpos1; chYpos <= chYpos2; chYpos ++) {
			ssd1306_draw_point(chXpos, chYpos, chDot);
		}
	}

	ssd1306_refresh_gram(pGPIOCfg, pSPIConfig);
}

/**************************************************************************
 * @fn			- ssd1306_draw_point
 * @brief		- Draws a piont on the screen
 *
 * @param [in]  - Specifies the X position
 * @param [in]  - Specifies the Y position
 * @param [in]	- 0: the point turns off    1: the piont turns on
 *
 * @return		- none
 * @Note		- none
 */
void ssd1306_draw_point(uint8_t chXpos, uint8_t chYpos, uint8_t chPoint) {
	uint8_t chPos, chBx, chTemp = 0;

	if (chXpos > 127 || chYpos > 63) {
		return;
	}
	chPos = 7 - chYpos / 8; //
	chBx = chYpos % 8;
	chTemp = 1 << (7 - chBx);

	if (chPoint) {
		s_chDispalyBuffer[chXpos][chPos] |= chTemp;

	} else {
		s_chDispalyBuffer[chXpos][chPos] &= ~chTemp;
	}
}

/**************************************************************************
 * @fn			- ssd1306_display_off
 * @brief		- OLED turns off
 *
 * @param [in]	- The GPIO port reference
 * @param [in]	- The SPI peripheral configuration reference
 *
 * @return		- none
 * @Note		- none
 */
void ssd1306_display_off(GPIO_ConfigDef *pGPIOCfg, SPI_ConfigDef *pSPIConfig) {
	ssd1306_write_byte(pGPIOCfg, pSPIConfig, 0x8D, SSD1306_CMD);
	ssd1306_write_byte(pGPIOCfg, pSPIConfig, 0x10, SSD1306_CMD);
	ssd1306_write_byte(pGPIOCfg, pSPIConfig, 0xAE, SSD1306_CMD);
}

/**************************************************************************
 * @fn			- ssd1306_display_on
 * @brief		- OLED turns on
 *
 * @param [in]	- The GPIO port reference
 * @param [in]	- The SPI peripheral configuration reference
 *
 * @return		- none
 * @Note		- none
 */
void ssd1306_display_on(GPIO_ConfigDef *pGPIOCfg, SPI_ConfigDef *pSPIConfig) {
	ssd1306_write_byte(pGPIOCfg, pSPIConfig, 0x8D, SSD1306_CMD);
	ssd1306_write_byte(pGPIOCfg, pSPIConfig, 0x14, SSD1306_CMD);
	ssd1306_write_byte(pGPIOCfg, pSPIConfig, 0xAF, SSD1306_CMD);
}

/**************************************************************************
 * @fn			- ssd1306_init
 * @brief		- The initialization function for the WaveShare OLED 1.3 display
 *
 * @param [in]  - the GPIO configuration reference
 *
 * @return		- none
 * @Note		- none
 */
void ssd1306_init(GPIO_ConfigDef *pGPIOCfg, SPI_ConfigDef *pSPIConfig) {
	__SSD1306_CS_SET(pGPIOCfg);   //CS set
	__SSD1306_DC_CLR(pGPIOCfg);   //D/C reset
	__SSD1306_RES_SET(pGPIOCfg);  //RES set

	/// Initiate the SPI peripheral (SPE)
	__SPI_PERIPHERAL_ENABLE(pSPIConfig);

	ssd1306_write_byte(pGPIOCfg, pSPIConfig, 0xAE, SSD1306_CMD);//--turn off oled panel
	ssd1306_write_byte(pGPIOCfg, pSPIConfig, 0x00, SSD1306_CMD);//---set low column address
	ssd1306_write_byte(pGPIOCfg, pSPIConfig, 0x10, SSD1306_CMD);//---set high column address
	ssd1306_write_byte(pGPIOCfg, pSPIConfig, 0x40, SSD1306_CMD);//--set start line address  Set Mapping RAM Display Start Line (0x00~0x3F)
	ssd1306_write_byte(pGPIOCfg, pSPIConfig, 0x81, SSD1306_CMD);//--set contrast control register
	ssd1306_write_byte(pGPIOCfg, pSPIConfig, 0xCF, SSD1306_CMD);// Set SEG Output Current Brightness
	ssd1306_write_byte(pGPIOCfg, pSPIConfig, 0xA1, SSD1306_CMD);//--Set SEG/Column Mapping
	ssd1306_write_byte(pGPIOCfg, pSPIConfig, 0xC0, SSD1306_CMD);//Set COM/Row Scan Direction
	ssd1306_write_byte(pGPIOCfg, pSPIConfig, 0xA6, SSD1306_CMD);//--set normal display
	ssd1306_write_byte(pGPIOCfg, pSPIConfig, 0xA8, SSD1306_CMD);//--set multiplex ratio(1 to 64)
	ssd1306_write_byte(pGPIOCfg, pSPIConfig, 0x3f, SSD1306_CMD);//--1/64 duty
	ssd1306_write_byte(pGPIOCfg, pSPIConfig, 0xD3, SSD1306_CMD);//-set display offset	Shift Mapping RAM Counter (0x00~0x3F)
	ssd1306_write_byte(pGPIOCfg, pSPIConfig, 0x00, SSD1306_CMD);//-not offset
	ssd1306_write_byte(pGPIOCfg, pSPIConfig, 0xd5, SSD1306_CMD);//--set display clock divide ratio/oscillator frequency
	ssd1306_write_byte(pGPIOCfg, pSPIConfig, 0x80, SSD1306_CMD);//--set divide ratio, Set Clock as 100 Frames/Sec
	ssd1306_write_byte(pGPIOCfg, pSPIConfig, 0xD9, SSD1306_CMD);//--set pre-charge period
	ssd1306_write_byte(pGPIOCfg, pSPIConfig, 0xF1, SSD1306_CMD);//Set Pre-Charge as 15 Clocks & Discharge as 1 Clock
	ssd1306_write_byte(pGPIOCfg, pSPIConfig, 0xDA, SSD1306_CMD);//--set com pins hardware configuration
	ssd1306_write_byte(pGPIOCfg, pSPIConfig, 0x12, SSD1306_CMD);
	ssd1306_write_byte(pGPIOCfg, pSPIConfig, 0xDB, SSD1306_CMD);//--set vcomh
	ssd1306_write_byte(pGPIOCfg, pSPIConfig, 0x40, SSD1306_CMD);//Set VCOM Deselect Level
	ssd1306_write_byte(pGPIOCfg, pSPIConfig, 0x20, SSD1306_CMD);//-Set Page Addressing Mode (0x00/0x01/0x02)
	ssd1306_write_byte(pGPIOCfg, pSPIConfig, 0x02, SSD1306_CMD);//
	ssd1306_write_byte(pGPIOCfg, pSPIConfig, 0x8D, SSD1306_CMD);//--set Charge Pump enable/disable
	ssd1306_write_byte(pGPIOCfg, pSPIConfig, 0x14, SSD1306_CMD);//--set(0x10) disable
	ssd1306_write_byte(pGPIOCfg, pSPIConfig, 0xA4, SSD1306_CMD);// Disable Entire Display On (0xa4/0xa5)
	ssd1306_write_byte(pGPIOCfg, pSPIConfig, 0xA6, SSD1306_CMD);// Disable Inverse Display On (0xa6/a7)
	ssd1306_write_byte(pGPIOCfg, pSPIConfig, 0xAF, SSD1306_CMD);//--turn on oled panel

	ssd1306_clear_screen(pGPIOCfg, pSPIConfig, 0x00);
	_delay_ms(100U); // per specification
}

/**************************************************************************
 * @fn			- ssd1306_clear_screen
 * @brief		- Clears the screen
 *
 * @param [in]  - the GPIO pointer reference
 * @param [in] 	- the SPI peripheral configuration
 * @param [in] 	- the byte to be transmitted
 *
 * @return		- none
 * @Note		- none
 */
void ssd1306_clear_screen(GPIO_ConfigDef *pGPIOCfg, SPI_ConfigDef *pSPICfg, uint8_t chFill) {
	uint8_t i, j;

	for (i = 0; i < 8; i ++) {
		ssd1306_write_byte(pGPIOCfg, pSPICfg, 0xB0 + i, SSD1306_CMD);
		__SET_COL_START_ADDR(pGPIOCfg, pSPICfg);
		for (j = 0; j < 128; j ++) {
			s_chDispalyBuffer[j][i] = chFill;
		}
	}

	ssd1306_refresh_gram(pGPIOCfg, pSPICfg);
}

/**************************************************************************
 * @fn			- ssd1306_refresh_gram
 * @brief		- Refreshs the graphic ram
 *
 * @param [in]  - the GPIO pointer reference
 * @param [in] 	- the SPI peripheral configuration
 *
 * @return		- none
 * @Note		- none
 */
void ssd1306_refresh_gram(GPIO_ConfigDef *pGPIOCfg, SPI_ConfigDef *pSPICfg) {
	uint8_t i, j;

	for (i = 0; i < 8; i ++) {
		ssd1306_write_byte(pGPIOCfg, pSPICfg, 0xB0 + i, SSD1306_CMD);
		__SET_COL_START_ADDR(pGPIOCfg, pSPICfg);
		for (j = 0; j < 128; j ++) {
			ssd1306_write_byte(pGPIOCfg, pSPICfg, s_chDispalyBuffer[j][i], SSD1306_DAT);
		}
	}
}

static void ssd1306_write_byte(GPIO_ConfigDef *pGPIOCfg, SPI_ConfigDef *pSPICfg, uint8_t chData, uint8_t chCmd) {
	__SSD1306_CS_CLR(pGPIOCfg);

	if (chCmd) {
		__SSD1306_DC_SET(pGPIOCfg);
	} else {
		__SSD1306_DC_CLR(pGPIOCfg);
	}
	/// SPI COMM
	__SSD1306_WRITE_BYTE(pSPICfg, chData);

	__SSD1306_DC_SET(pGPIOCfg);
	__SSD1306_CS_SET(pGPIOCfg);
}
static uint8_t spi_read_write_byte(SPI_ConfigDef* pSPIConfig, uint8_t chByte) {
	uint8_t chRetry = 0;

	__spi_send_partial(&chByte, 0, 1, pSPIConfig, NO, NO);
	__spi_receive(&chRetry, 1, pSPIConfig, NO, NO);

	return chRetry;
}
