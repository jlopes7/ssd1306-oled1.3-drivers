/**
 ******************************************************************************
 * @file           : ssd1306.h
 * @author         : J0a0 Gonzalez
 * @date		   : Oct 2, 2020
 * @version	       : 1.0
 * @brief          : Waveshare OLED 1.3 driver specification
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
#ifndef SSD1306_H_
#define SSD1306_H_

#include "stm32f401xx_gpio.driver.h"
#include "stm32f401xx_spi.driver.h"
#include "Fonts.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define SSD1306_CMD    0
#define SSD1306_DAT    1

#define SSD1306_WIDTH    128
#define SSD1306_HEIGHT   64

/*********************** GPIOx pin mapping configuration to SPI ********************/
#define SPI1_DIN_PIN	GPIOx_PIN_07	/*MOSI*/
#define SPI1_DC_PIN		GPIOx_PIN_06	/*MISO*/
#define SPI1_SCLK_PIN	GPIOx_PIN_05	/*Clock*/
#define SPI1_CS_PIN		GPIOx_PIN_04	/*NSS*/
#define SPI1_RES_PIN	GPIOx_PIN_03	/*RES*/

#define SPI2_MOSI_PIN	GPIOx_PIN_15
#define SPI2_MISO_PIN	GPIOx_PIN_14
#define SPI2_SCLK_PIN	GPIOx_PIN_10
#define SPI2_NSS_PIN	GPIOx_PIN_09

/* Macros define ------------------------------------------------------------*/
#define __SSD1306_CS_SET(pGPIOCfg)      __gpio_write_pin(SET, __GET_PIN_CONFIG((pGPIOCfg), SPI1_CS_PIN), (pGPIOCfg))
#define __SSD1306_CS_CLR(pGPIOCfg)		__gpio_write_pin(RESET, __GET_PIN_CONFIG((pGPIOCfg), SPI1_CS_PIN), (pGPIOCfg))

#define __SSD1306_RES_SET(pGPIOCfg)     __gpio_write_pin(SET, __GET_PIN_CONFIG((pGPIOCfg), SPI1_RES_PIN), (pGPIOCfg))
#define __SSD1306_RES_CLR(pGPIOCfg)     __gpio_write_pin(RESET, __GET_PIN_CONFIG((pGPIOCfg), SPI1_RES_PIN), (pGPIOCfg))

#define __SSD1306_DC_SET(pGPIOCfg)      __gpio_write_pin(SET, __GET_PIN_CONFIG((pGPIOCfg), SPI1_DC_PIN), (pGPIOCfg))
#define __SSD1306_DC_CLR(pGPIOCfg)      __gpio_write_pin(RESET, __GET_PIN_CONFIG((pGPIOCfg), SPI1_DC_PIN), (pGPIOCfg))

#define __SSD1306_CLK_SET(pGPIOCfg)     __gpio_write_pin(SET, __GET_PIN_CONFIG((pGPIOCfg), SPI1_SCLK_PIN), (pGPIOCfg))
#define __SSD1306_CLK_CLR(pGPIOCfg)     __gpio_write_pin(RESET, __GET_PIN_CONFIG((pGPIOCfg), SPI1_SCLK_PIN), (pGPIOCfg))

#define __SSD1306_DIN_SET(pGPIOCfg)     __gpio_write_pin(SET, __GET_PIN_CONFIG((pGPIOCfg), SPI1_DIN_PIN), (pGPIOCfg))
#define __SSD1306_DIN_CLR(pGPIOCfg)     __gpio_write_pin(RESET, __GET_PIN_CONFIG((pGPIOCfg), SPI1_DIN_PIN), (pGPIOCfg))

/* Exported functions ------------------------------------------------------- */

extern void ssd1306_clear_screen(GPIO_ConfigDef *pGPIOCfg, SPI_ConfigDef *pSPICfg, uint8_t chFill);
extern void ssd1306_refresh_gram(GPIO_ConfigDef *pGPIOCfg, SPI_ConfigDef *pSPICfg);
extern void ssd1306_draw_point(uint8_t chXpos, uint8_t chYpos, uint8_t chPoint);
extern void ssd1306_fill_screen(GPIO_ConfigDef *pGPIOCfg, SPI_ConfigDef *pSPIConfig, uint8_t chXpos1, uint8_t chYpos1, uint8_t chXpos2, uint8_t chYpos2, uint8_t chDot);
extern void ssd1306_display_char(GPIO_ConfigDef *pGPIOCfg, SPI_ConfigDef *pSPIConfig, uint8_t chXpos, uint8_t chYpos, uint8_t chChr, uint8_t chSize, uint8_t chMode);
extern void ssd1306_display_num(GPIO_ConfigDef *pGPIOCfg, SPI_ConfigDef *pSPIConfig, uint8_t chXpos, uint8_t chYpos, uint32_t chNum, uint8_t chLen,uint8_t chSize);
extern void ssd1306_display_string(GPIO_ConfigDef *pGPIOCfg, SPI_ConfigDef *pSPIConfig, uint8_t chXpos, uint8_t chYpos, const uint8_t *pchString, uint8_t chSize, uint8_t chMode);
extern void ssd1306_draw_1616char(GPIO_ConfigDef *pGPIOCfg, SPI_ConfigDef *pSPIConfig, uint8_t chXpos, uint8_t chYpos, uint8_t chChar);
extern void ssd1306_draw_3216char(GPIO_ConfigDef *pGPIOCfg, SPI_ConfigDef *pSPIConfig, uint8_t chXpos, uint8_t chYpos, uint8_t chChar);
extern void ssd1306_draw_bitmap(GPIO_ConfigDef *pGPIOCfg, SPI_ConfigDef *pSPIConfig, uint8_t chXpos, uint8_t chYpos, const uint8_t *pchBmp, uint8_t chWidth, uint8_t chHeight);

extern void ssd1306_init(GPIO_ConfigDef *pGPIOCfg, SPI_ConfigDef *pSPIConfig);

#endif /* SSD1306_H_ */
