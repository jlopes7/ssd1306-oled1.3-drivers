/**
 ******************************************************************************
 * @file           : waveshare-oled1.3.h
 * @author         : J0a0 Gonzalez
 * @brief          : Concentrates all the base configurations that need to be
 * 					 done for the WaveShare OLED 1.3 display using SPI 4 channels.
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
#ifndef WAVESHARE_OLED1_3_H_
#define WAVESHARE_OLED1_3_H_

#include "ssd1306.h"
#include "stm32f401xx_systick.driver.h"

/* DTSs for the SPI/GPIO configuration ***********************************************/
typedef struct {
	GPIO_ConfigDef *pGPIOConfiguration;
	SPI_ConfigDef  *pSPIConfiguration;
} SSD1306_Config_t;

/* Specific functions prototypes       ***********************************************/
SSD1306_Config_t ws_sys_start();

#endif /* WAVESHARE_OLED1_3_H_ */
