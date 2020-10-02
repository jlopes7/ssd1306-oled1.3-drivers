/**
 ******************************************************************************
 * @file           : Fonts.h
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
#ifndef FONTS_H_
#define FONTS_H_

#include <stdint.h>

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
extern const uint8_t c_chFont1206[95][12];
extern const uint8_t c_chFont1608[95][16];
extern const uint8_t c_chFont1612[11][32];
extern const uint8_t c_chFont3216[11][64];
extern const uint8_t c_chBmp4016[96];
extern const uint8_t c_chSingal816[16];
extern const uint8_t c_chMsg816[16];
extern const uint8_t c_chBluetooth88[8];
extern const uint8_t c_chBat816[16];
extern const uint8_t c_chGPRS88[8];
extern const uint8_t c_chAlarm88[8];

#endif /* FONTS_H_ */
