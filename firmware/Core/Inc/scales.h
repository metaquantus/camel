/*
 * scales.h
 *
 *  Created on: Oct 27, 2024
 *      Author: cav
 */

#ifndef INC_SCALES_H_
#define INC_SCALES_H_

#include "main.h"

typedef struct {
	GPIO_TypeDef *DOUT_Port;
	uint16_t DOUT_Pin;
	GPIO_TypeDef *SCK_Port;
	uint16_t SCK_Pin;
	// GAIN: 1, 2, 3, corresponding to 128, 32 or 64
	// channel B is not used, so valid values are 128 and 64
	uint8_t GAIN;
	uint8_t enabled;  // power off/on
	uint8_t modified; // for synchronization
} HX711_TypeDef ;

uint8_t HX711_isReady(HX711_TypeDef* cell);
void HX711_readScales(HX711_TypeDef* lcell, HX711_TypeDef* rcell);
void HX711_powerUp(HX711_TypeDef* cell);
void HX711_powerDown(HX711_TypeDef* cell);

#endif /* INC_SCALES_H_ */
