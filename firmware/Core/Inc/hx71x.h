/*
 * scales.h
 *
 *  Created on: Oct 27, 2024
 *      Author: cav
 */

#ifndef INC_HX71X_H_
#define INC_HX71X_H_

#include "main.h"

typedef struct {
	GPIO_TypeDef *DOUT_Port;
	uint16_t DOUT_Pin;
	GPIO_TypeDef *SCK_Port;
	uint16_t SCK_Pin;
	// HX711: (CAMEL1)
	// GAIN: 1, 2, 3, corresponding to 128, 32 or 64
	// channel B is not used, so valid values are only 128 and 64
	// HX712: (CAMEL2)
	// GAIN/FREQ: 1, 2, 3, 4, 5: 128/10Hz, BAT/40Hz, 128/40Hz, 256/10Hz, 256/40Hz
	uint8_t GAIN;
	uint8_t enabled;  // power off/on
	uint8_t modified; // for synchronization
} HX71x_TypeDef ;

uint8_t HX71x_isReady(HX71x_TypeDef* cell);
void HX71x_read(HX71x_TypeDef* lcell, HX71x_TypeDef* rcell);
void HX71x_powerUp(HX71x_TypeDef* cell);
void HX71x_powerDown(HX71x_TypeDef* cell);

#endif /* INC_HX71X_H_ */
