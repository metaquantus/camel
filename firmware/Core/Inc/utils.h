/*
 * utils.h
 *
 *  Created on: Jul 23, 2025
 *      Author: cav
 */

#ifndef INC_UTILS_H_
#define INC_UTILS_H_
#include "stm32l0xx_hal.h"

uint8_t _gencrc(uint8_t *data, size_t len);
float atoff(const char *nptr);
void fftoa(float f, char *str, uint8_t precision);

/* be sure CRC handle is defined before using this macro */
#ifdef HAL_CRC_MODULE_ENABLED
#define GENCRC(buf, len) (uint8_t)(HAL_CRC_Calculate(&hcrc, (uint32_t *) (buf) , (size_t) (len) ) & 0xFF)
#else
#define GENCRC(buf, len) _gencrc((uint8_t *) (buf) , (size_t) (len) )
#endif

#endif /* INC_UTILS_H_ */
