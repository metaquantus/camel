/*
 * scales.c
 *
 *  Created on: Oct 27, 2024
 *      Author: cav
 */
#include <hx71x.h>
#include "main.h"
#include "eeprom.h"
#include <string.h>
#include "utils.h"

/*
 * HX711s are wired at 10 samples per second, channel B is not used.
 * HX712 can sample at 10Hz or 40Hz controlled by configuration.
 * Each HX71x has its own separate PD_SCK clock signal, so it can be power down if not used.
 * Power down is done by setting its PD_SCK to HIGH for more than 60us.
 * This MCU runs at 32MHz (31ns clock period), some extra delay is added using ASM NOPs.
 * The main loop waits for both left and right HX711s to be ready, so they can be read together.
 * So they will be ready about the same time on the next reading.
 * Data is read and copied to the global buffer.
 * There's the issue of the read routine being interrupted and the risk of the HX711s getting power down while servicing the ISR,
 * since we have the I2C subsystem using interrupt mode. However, we assume the I2C IRQ handlers don't take more than 60us.
 */
extern CRC_HandleTypeDef hcrc;

extern uint8_t FUNC_FLAG;
extern uint8_t SCALES_DATA[SCALES_DATA_SIZE];
extern uint8_t CAL_COUNT;
extern uint8_t CAL_OFFSET;
extern uint8_t CAL_DATA[CAMEL_CAL_DATA_SIZE];
extern uint8_t TARE_TIMES;
extern uint8_t TARE_INDEX;
extern uint8_t LEFT_CAL_TIMES;
extern uint8_t LEFT_CAL_INDEX;
extern uint8_t RIGHT_CAL_TIMES;
extern uint8_t RIGHT_CAL_INDEX;
extern float SCALES_VALUE;
extern float LEFT_CAL_VALUE;
extern float RIGHT_CAL_VALUE;
extern long LEFT_TARE_OFFSET;
extern long RIGHT_TARE_OFFSET;


// reads one byte from both left and right HX711s
uint16_t shiftIn(HX71x_TypeDef *lcell, HX71x_TypeDef *rcell) {
  uint16_t lvalue = 0;
  uint16_t rvalue = 0;
  uint8_t i;

  for (i = 0; i < 8; ++i) {
    // We do it this way to give more time to the signals,
    // At 32MHz MCU clock, we need at least 4 clock cycles to meet minimum value required by HX711 (0.1us)
    // for rising edge to value sampling
    // asm("NOP\n\tNOP\n\tNOP\n\tNOP");
    if (lcell->enabled) {
      HAL_GPIO_WritePin(lcell->SCK_Port, lcell->SCK_Pin, GPIO_PIN_SET);
    }
    if (rcell->enabled) {
      HAL_GPIO_WritePin(rcell->SCK_Port, rcell->SCK_Pin, GPIO_PIN_SET);
    }
    asm("NOP\n\tNOP\n\tNOP\n\tNOP\n\tNOP\n\tNOP");
    if (lcell->enabled) {
      lvalue |= HAL_GPIO_ReadPin(lcell->DOUT_Port, lcell->DOUT_Pin) << (7 - i);
    }
    if (rcell->enabled) {
      rvalue |= HAL_GPIO_ReadPin(rcell->DOUT_Port, rcell->DOUT_Pin) << (7 - i);
    }
    // asm("NOP\n\tNOP\n\tNOP\n\tNOP");
    if (lcell->enabled) {
      HAL_GPIO_WritePin(lcell->SCK_Port, lcell->SCK_Pin, GPIO_PIN_RESET);
    }
    if (rcell->enabled) {
      HAL_GPIO_WritePin(rcell->SCK_Port, rcell->SCK_Pin, GPIO_PIN_RESET);
    }
    asm("NOP\n\tNOP\n\tNOP\n\tNOP\n\tNOP\n\tNOP");
  }
  // left value in MSB
  return (lvalue << 8) | rvalue;
}

void HX71x_read(HX71x_TypeDef* lcell, HX71x_TypeDef* rcell) {
	// cells should be in ready state if enabled
	uint16_t data[3] = {0, 0, 0};

	// as long as interrupts don't take longer than 60us, interrupts can be enabled
	// disable interrupts
	// noInterrupts();

	//make sure clock is enabled
	if (lcell->enabled ) {
		HAL_GPIO_WritePin(lcell->SCK_Port, lcell->SCK_Pin, GPIO_PIN_RESET);
	}
	if (rcell->enabled ) {
		HAL_GPIO_WritePin(rcell->SCK_Port, rcell->SCK_Pin, GPIO_PIN_RESET);
	}

	// Pulse the clock pin 24 times to read the data.
	data[2] = shiftIn(lcell, rcell);
	data[1] = shiftIn(lcell, rcell);
	data[0] = shiftIn(lcell, rcell);

	// Set the channel and the gain factor for the next reading using the clock pin.
	// GAIN should be 1 or 3 for hx711 or 1,3,4 or 5 for hx712
	for (unsigned int i = 0; i < lcell->GAIN || i < rcell->GAIN; i++) {
		asm("NOP\n\tNOP\n\tNOP\n\tNOP");
		if ( lcell->enabled && i < lcell->GAIN) {
			HAL_GPIO_WritePin(lcell->SCK_Port, lcell->SCK_Pin, GPIO_PIN_SET);
		}
		if ( rcell->enabled && i < rcell->GAIN) {
			HAL_GPIO_WritePin(rcell->SCK_Port, rcell->SCK_Pin, GPIO_PIN_SET);
		}
		asm("NOP\n\tNOP\n\tNOP\n\tNOP\n\tNOP\n\tNOP");
		if (lcell->enabled && i < lcell->GAIN ) {
			HAL_GPIO_WritePin(lcell->SCK_Port, lcell->SCK_Pin, GPIO_PIN_RESET);
		}
		if (rcell->enabled && i < rcell->GAIN ) {
			HAL_GPIO_WritePin(rcell->SCK_Port, rcell->SCK_Pin, GPIO_PIN_RESET);
		}
	}
	// ADCs should start sampling the next value now
	// We can then perform other operations while ADCs work

	// enable interrupts
	// interrupts();

	// copy raw data to global buffer
	// left
	SCALES_DATA[0] = (data[0] >> 8) & 0xFF;
	SCALES_DATA[1] = (data[1] >> 8) & 0xFF;
	SCALES_DATA[2] = (data[2] >> 8) & 0xFF;
	// right
	SCALES_DATA[3] = data[0] & 0xFF;
	SCALES_DATA[4] = data[1] & 0xFF;
	SCALES_DATA[5] = data[2] & 0xFF;

	// compute the scaled calibrated value
	float lvalue = 0.0;
	float rvalue = 0.0;
	if (lcell->enabled ) {
		long value = SCALES_DATA[0] | (SCALES_DATA[1] << 8) | (SCALES_DATA[2] << 16);
		if ( (SCALES_DATA[2] & 0x80) != 0 ) {
			value |= 0xFF000000; // extend sign
		}
		float leftScale = 1;
		// find the calibration scale factor for target value
		// calibration points should be in ascending absolute values
		// closest to current reading is chosen
		for (int i = 0; i < CAL_COUNT; i++) {
			leftScale = * (float *) (CAL_DATA + (i << 3));
			long cvalue = (* (long *) (CAL_DATA + (i << 3) + 4) ) & 0x00FFFFFF; // fourth byte may be a CRC
			// value is 24 bits, extend sign
			if ( (cvalue & 0x00800000) != 0 ) {
				cvalue |= 0xFF000000;
			}
			// value may be negative depending on how the cell is mechanically connected (bending in opposite direction)
			if ( value >= 0) {
				if ( value <=  cvalue) {
					break;
				}
			} else {
				if ( value >= cvalue) {
					break;
				}
			}
		}
		// tare
		if ( TARE_TIMES > 0 ) {
			if ( TARE_INDEX == 0 ) {
				LEFT_TARE_OFFSET = value;
			} else {
				// average
				LEFT_TARE_OFFSET = (LEFT_TARE_OFFSET + value) / 2;
			}
		}
		// compute value
		lvalue = (value - LEFT_TARE_OFFSET) * leftScale;
#ifdef CAMEL_LOCAL_CALIBRATION
		// compute new left scale factor if in calibration mode
		if ( LEFT_CAL_TIMES > 0 ) {

			float scale = leftScale;
			if ( CAL_OFFSET == 0 ) {
				scale = LEFT_CAL_VALUE / (value - LEFT_TARE_OFFSET);
			} else {
				// get the current raw value
				long cvalue = CAL_DATA[CAL_OFFSET + 4]
						| (CAL_DATA[CAL_OFFSET + 5] << 8)
						| (CAL_DATA[CAL_OFFSET + 6] << 16);
				if ( (cvalue & 0x00800000) != 0 ) {
					cvalue |= 0xFF000000;
				}
				// get the index - 1 (previous) raw value
				long pvalue = CAL_DATA[CAL_OFFSET - CAMEL_CAL_POINT_SIZE + 4]
					|  (CAL_DATA[CAL_OFFSET - CAMEL_CAL_POINT_SIZE + 5] << 8)
					|  (CAL_DATA[CAL_OFFSET - CAMEL_CAL_POINT_SIZE + 6] << 16) ;
				if ( (pvalue & 0x00800000) != 0 ) {
					pvalue |= 0xFF000000;
				}
				// get the index - 1 (previous) scale factor
				float pscale = * (float *)(CAL_DATA + CAL_OFFSET - CAMEL_CAL_POINT_SIZE);
				// memcpy(&pscale, CAL_DATA + CAL_OFFSET - CAMEL_CAL_POINT_SIZE, 4);
				// average raw values
				value = (cvalue + value) / 2;
				// compute index - 1 (previous, less weight) calibrated value
				float w0 = (pvalue - LEFT_TARE_OFFSET) * pscale;
				// finally, compute the current scale value,
				// which is the slope of the line segment from previous calibration point to current point
				scale = (LEFT_CAL_VALUE - w0) / (value - pvalue);
			}
			// memcpy(CAL_DATA + CAL_OFFSET, &scale, 4);
			* (float *)(CAL_DATA + CAL_OFFSET) = scale;
			CAL_DATA[CAL_OFFSET + 4] = value & 0xFF;
			CAL_DATA[CAL_OFFSET + 5] = (value >> 8) & 0xFF;
			CAL_DATA[CAL_OFFSET + 6] = (value >> 16) & 0xFF;
			CAL_DATA[CAL_OFFSET + 7] = GENCRC(CAL_DATA + CAL_OFFSET, 7); // CRC-8
		}
#endif
	}
	if ( rcell->enabled ) {
		long value = SCALES_DATA[3] | (SCALES_DATA[4] << 8) | (SCALES_DATA[5] << 16);
		if ((SCALES_DATA[5] & 0x80) != 0) {
			value |= 0xFF000000;
		}
		float rightScale = 1;
		for (int i = 0; i < CAL_COUNT; i++) {
			rightScale = *(float*) (CAL_DATA + CAMEL_CAL_RIGHT_OFFSET + (i << 3));
			long cvalue = (*(long*) (CAL_DATA + CAMEL_CAL_RIGHT_OFFSET + (i << 3) + 4) ) & 0x00FFFFFF;
			if ( (cvalue & 0x00800000) != 0) {
				cvalue |= 0xFF000000;
			}
			if (value >= 0) {
				if (value <= cvalue) {
					break;
				}
			} else {
				if (value >= cvalue) {
					break;
				}
			}
		}
		if (TARE_TIMES > 0) {
			if (TARE_INDEX == 0) {
				RIGHT_TARE_OFFSET = value;
			} else {
				// average
				RIGHT_TARE_OFFSET = (RIGHT_TARE_OFFSET + value) / 2;
			}
		}

		rvalue = (value - RIGHT_TARE_OFFSET) * rightScale;
#ifdef CAMEL_LOCAL_CALIBRATION
		if (RIGHT_CAL_TIMES > 0) {

			float scale = rightScale;
			if (CAL_OFFSET == 0) {
				scale = RIGHT_CAL_VALUE / (value - RIGHT_TARE_OFFSET);
			} else {
				// get the current raw value
				long cvalue = CAL_DATA[CAL_OFFSET + CAMEL_CAL_RIGHT_OFFSET + 4]
						| (CAL_DATA[CAL_OFFSET + CAMEL_CAL_RIGHT_OFFSET + 5] << 8)
						| (CAL_DATA[CAL_OFFSET + CAMEL_CAL_RIGHT_OFFSET + 6] << 16);
				if ((cvalue & 0x00800000) != 0) {
					cvalue |= 0xFF000000;
				}
				// get the index - 1 (previous) raw value
				long pvalue = CAL_DATA[CAL_OFFSET + CAMEL_CAL_RIGHT_OFFSET - CAMEL_CAL_POINT_SIZE + 4]
						| (CAL_DATA[CAL_OFFSET + CAMEL_CAL_RIGHT_OFFSET - CAMEL_CAL_POINT_SIZE + 5] << 8)
						| (CAL_DATA[CAL_OFFSET + CAMEL_CAL_RIGHT_OFFSET - CAMEL_CAL_POINT_SIZE + 6] << 16);
				if ((pvalue & 0x00800000) != 0) {
					pvalue |= 0xFF000000;
				}
				// get the index - 1 (previous) scale factor
				float pscale = * (float *) (CAL_DATA + CAMEL_CAL_RIGHT_OFFSET + CAL_OFFSET - CAMEL_CAL_POINT_SIZE);
				// memcpy(&pscale, CAL_DATA + CAMEL_CAL_RIGHT_OFFSET + CAL_OFFSET - CAMEL_CAL_POINT_SIZE, 4);
				// average raw values
				value = (cvalue + value) / 2;
				// compute index - 1 (previous, less weight) calibrated value
				float w0 = (pvalue - RIGHT_TARE_OFFSET) * pscale;
				// finally, compute the current scale value,
				// which is the slope of the line segment from previous calibration point to current point
				scale = (RIGHT_CAL_VALUE - w0) / (value - pvalue);
			}
			// memcpy(CAL_DATA + CAMEL_CAL_RIGHT_OFFSET + CAL_OFFSET, &scale, 4);
			* (float *) (CAL_DATA + CAMEL_CAL_RIGHT_OFFSET + CAL_OFFSET) = scale;
			CAL_DATA[CAL_OFFSET + CAMEL_CAL_RIGHT_OFFSET + 4] = value & 0xFF;
			CAL_DATA[CAL_OFFSET + CAMEL_CAL_RIGHT_OFFSET + 5] = (value >> 8) & 0xFF;
			CAL_DATA[CAL_OFFSET + CAMEL_CAL_RIGHT_OFFSET + 6] = (value >> 16) & 0xFF;
			CAL_DATA[CAL_OFFSET + CAMEL_CAL_RIGHT_OFFSET + 7] = GENCRC(CAL_DATA + CAMEL_CAL_RIGHT_OFFSET + CAL_OFFSET, 7); // CRC-8
		}
#endif
	}
	if ( TARE_TIMES > 0 ) {
		TARE_INDEX++;
		if ( TARE_INDEX >= TARE_TIMES ) {
			// finish tare operation and reset values
			TARE_TIMES = 0;
			TARE_INDEX = 0;
		}
	}
	if ( LEFT_CAL_TIMES > 0 ) {
		LEFT_CAL_INDEX++;
		if ( LEFT_CAL_INDEX >= LEFT_CAL_TIMES ) {
			LEFT_CAL_TIMES = 0;
			LEFT_CAL_INDEX = 0;
			if ( lcell->enabled ) {
				// this will store calibration data to EEPROM
				FUNC_FLAG = LEFT_MASK;
			}
		}
	}
	if (RIGHT_CAL_TIMES > 0) {
		RIGHT_CAL_INDEX++;
		if (RIGHT_CAL_INDEX >= RIGHT_CAL_TIMES) {
			RIGHT_CAL_TIMES = 0;
			RIGHT_CAL_INDEX = 0;
			if ( rcell-> enabled ) {
				// this will store calibration data to EEPROM
				FUNC_FLAG = RIGHT_MASK;
			}
		}
	}
	SCALES_VALUE = lvalue + rvalue;
}


uint8_t HX71x_isReady(HX71x_TypeDef* cell) {
	return HAL_GPIO_ReadPin(cell->DOUT_Port, cell->DOUT_Pin) == GPIO_PIN_RESET;
}

void HX71x_powerUp(HX71x_TypeDef* cell) {
	HAL_GPIO_WritePin(cell->SCK_Port, cell->SCK_Pin, GPIO_PIN_RESET);
}

void HX71x_powerDown(HX71x_TypeDef* cell) {
	HAL_GPIO_WritePin(cell->SCK_Port, cell->SCK_Pin, GPIO_PIN_RESET);
	asm("NOP\n\tNOP\n\tNOP\n\tNOP\n\tNOP\n\tNOP");
	HAL_GPIO_WritePin(cell->SCK_Port, cell->SCK_Pin, GPIO_PIN_SET);
	// it will power down if SCK remains high for 60us
	// this will be the case if no more commands from the host override it
}
