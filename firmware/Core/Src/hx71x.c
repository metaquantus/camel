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

uint8_t CAL_COUNT;
uint8_t CAL_OFFSET;
uint8_t TARE_TIMES;
uint8_t TARE_INDEX;
uint8_t LEFT_CAL_TIMES;
uint8_t LEFT_CAL_INDEX;
uint8_t RIGHT_CAL_TIMES;
uint8_t RIGHT_CAL_INDEX;
float SCALES_VALUE;
float LEFT_CAL_VALUE;
float RIGHT_CAL_VALUE;
long LEFT_TARE_OFFSET;
long RIGHT_TARE_OFFSET;

HX71x_TypeDef leftCell = { LEFT_DOUT_GPIO_Port, LEFT_DOUT_Pin, LEFT_SCK_GPIO_Port, LEFT_SCK_Pin };
HX71x_TypeDef rightCell= { RIGHT_DOUT_GPIO_Port, RIGHT_DOUT_Pin, RIGHT_SCK_GPIO_Port, RIGHT_SCK_Pin };

uint8_t SCALES_DATA[SCALES_DATA_SIZE] = { 0, 0, 0, 0, 0, 0 };

// cache, this will be stored in EEPROM also
// uint8_t CAL_DATA[CAMEL_CAL_DATA_SIZE];
CalibrationEntry_TypeDef CAL_DATA[CAL_DATA_MAX_COUNT];
DeviceConfiguration_TypeDef SCALES_CONFIG = { 0x33, 0, 0, 1, 0xFF };

typedef struct {
  uint8_t leftValue;
  uint8_t rightValue;
} CellReadValue_TypeDef;

void HX71x_init() {
  SCALES_CONFIG.config.value = 0x33;
  SCALES_CONFIG.count = 0;
  SCALES_CONFIG.mode = 0;
  SCALES_CONFIG.prog = 4; // 256 gain/10Hz
}

void HX71x_readConfig(void) {
  // 1st byte stores cells configuration, 2nd byte is calibration data point count sent by host
  // we should keep the count and send it back when requested.
  // we store also a CRC to validate EEPROM DATA
  uint8_t config[4] = {0 , 0, 0, 0};
  SCALES_CONFIG = eeprom_read_byte(CAMEL_CONFIG_EEPROM_START);
  CAL_COUNT = eeprom_read_byte(CAMEL_CONFIG_EEPROM_START+1);
  READ_VALUE_MODE = eeprom_read_byte(CAMEL_CONFIG_EEPROM_START+2);
  uint8_t scrc = eeprom_read_byte(CAMEL_CONFIG_EEPROM_START+3);
  config[0] = SCALES_CONFIG;
  config[1] = CAL_COUNT;
  config[2] = READ_VALUE_MODE;

  uint8_t crc = GENCRC(config, 3);
  if ( crc != scrc ) {
    // wrong CRC
    // load default value
    SCALES_CONFIG = DEFAULT_CONFIG;
    CAL_COUNT = 0;
    READ_VALUE_MODE = 0;
  }
  if ( CAL_COUNT > CAMEL_CAL_DATA_COUNT ) {
    CAL_COUNT = CAMEL_CAL_DATA_COUNT;
  }
  if ( READ_VALUE_MODE > 2 ) {
    READ_VALUE_MODE = 0;
  }
  TARE_TIMES = 0;
  LEFT_CAL_TIMES = 0;
  RIGHT_CAL_TIMES = 0;
  LEFT_TARE_OFFSET = 0;
  RIGHT_TARE_OFFSET = 0;

  FUNC_FLAG = CONFIG_MASK;


  // EEPROM_DATA is a shadow in SRAM of EEPROM calibration data as EEPROM NVM is slower to read and write
  // no check is done on data, the host should do the checking as format is host specific
  // memcpy(EEPROM_DATA, (const void*) (DATA_EEPROM_BASE + CAMEL_EEPROM_START), EEPROM_DATA_SIZE);
  // copy only valid data
  if ( CAL_COUNT > 0 && CAL_COUNT < CAMEL_CAL_DATA_COUNT ) {
    for(size_t i=0; i<CAL_COUNT; i++) {
      uint32_t offset = i << 3; // i * 8
      uint32_t value = eeprom_read_word(CAMEL_LEFT_EEPROM_START + offset);
      // memcpy(CAL_DATA + offset, &value, 4);
      * (uint32_t *) (CAL_DATA + offset) = value;
      value = eeprom_read_word(CAMEL_LEFT_EEPROM_START + offset + 4);
      // memcpy(CAL_DATA + offset + 4, &value, 4);
      * (uint32_t *)(CAL_DATA + offset + 4) = value;
      value = eeprom_read_word(CAMEL_RIGHT_EEPROM_START + offset);
      // memcpy(CAL_DATA + CAMEL_CAL_RIGHT_OFFSET + offset, &value, 4);
      * (uint32_t *)(CAL_DATA + CAMEL_CAL_RIGHT_OFFSET + offset) = value;
      value = eeprom_read_word(CAMEL_RIGHT_EEPROM_START + offset + 4);
      // memcpy(CAL_DATA + CAMEL_CAL_RIGHT_OFFSET + offset + 4, &value, 4);
      * (uint32_t *)(CAL_DATA + CAMEL_CAL_RIGHT_OFFSET + offset + 4) = value;
    }
  }
}

// reads one byte from both left and right HX711s
CellReadValue_TypeDef shiftIn() {
  uint16_t lvalue = 0;
  uint16_t rvalue = 0;
  CellReadValue_TypeDef result;
  uint8_t i;

  for (i = 0; i < 8; ++i) {
    // We do it this way to give more time to the signals,
    // At 32MHz MCU clock, we need at least 4 clock cycles to meet minimum value required by HX711 (0.1us)
    // for rising edge to value sampling
    // asm("NOP\n\tNOP\n\tNOP\n\tNOP");
    if (SCALES_CONFIG.config.V.leftEnable ) {
      HAL_GPIO_WritePin(leftCell.SCK_Port, leftCell.SCK_Pin, GPIO_PIN_SET);
    }
    if (SCALES_CONFIG.config.V.rightEnable) {
      HAL_GPIO_WritePin(rightCell.SCK_Port, rightCell.SCK_Pin, GPIO_PIN_SET);
    }
    asm("NOP\n\tNOP\n\tNOP\n\tNOP\n\tNOP\n\tNOP");
    if (SCALES_CONFIG.config.V.leftEnable) {
      lvalue |= HAL_GPIO_ReadPin(leftCell.DOUT_Port, leftCell.DOUT_Pin) << (7 - i);
    }
    if (SCALES_CONFIG.config.V.rightEnable) {
      rvalue |= HAL_GPIO_ReadPin(rightCell.DOUT_Port, rightCell.DOUT_Pin) << (7 - i);
    }
    // asm("NOP\n\tNOP\n\tNOP\n\tNOP");
    if (SCALES_CONFIG.config.V.leftEnable) {
      HAL_GPIO_WritePin(leftCell.SCK_Port, leftCell.SCK_Pin, GPIO_PIN_RESET);
    }
    if (SCALES_CONFIG.config.V.rightEnable ) {
      HAL_GPIO_WritePin(rightCell.SCK_Port, rightCell.SCK_Pin, GPIO_PIN_RESET);
    }
    asm("NOP\n\tNOP\n\tNOP\n\tNOP\n\tNOP\n\tNOP");
  }
  // TODO: return a struct for easy result handling
  // left value in MSB
  result.leftValue = lvalue;
  result.rightValue = rvalue;
  return result;
}

void HX71x_read(HX71x_TypeDef* lcell, HX71x_TypeDef* rcell) {
	// cells should be in ready state if enabled
	CellReadValue_TypeDef data[3] = { {0,0}, {0,0}, {0,0} };

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

	// TODO: use a proper structure to store SCALES_DATA and CAL_DATA
	// so we don't have to do this byte juggling
	// also store left and right values continuously for each calibration point
	// rather than first all lefts and then all rights

	// copy raw data to global buffer
	// left
	SCALES_DATA[0] = data[0].leftValue;
	SCALES_DATA[1] = data[1].leftValue;
	SCALES_DATA[2] = data[2].leftValue;
	// right
	SCALES_DATA[3] = data[0].rightValue;
	SCALES_DATA[4] = data[1].rightValue;
	SCALES_DATA[5] = data[2].rightValue;

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
		  // this is wrong if LEFT_CAL_INDEX == 0, this is the first value
		  // so we should not look at the previous value
		  // we may have garbage in CAL_DATA to start with
			float scale = leftScale;

			if ( LEFT_CAL_INDEX > 0 ) { // otherwise this is the first value
			  long c1 = CAL_DATA[CAL_OFFSET + 4];
			  long c2 = CAL_DATA[CAL_OFFSET + 5];
			  c2 = c2 << 8;
			  long c3 = CAL_DATA[CAL_OFFSET + 6];
			  c3 = c3 << 16;
        long cvalue = c1 | c2 | c3;
        if ((cvalue & 0x00800000) != 0) {
          cvalue |= 0xFF000000;
        }
        // average raw values by accumulating
        value = (cvalue + value) / 2;
			}

      if (CAL_OFFSET == 0) {
        // use only this value
        if ( (value - LEFT_TARE_OFFSET) != 0 ) {
          scale = LEFT_CAL_VALUE / (value - LEFT_TARE_OFFSET);
        } else {
          scale = 1;
        }
      } else {
        // use this value and the previous calibration point to compute the current scale
        // previous calibration point must be set before this

        // get the index - 1 (previous) raw value
        long p1 = CAL_DATA[CAL_OFFSET - CAMEL_CAL_POINT_SIZE + 4];
        long p2 = CAL_DATA[CAL_OFFSET - CAMEL_CAL_POINT_SIZE + 5];
        p2 = p2 << 8;
        long p3 = CAL_DATA[CAL_OFFSET - CAMEL_CAL_POINT_SIZE + 6];
        p3 = p3 << 16;
        long pvalue = p1 | p2 | p3;
        if ((pvalue & 0x00800000) != 0) {
          pvalue |= 0xFF000000;
        }
        // get the index - 1 (previous) scale factor
        float pscale = *(float*) (CAL_DATA + CAL_OFFSET - CAMEL_CAL_POINT_SIZE);
        // memcpy(&pscale, CAL_DATA + CAL_OFFSET - CAMEL_CAL_POINT_SIZE, 4);

        // compute index - 1 (previous, less weight) calibrated value
        // TODO: maybe store also the original weight, so we don't have to recompute it
        float w0 = (pvalue - LEFT_TARE_OFFSET) * pscale;
        // finally, compute the current scale value,
        // which is the slope of the line segment from previous calibration point to current point
        if ( (value - pvalue) != 0 ) {
          scale = (LEFT_CAL_VALUE - w0) / (value - pvalue);
        } else {
          scale = 1;
        }
      }
      // TODO: rethink the calibration data storage layout to simplify handling
      // copy to memory
      // memcpy(CAL_DATA + CAL_OFFSET, &scale, 4);
      *(float*) (CAL_DATA + CAL_OFFSET) = scale;
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
			if ( RIGHT_CAL_INDEX > 0 ) {
        long cvalue = CAL_DATA[CAL_OFFSET + CAMEL_CAL_RIGHT_OFFSET + 4] | (CAL_DATA[CAL_OFFSET + CAMEL_CAL_RIGHT_OFFSET + 5] << 8)
            | (CAL_DATA[CAL_OFFSET + CAMEL_CAL_RIGHT_OFFSET + 6] << 16);
        if ((cvalue & 0x00800000) != 0) {
          cvalue |= 0xFF000000;
        }
        // average raw values cumulatively
        value = (value + cvalue) / 2;
			}
      if (CAL_OFFSET == 0) {
        // there is no previous calibration point (or is zero, the tare value)
        scale = RIGHT_CAL_VALUE / (value - RIGHT_TARE_OFFSET);
      } else {
        // get the index - 1 (previous) raw value
        long pvalue = CAL_DATA[CAL_OFFSET + CAMEL_CAL_RIGHT_OFFSET - CAMEL_CAL_POINT_SIZE + 4]
            | (CAL_DATA[CAL_OFFSET + CAMEL_CAL_RIGHT_OFFSET - CAMEL_CAL_POINT_SIZE + 5] << 8)
            | (CAL_DATA[CAL_OFFSET + CAMEL_CAL_RIGHT_OFFSET - CAMEL_CAL_POINT_SIZE + 6] << 16);
        if ((pvalue & 0x00800000) != 0) {
          pvalue |= 0xFF000000;
        }
        // get the index - 1 (previous) scale factor
        float pscale = *(float*) (CAL_DATA + CAMEL_CAL_RIGHT_OFFSET + CAL_OFFSET - CAMEL_CAL_POINT_SIZE);
        // memcpy(&pscale, CAL_DATA + CAMEL_CAL_RIGHT_OFFSET + CAL_OFFSET - CAMEL_CAL_POINT_SIZE, 4);

        // compute index - 1 (previous, less weight) calibrated value
        float w0 = (pvalue - RIGHT_TARE_OFFSET) * pscale;
        // finally, compute the current scale value,
        // which is the slope of the line segment from previous calibration point to current point
        scale = (RIGHT_CAL_VALUE - w0) / (value - pvalue);
      }

			// store right calibration data
      // memcpy(CAL_DATA + CAMEL_CAL_RIGHT_OFFSET + CAL_OFFSET, &scale, 4);
      *(float*) (CAL_DATA + CAMEL_CAL_RIGHT_OFFSET + CAL_OFFSET) = scale;
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
			  // disable this for now, handle it in the command processor
				// FUNC_FLAG = LEFT_MASK;
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
			  // disable this for now, handle it in the command processor
				// FUNC_FLAG = RIGHT_MASK;
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
