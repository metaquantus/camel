/*
 * scales.h
 *
 *  Created on: Oct 27, 2024
 *      Author: cav
 */

#ifndef INC_HX71X_H_
#define INC_HX71X_H_

#include "main.h"

#define CAL_DATA_MAX_COUNT 4

typedef struct {
	GPIO_TypeDef *DOUT_Port;
	uint16_t DOUT_Pin;
	GPIO_TypeDef *SCK_Port;
	uint16_t SCK_Pin;
	/* Move this to DeviceConfiguration
	// HX711: (CAMEL1)
	// GAIN: 1, 2, 3, corresponding to 128, 32 or 64
	// channel B is not used, so valid values are only 128 and 64
	// HX712: (CAMEL2)
	// GAIN/FREQ: 1, 2, 3, 4, 5: 128/10Hz, BAT/40Hz, 128/40Hz, 256/10Hz, 256/40Hz
	uint8_t GAIN;
	uint8_t enabled;  // power off/on
	uint8_t modified; // for synchronization
	*/
} HX71x_TypeDef ;

typedef struct {
  uint32_t flag : 24; // Miscellaneous flags, not used
  uint32_t crc : 8;   // CRC-8
} CalibrationPointInfo_TypeDef;

/**
 * Calibration Point
 * Used to compute weight value from cell measured value.
 * 16bytes per point.
 * This will be stored in EEPROM for persistence with a copy in RAM for runtime use.
 */
typedef struct {
  float scale;    // scale factor
  uint32_t value; // cell value, HX712/HX711 ADCs have 24bits resolution so only 3 bytes sent to host
  float weight;   // original calibration weight
  CalibrationPointInfo_TypeDef info; // extra metadata info
} CalibrationPoint_TypeDef ;

/**
 * Calibration Entry
 * Keep Left and Right values for a given calibration point together.
 * 32bytes per point.
 * We need 128bytes for 4 calibration points.
 */
typedef struct {
  CalibrationPoint_TypeDef Left;
  CalibrationPoint_TypeDef Right;
} CalibrationEntry_TypeDef ;

/**
 * HX712 ADCs configuration
 */
typedef struct {
  uint8_t rightEnable : 1;
  uint8_t rightGain: 1;  // 0: 128, 1: 256 for HX712 or 0: 64, 1: 128 for HX711
  uint8_t rightRate: 1;  // 0: 10Hz, 1: 40Hz for HX712
  uint8_t rightModified: 1; // for keeping track of applied changes
  uint8_t leftEnable : 1;
  uint8_t leftGain : 1;
  uint8_t leftRate : 1;
  uint8_t leftModified: 1;
} CellConfiguration_TypeDef;

typedef union {
  uint8_t value;
  CellConfiguration_TypeDef V;
} CellConfigurationUnion_TypeDef;
/**
 * Cell and device configuration for HX712. HX711 uses only 2 bits.
 * The config value stores cell configuration in a compact way using up to 4-bits for each cell.
 * Currently 3-bits used:
 * bit 0: cell enable
 * bit 1: gain factor 128 or 256
 * bit 2: sampling rate 10Hz or 40Hz
 * bit 3: reserved
 */
typedef struct {
  CellConfigurationUnion_TypeDef config;  // cell configuration 4-bit MSB for left, 4-bit LSB for right
  uint8_t count : 4;   // calibration count 0 to 4, calibration points in CAL_DATA and EEPROM
  uint8_t mode : 4;    // read mode for sending data to host: 0 (6bytes cell value only), 1 (4bytes weight value only),
                       // 2 (10bytes, both)
  uint8_t prog;        // number of pulses used to program the HX71x into the correct gain and sampling rate:
                       // 1, 2, 3, 4, 5: 128/10Hz, BAT/40Hz, 128/40Hz, 256/10Hz, 256/40Hz
  uint8_t crc;         // CRC-8
} DeviceConfiguration_TypeDef;

uint8_t HX71x_isReady(HX71x_TypeDef* cell);
void HX71x_read(HX71x_TypeDef* lcell, HX71x_TypeDef* rcell);
void HX71x_powerUp(HX71x_TypeDef* cell);
void HX71x_powerDown(HX71x_TypeDef* cell);
void HX71x_readConfig(); // read device configuration from EEPROM to transient configuration in memory
void HX71x_processConfig(); // process configuration commands sent via global variables (change config, save config to EEPROM, etc)

#endif /* INC_HX71X_H_ */
