/*
 * eeprom.h
 *
 *  Created on: Nov 17, 2024
 *      Author: cav
 *
 *  Non-volatile memory functions used to store calibration data for
 *  the load cells.
 *  We take 68 bytes at the end of the EEPROM memory to store the data
 *
 */

#ifndef INC_EEPROM_H_
#define INC_EEPROM_H_

#include "stm32l0xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

// In stm32l071xx, EEPROM is divided in 2 banks, one bank is more than enough
#ifndef DATA_EEPROM_END
#define DATA_EEPROM_END DATA_EEPROM_BANK1_END
#endif

/*
 * Reserve 128+8 bytes of EEPROM to store configuration and cell calibration data.
 * Calibration of up to 4 points can be stored for each cell, each 16 bytes wide, intended to
 * hold 4 bytes for a floating point scale factor value, a 3 bytes load cell value
 * and 1 byte for CRC. The calibration data is not checked, is simply stored and loaded
 * and passed to the host as is.
 * 4 additional bytes are reserved to store cell configuration,
 * 1 byte for the configuration data, 1 byte for the calibration count,
 * 1 byte for the read value mode and 1 byte for a CRC.
 *
 * The following constants are offsets relative to the start of the EEPROM memory (DATA_EEPROM_BASE).
 */
#define CAMEL_CAL_DATA_COUNT      4
#define CAMEL_CAL_DATA_SIZE       128
// #define CAMEL_CAL_RIGHT_OFFSET    64

#define CAMEL_CAL_POINT_SIZE      16
#define CAMEL_EEPROM_SIZE         (CAMEL_CAL_DATA_SIZE + 8)
#define CAMEL_EEPROM_START2       (DATA_EEPROM_END - DATA_EEPROM_BASE - CAMEL_EEPROM_SIZE + 1)
#define CAMEL_EEPROM_START        1024
// #define CAMEL_LEFT_EEPROM_START   CAMEL_EEPROM_START
// #define CAMEL_RIGHT_EEPROM_START  (CAMEL_EEPROM_START + CAMEL_CAL_RIGHT_OFFSET)
#define CAMEL_CONFIG_EEPROM_START (CAMEL_EEPROM_START + 256)


// pos is relative to start of EEPROM

uint8_t eeprom_read_byte(const uint32_t pos);

void eeprom_write_byte(const uint32_t pos, const uint8_t value);
uint32_t eeprom_read_word(const uint32_t pos);
void eeprom_write_word(const uint32_t pos, const uint32_t value);

uint16_t eeprom_length(void);
uint32_t eeprom_base_address(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_EEPROM_H_ */
