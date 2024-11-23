/*
 * eeprom.h
 *
 *  Created on: Nov 17, 2024
 *      Author: cav
 *
 *  Non-volatile memory functions used to store calibration data for
 *  the load cells.
 *  We take 16 bytes at the end of the EEPROM memory to store the data
 *
 */

#ifndef INC_EEPROM_H_
#define INC_EEPROM_H_

#include "stm32l0xx_hal.h"

/*
 * Use the last 16 bytes of EEPROM for CAMEL (only 9 bytes used at the moment)
 * We use 4 bytes to store a float value holding the scaling factor for each cell.
 * The scaling factors can be written and read from host. We just store the values,
 * so the device can be disconnected and moved to another host and still retain
 * the calibration data.
 * Configuration byte is stored in the ninth byte.
 */
#define CAMEL_EEPROM_START       (DATA_EEPROM_END - DATA_EEPROM_BASE - 0x0F)
#define CAMEL_LEFT_EEPROM_START  CAMEL_EEPROM_START
#define CAMEL_RIGHT_EEPROM_START (CAMEL_EEPROM_START + 4)
#define CAMEL_CONFIG_EEPROM_START (CAMEL_EEPROM_START + 8)
#define EEPROM_DATA_SIZE 8

// pos is relative to start of EEPROM
uint8_t eeprom_read_byte(const uint32_t pos);
void eeprom_write_byte(const uint32_t pos, const uint8_t value);
uint32_t eeprom_read_word(const uint32_t pos);
void eeprom_write_word(const uint32_t pos, const uint32_t value);

uint16_t eeprom_length(void);

uint32_t eeprom_base_address(void);

#endif /* INC_EEPROM_H_ */
