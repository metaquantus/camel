/*
 * camel_i2c.c
 *
 *  Created on: Oct 27, 2024
 *      Author: cav
 *
 *  Camel I2C slave communications.
 *  We support 1 (host) transmit command receiving 1 configuration byte
 *  and a (host) read command that can send 3 or 6 bytes depending on
 *  whether both HX711 cells are enabled or only one.
 *  The configuration command enables or disables the cells.
 *  No sub-addresses (registers) are needed, so it's simple.
 */
#include <hx71x.h>
#include <camel_i2c.h>
#include <main.h>
#include <string.h>
#include <eeprom.h>


extern I2C_HandleTypeDef hi2c1;
extern uint8_t SCALES_DATA[SCALES_DATA_SIZE];
extern uint8_t SCALES_CONFIG;
extern uint8_t CAL_COUNT;
extern uint8_t CAL_OFFSET;
extern uint8_t FUNC_FLAG;
extern HX71x_TypeDef leftCell;
extern HX71x_TypeDef rightCell;
extern uint8_t CAL_DATA[CAMEL_CAL_DATA_SIZE];

// first byte address/function and up to 8 bytes of data
#define RxSIZE  10
static uint8_t RxData[RxSIZE];
static uint8_t rxcount = 0;
static uint8_t txcount = 0;
static uint8_t startPosition=0;
#define TxSIZE 10
static uint8_t TxData[TxSIZE];

// static int counterror = 0;
/*
 * rxmode = 0, normal read sensor data 3 or 6 bytes
 * rxmode = 1, read flash length, 2 bytes
 * rxmode = 2, read flash base address, 4 bytes
 */
static uint8_t rxmode = 0;

void process_data (void)
{
  uint8_t addr = RxData[0];
  uint8_t caddr = addr & 0x8F;
  rxmode = 0;

  if (addr == 0) {
    // just pass the data to the main loop for parsing
    // so we spend less time in the interrupt
    SCALES_CONFIG = RxData[1];
    FUNC_FLAG = CONFIG_MASK;
    // SCALES_CONFIG |= CONFIG_MASK;
  } else if (caddr == 1) {
    // left
    // the calibration address is in the high nibble
    // copy to buffer and pass to main loop for storing in EEPROM
    uint32_t offset = (addr & 0x70) >> 4;
    if ( offset >= 0 && offset < CAMEL_CAL_DATA_COUNT ) {
      CAL_OFFSET = offset << 3; // offset * 8
      uint8_t *pos = CAL_DATA + CAL_OFFSET ;
      memcpy(pos, RxData+1, 8);

      FUNC_FLAG = LEFT_MASK;
    }
  } else if ( caddr == 2) {
    // right
    // the calibration address is in the high nibble
    uint32_t offset = (addr & 0x70) >> 4;
    if (offset >= 0 && offset < CAMEL_CAL_DATA_COUNT) {
      CAL_OFFSET = offset << 3; // offset * 8
      uint8_t *pos = CAL_DATA + CAMEL_CAL_RIGHT_OFFSET + CAL_OFFSET;
      memcpy(pos, RxData + 1, 8);
      FUNC_FLAG = RIGHT_MASK;
    }
  } else if ( caddr == 4 ) {
    // calibration count
    CAL_COUNT = RxData[1] & 0x0F;
    FUNC_FLAG = COUNT_MASK;
    // this function also saves both the count and config to EEPROM
    // SCALES_CONFIG |= COUNT_MASK;
  } else if ( addr == 0x80 ) {
    // this is a request for calibration data, 8 bytes, or config/count, 2 bytes
    // the cell and offset are encoded in the next byte

    rxmode = RxData[1];
    // make sure we have valid codes
    // Note that code 0, will simply result in sensor data
    if ( (rxmode & 0x0F) > 6 ) {
      rxmode = 0;
    }
  }
}

void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c)
{
	HAL_I2C_EnableListen_IT(hi2c);
}

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
{
	if (TransferDirection == I2C_DIRECTION_TRANSMIT) // if the master wants to transmit the data
	{
		txcount = 0;
		rxcount = 0;
		// receive using sequential function.

		// first byte is "register" address, rest up to 8 bytes of data depending on address
		HAL_I2C_Slave_Sequential_Receive_IT(hi2c, RxData, 1, I2C_FIRST_FRAME);
	} else  // if the master requests the data from the slave
	{
		txcount = 0;
		uint8_t mode = rxmode & 0x0F;
		uint32_t offset;
		switch (mode) {
		case 1:
			// left calibration data
			txcount = 8;
			offset = (rxmode & 0xF0) >> 1; // ((rxmode & 0xF0) >> 4) << 3; // index * 8
			HAL_I2C_Slave_Seq_Transmit_IT(hi2c, CAL_DATA + offset, txcount,
					I2C_FIRST_AND_LAST_FRAME);
			break;
		case 2:
			// right calibration data, right data starts at EEPROM_DATA + CAMEL_CAL_RIGHT_OFFSET
			txcount = 8;
			offset = (rxmode & 0xF0) >> 1; // ((rxmode & 0xF0) >> 4) << 3; // index * 8
			HAL_I2C_Slave_Seq_Transmit_IT(hi2c, CAL_DATA + CAMEL_CAL_RIGHT_OFFSET + offset, txcount,
					I2C_FIRST_AND_LAST_FRAME);
			break;
		case 4:
			// config and calibration count
			txcount = 2;
			TxData[0] = SCALES_CONFIG;
			TxData[1] = CAL_COUNT;
			HAL_I2C_Slave_Seq_Transmit_IT(hi2c, TxData, txcount,
					I2C_FIRST_AND_LAST_FRAME);
			break;
		case 5:
			uint16_t len = eeprom_length();
			memcpy(TxData, &len, 2);
			txcount = 2;
			HAL_I2C_Slave_Seq_Transmit_IT(hi2c, TxData, txcount,
					I2C_FIRST_AND_LAST_FRAME);
			break;
		case 6:
			uint32_t base = eeprom_base_address();
			memcpy(TxData, &base, 4);
			txcount = 4;
			HAL_I2C_Slave_Seq_Transmit_IT(hi2c, TxData, txcount,
					I2C_FIRST_AND_LAST_FRAME);
			break;
		default:
			// If both cells are enabled, transmit the whole 6 bytes, left Cell first;
			// otherwise only transmit the 3 bytes of the enabled cell
			txcount = 3;
			startPosition = 0;
			if (!leftCell.enabled && rightCell.enabled) {
				startPosition = 3;
			}
			if (leftCell.enabled && rightCell.enabled) {
				txcount = 6;
			}
			// transmit 3 or 6 bytes depending on whether both cells are enabled or only one
			HAL_I2C_Slave_Seq_Transmit_IT(hi2c, SCALES_DATA + startPosition,
					txcount, I2C_FIRST_AND_LAST_FRAME);
		}
		// reset back to default function: sending the load cell values.
		// other functions are only set for one write/read transaction
		rxmode = 0;
	}
}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  /* nothing to do, we have sent right amount of data or host aborted before finish, getting an AF error
  */
  txcount = 0;
}


void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  // first byte is address to write to:
  // 0: configuration for left and right cells (no saving to EEPROM)
  // 1: left cell calibration data (8 bytes) (saved to EEPROM)
  // 2: right cell calibration data (8 bytes) (saved to EEPROM)
  // 4: calibration count (saved to EEPROM together with configuration)
  if ( rxcount == 0 ) {
    // we just received the first byte (address)
    rxcount++;
    // mask the calibration address
    uint8_t reg = RxData[0] & 0x8F;

    if ( reg == 0 ) {
      // configuration register (but do not save to EEPROM)
      HAL_I2C_Slave_Sequential_Receive_IT(hi2c, RxData + rxcount, 1, I2C_LAST_FRAME);
    } else if ( reg == 1 || reg == 2) {
      // left or right calibration data, offset encoded in first byte high nibble
      HAL_I2C_Slave_Sequential_Receive_IT(hi2c, RxData + rxcount, 8, I2C_LAST_FRAME);
    } else if ( reg == 4 ) {
      // calibration count (also saves config to EEPROM)
      HAL_I2C_Slave_Sequential_Receive_IT(hi2c, RxData + rxcount, 1, I2C_LAST_FRAME);
    } else if ( reg >= 0x80 && reg <= 0x86 ) {
      // these codes are for requesting other data to send,
      // by default a read request sends the cells sensor data
      // read one more byte indicating what to send in next read request (0x80)

      // 0x81 - 0x86
      // try to read more data but ignore, host should stop transmission now and don't send anymore data,
      // this will cause AF error but it should be OK, we got what we need now. (process data will be handled in error handler).
      // Is there a way to send last frame condition without requesting more data?
      // so we can do it without causing an error. (Maybe some LL_ function or programming a register directly)
      // We need to read the first byte (register address) to know what to do next, but in case we don't need
      // more data, it's already too late to send stop from our side using the HAL API, or so it seems.
      HAL_I2C_Slave_Sequential_Receive_IT(hi2c, RxData+rxcount, 1, I2C_LAST_FRAME);
    } else {
      // try to read more data but ignore, host should stop transmission now and don't send anymore data
      HAL_I2C_Slave_Sequential_Receive_IT(hi2c, RxData+rxcount, 1, I2C_LAST_FRAME);
    }
  } else  {
    process_data();
  }

}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
	// counterror++;
	uint32_t errorcode = HAL_I2C_GetError(hi2c);
	if (errorcode == HAL_I2C_ERROR_AF)  // AF error
	{
	  if ( txcount == 0 ) { // error is while slave is receiving
	    rxcount = 0;
	    // host probably tried to send more or less than requested,
	    // ignore it and process the first byte as normal
	    process_data();
	  } else { // error while slave is transmitting
	    // just reset, the host didn't want more data
	    txcount = 0;
	  }
	}
	/* BERR Error commonly occurs during the Direction switch
	 * Here the software reset bit is set by the HAL error handler
	 * Before resetting this bit, we make sure the I2C lines are released and the bus is free
	 * I am simply reinitializing the I2C to do so
	 */
	else if (errorcode == HAL_I2C_ERROR_BERR)  // BERR Error
	{
		HAL_I2C_DeInit(hi2c);
		HAL_I2C_Init(hi2c);
		// memset(RxData,'\0',RxSIZE);  // reset the Rx buffer
		RxData[0] = '\0';
		// reset the count
		rxcount = 0;
		txcount = 0;
		rxmode = 0;
	}
	// everything good, keep listening
	HAL_I2C_EnableListen_IT(hi2c);
}
