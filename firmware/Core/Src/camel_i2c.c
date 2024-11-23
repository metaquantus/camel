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
extern uint16_t SCALES_CONFIG;
extern HX71x_TypeDef leftCell;
extern HX71x_TypeDef rightCell;
extern uint8_t EEPROM_DATA[EEPROM_DATA_SIZE];

// first byte address/function and up to 8 bytes of data
#define RxSIZE  9
static uint8_t RxData[RxSIZE];
static uint8_t rxcount = 0;
static uint8_t txcount = 0;
static uint8_t startPosition=0;
#define TxSIZE 9
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

  rxmode = 0;
  /*
  if (addr == 0x81) {
    rxmode = 1;
  } else if (addr == 0x82) {
    rxmode = 2;
  } else {

    // just pass the data to the main loop for parsing
    // so we spend less time in the interrupt
    SCALES_CONFIG = addr & 0xFF;
    // MSB is used to indicate modified
    SCALES_CONFIG |= 0x8000;
  }
  */
  switch(addr) {
  case 0:
    // just pass the data to the main loop for parsing
    // so we spend less time in the interrupt
    SCALES_CONFIG = RxData[1] & 0xFF;
    // MSB nibble is used to indicate modified
    SCALES_CONFIG |= 0x8000;

    break;
  case 1:
    // pass data to main loop and process there
    // left scaling factor store, data in EEPROM_DATA
    SCALES_CONFIG |= 0x4000;
    break;
  case 2:
    // pass data to main loop and do process there
    // right scaling factor store, data in EEPROM_DATA
    SCALES_CONFIG |= 0x2000;
    break;
  case 4:
    // pass data to main loop and do process there
    // left & right scaling factors store, data in EEPROM_DATA
    // also store current config in EEPROM
    SCALES_CONFIG |= 0x6000;
    break;
  case 0x80:
    // read target indicated in second byte
    // combines all the other 0x8x commands
    rxmode = RxData[1];
    break;
  case 0x81:
    // left scaling
    rxmode = 1;
    break;
  case 0x82:
    // right scaling
    rxmode = 2;
    break;
  case 0x83:
    // config
    rxmode = 3;
    break;
  case 0x84:
    // left, right and config
    rxmode = 4;
    break;
  case 0x85:
    // EEPROM size
    rxmode = 5;
    break;
  case 0x86:
    // EEPROM base address
    rxmode = 6;
    break;
  }
}

void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c)
{
	HAL_I2C_EnableListen_IT(hi2c);
}

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
{
	if (TransferDirection == I2C_DIRECTION_TRANSMIT)  // if the master wants to transmit the data
	{
		rxcount = 0;
		// receive using sequential function.

		// first byte is "register" address, rest up to 8 bytes of data depending on address
		HAL_I2C_Slave_Sequential_Receive_IT(hi2c, RxData, 1, I2C_FIRST_FRAME);
	}
	else  // if the master requests the data from the slave
	{
	  switch(rxmode) {
    case 1:
      // left scaling
      txcount = 4;
      HAL_I2C_Slave_Seq_Transmit_IT(hi2c, EEPROM_DATA, txcount, I2C_FIRST_AND_LAST_FRAME);
      // reset mode, only valid for one cycle
      break;
    case 2:
      // right scaling
      txcount = 4;
      HAL_I2C_Slave_Seq_Transmit_IT(hi2c, EEPROM_DATA+4, txcount, I2C_FIRST_AND_LAST_FRAME);
      // reset mode, only valid for one cycle
      break;
    case 3:
      // send current configuration
      txcount = 1;
      TxData[0] = SCALES_CONFIG & 0xFF;
      HAL_I2C_Slave_Seq_Transmit_IT(hi2c, TxData, txcount, I2C_FIRST_AND_LAST_FRAME);
      // reset mode, only valid for one cycle
      break;
    case 4:
      // left and right scalings and config
      txcount = 9;
      memcpy(TxData, EEPROM_DATA, 8);
      TxData[8] = SCALES_CONFIG & 0xFF;
      HAL_I2C_Slave_Seq_Transmit_IT(hi2c, TxData, txcount, I2C_FIRST_AND_LAST_FRAME);
      // reset mode, only valid for one cycle
      break;
	  case 5:
      uint16_t len = eeprom_length();
      memcpy(TxData, &len, 2);
      txcount = 2;
      HAL_I2C_Slave_Seq_Transmit_IT(hi2c, TxData, txcount, I2C_FIRST_AND_LAST_FRAME);
      break;
	  case 6:
      uint32_t base = eeprom_base_address();
      memcpy(TxData, &base, 4);
      txcount = 4;
      HAL_I2C_Slave_Seq_Transmit_IT(hi2c, TxData, txcount, I2C_FIRST_AND_LAST_FRAME);
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
      HAL_I2C_Slave_Seq_Transmit_IT(hi2c, SCALES_DATA + startPosition, txcount, I2C_FIRST_AND_LAST_FRAME);
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
}


void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  // first byte is address to write to:
  // 0: configuration
  // 1: left cell scaling factor
  // 2: right cell scaling factor
  // 4: both left cell and right cell scaling factors (first 4 bytes for left cell)
  if ( rxcount == 0 ) {
    // we just received the first byte (address)
    rxcount++;
    switch (RxData[0]) {
    case 0:
      // configuration register
      HAL_I2C_Slave_Sequential_Receive_IT(hi2c, RxData + rxcount, 1, I2C_LAST_FRAME);
      break;
    case 1:
      // note that this way, we don't know exactly where host stopped sending data if it sends less than expected
      // left scaling factor storage
      HAL_I2C_Slave_Sequential_Receive_IT(hi2c, EEPROM_DATA, 4, I2C_LAST_FRAME);
      break;
    case 2:
      // right scaling factor storage
      HAL_I2C_Slave_Sequential_Receive_IT(hi2c, EEPROM_DATA+4, 4, I2C_LAST_FRAME);
      break;
    case 4:
      // left and right scaling factors storage
      HAL_I2C_Slave_Sequential_Receive_IT(hi2c, EEPROM_DATA, 8, I2C_LAST_FRAME);
      break;
    // the following codes request other data from the device,
    // by default a read sends load cell values, but the next codes switches read target for next read.
    case 0x80:
      // read one more byte indicating what to send in next read request
      HAL_I2C_Slave_Sequential_Receive_IT(hi2c, RxData+rxcount, 1, I2C_LAST_FRAME);
      break;
    // the following are an alternative to 0x80 but encoded in the "register" address, so no need for a second byte
    case 0x81:
    case 0x82:
    case 0x83:
    case 0x84:
    case 0x85:
    case 0x86:
      // try to read more data but ignore, host should stop transmission now and don't send anymore data,
      // this will cause AF error but it should be OK, we got what we need now. (process data will be handled in error handler).
      // Is there a way to send last frame condition without requesting more data?
      // so we can do it without causing an error. (Maybe some LL_ function or programming a register directly)
      // We need to read the first byte (register address) to know what to do next, but in case we don't need
      // more data, it's already too late to send stop from our side using the HAL API, or so it seems.
      HAL_I2C_Slave_Sequential_Receive_IT(hi2c, RxData+rxcount, 1, I2C_LAST_FRAME);
      break;
    default:
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
