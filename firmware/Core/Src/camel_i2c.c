/*
 * camel_i2c.c
 *
 *  Created on: Oct 27, 2024
 *      Author: cav
 */

#include "camel_i2c.h"
#include "main.h"
#include "scales.h"

extern I2C_HandleTypeDef hi2c1;
extern uint8_t SCALES_DATA[SCALES_DATA_SIZE];
extern HX711_TypeDef leftCell;
extern HX711_TypeDef rightCell;

#define RxSIZE  1
static uint8_t RxData[RxSIZE];
static uint8_t rxcount = 0;
static uint8_t txcount=0;
static uint8_t startPosition=0;
// static int counterror = 0;

void process_data (void)
{
  /*
	   We only support one receive command (master transmit):
       the configuration command.
     It consists of two nibbles, the most significant for the left cell,
     and the least significant for the right cell:
       00110011
     bit 0 of each nibble sets whether to enable/disable the corresponding cell
     bit 1 sets the gain factor: 0 for 64 or 1 for 128
     e.g: 0x33 enables both cells at 128 gain factor
     The other bits are ignored, reserved for future use, set to 0.
     If the enable bit is changed from previous value, then the modified state is also set to let the main
     loop know and do the job.
   */
  uint8_t config = RxData[0];
  // left cell gain
  if ( (config & 0x20) != 0 ) {
    leftCell.GAIN = 1; // 128, channel A
  } else {
    leftCell.GAIN = 3; // 64, channel A
  }
  // right cell gain
  if ( (config & 0x02) != 0 ) {
    rightCell.GAIN = 1; // 128, channel A
  } else {
    rightCell.GAIN = 3; // 64, channel A
  }
  // left cell enable
  if ( (config & 0x10) != 0 ) {
    if ( ! leftCell.enabled ) {
      leftCell.enabled = 1;
      leftCell.modified = 1;
    }
  } else {
    if (leftCell.enabled) {
      leftCell.enabled = 0;
      leftCell.modified = 1;
    }
  }
  // right cell enable
  if ((config & 0x01) != 0) {
    if (!rightCell.enabled) {
      rightCell.enabled = 1;
      rightCell.modified = 1;
    }
  } else {
    if (rightCell.enabled) {
      rightCell.enabled = 0;
      rightCell.modified = 1;
    }
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
		// only one byte to receive, if master tries to transmit more than 1 byte, we'll get an AF error
		HAL_I2C_Slave_Sequential_Receive_IT(hi2c, RxData, RxSIZE, I2C_FIRST_AND_LAST_FRAME);
	}
	else  // if the master requests the data from the slave
	{
	  // If both cells are enabled, transmit the whole 6 bytes, left Cell first;
	  // otherwise only transmit the 3 bytes of the enabled cell
	  txcount = 0;
	  startPosition = 0;
	  if ( !leftCell.enabled && rightCell.enabled ) {
	    startPosition = 3;
	  }
	  // transmit first byte, maximum 3 or 6 bytes depending on whether both cells are enabled or only one
	  HAL_I2C_Slave_Seq_Transmit_IT(hi2c, SCALES_DATA + startPosition + txcount, 1, I2C_FIRST_FRAME);
	}
}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  txcount++;
  uint32_t frame = I2C_NEXT_FRAME;
  uint8_t maxcount = 3;
  if ( leftCell.enabled && rightCell.enabled ) {
    maxcount = 6;
  }
  /* redundant test
  else if ( leftCell.enabled || rightCell.enabled ) {
    maxcount = 3;
  }
  */
  if ( txcount == maxcount ) {
    // transmit the last frame,
    // if master quits receiving before this, we'll get an AF error, but it's fine
    frame = I2C_LAST_FRAME;
  }
  HAL_I2C_Slave_Seq_Transmit_IT(hi2c, SCALES_DATA + startPosition + txcount, 1, frame);
}


void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	/*
	rxcount++;
	if (rxcount < RxSIZE)
	{
		if (rxcount == RxSIZE-1)
		{
			HAL_I2C_Slave_Sequential_Receive_IT(hi2c, RxData+rxcount, 1, I2C_LAST_FRAME);
		}
		else
		{
			HAL_I2C_Slave_Sequential_Receive_IT(hi2c, RxData+rxcount, 1, I2C_NEXT_FRAME);
		}
	}

	if (rxcount == RxSIZE)
	{
		process_data();
	}
	*/

	// only 1 byte to receive
	process_data();

}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
	// counterror++;
	uint32_t errorcode = HAL_I2C_GetError(hi2c);
	if (errorcode == HAL_I2C_ERROR_AF)  // AF error
	{
	  if ( txcount == 0 ) { // error is while slave is receiving
	    rxcount = 0;
	    process_data();
	  } else { // error while slave is transmitting
	    txcount = 0;
	  }
	}
	/* BERR Error commonly occurs during the Direction switch
	 * Here we the software reset bit is set by the HAL error handler
	 * Before resetting this bit, we make sure the I2C lines are released and the bus is free
	 * I am simply reinitializing the I2C to do so
	 */
	else if (errorcode == HAL_I2C_ERROR_BERR)  // BERR Error
	{
		HAL_I2C_DeInit(hi2c);
		HAL_I2C_Init(hi2c);
		// memset(RxData,'\0',RxSIZE);  // reset the Rx buffer
		RxData[0] = '\0';
		rxcount =0;  // reset the count
	}

	HAL_I2C_EnableListen_IT(hi2c);
}
