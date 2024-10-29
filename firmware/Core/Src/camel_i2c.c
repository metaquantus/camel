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

#include "camel_i2c.h"
#include "main.h"
#include "scales.h"

extern I2C_HandleTypeDef hi2c1;
extern uint8_t SCALES_DATA[SCALES_DATA_SIZE];
extern uint16_t SCALES_CONFIG;
extern HX711_TypeDef leftCell;
extern HX711_TypeDef rightCell;

#define RxSIZE  1
static uint8_t RxData[RxSIZE];
static uint8_t rxcount = 0;
static uint8_t txcount = 0;
static uint8_t startPosition=0;
// static int counterror = 0;

void process_data (void)
{
  // just pass the data to the main loop for parsing
  // so we spend less time in the interrupt
  SCALES_CONFIG = RxData[0];
  // MSB is used to indicate modified
  SCALES_CONFIG &= 0x80FF;
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
	  txcount = 3;
	  startPosition = 0;
	  if ( !leftCell.enabled && rightCell.enabled ) {
	    startPosition = 3;
	  }
	  if ( leftCell.enabled && rightCell.enabled ) {
	    txcount=6;
	  }
	  // transmit 3 or 6 bytes depending on whether both cells are enabled or only one
	  HAL_I2C_Slave_Seq_Transmit_IT(hi2c, SCALES_DATA + startPosition, txcount, I2C_FIRST_AND_LAST_FRAME);
	}
}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  /* nothing to do, we have sent right amount of data or host aborted before finish, getting an AF error

  // if no cells are enabled we still send 3 bytes of garbage
  uint8_t maxcount = 3;

  if ( leftCell.enabled && rightCell.enabled ) {
    maxcount = 6;
  }
  uint32_t frame = I2C_NEXT_FRAME;
  if ( txcount == maxcount ) {
    // transmit the last frame,
    // if master quits receiving before this, we'll get an AF error, but it's fine
    frame = I2C_LAST_FRAME;
  }
  HAL_I2C_Slave_Seq_Transmit_IT(hi2c, SCALES_DATA + startPosition + txcount, 1, frame);
  */
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

	// only 1 byte to receive, so if we are here, that's it
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
	    // host probably tried to send more than 1 byte,
	    // reject it and process the first byte as normal
	    process_data();
	  } else { // error while slave is transmitting
	    // just reset, the host didn't want more data
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
		// reset the count
		rxcount = 0;
		txcount = 0;
	}
	// everything good, keep listening
	HAL_I2C_EnableListen_IT(hi2c);
}
