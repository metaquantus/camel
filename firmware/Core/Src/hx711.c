/*
 * scales.c
 *
 *  Created on: Oct 27, 2024
 *      Author: cav
 */
#include <hx711.h>
#include "main.h"

/*
 * HX711s are wired at 10 samples per second, channel B is not used.
 * Each HX711 has its own separate PD_SCK clock signal, so it can be power down if not used.
 * Power down is done by setting its PD_SCK to HIGH for more than 60us.
 * This MCU runs at 32MHz (31ns clock period), some extra delay is added using ASM NOPs.
 * The main loop waits for both left and right HX711s to be ready, so they can be read together.
 * So they will be ready about the same time on the next reading.
 * Data is read and copied to the global buffer.
 * There's the issue of the read routine being interrupted and the risk of the HX711s getting power down while servicing the ISR,
 * since we have the I2C subsystem using interrupt mode. However, we assume the I2C IRQ handlers don't take more than 60us.
 */

extern uint8_t SCALES_DATA[SCALES_DATA_SIZE];

/*
uint8_t shiftIn(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder) {
    uint8_t value = 0;
    uint8_t i;

    for(i = 0; i < 8; ++i) {
        digitalWrite(clockPin, HIGH);
        delayMicroseconds(1);
        if(bitOrder == LSBFIRST)
            value |= digitalRead(dataPin) << i;
        else
            value |= digitalRead(dataPin) << (7 - i);
        digitalWrite(clockPin, LOW);
        delayMicroseconds(1);
    }
    return value;
}
*/

// reads one byte from both left and right HX711s
uint16_t shiftIn(HX711_TypeDef *lcell, HX711_TypeDef *rcell) {
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

void HX711_read(HX711_TypeDef* lcell, HX711_TypeDef* rcell) {
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
	// GAIN should be 1 or 3
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

	// enable interrupts
	// interrupts();

	// copy data to global buffer
	// left
	SCALES_DATA[0] = (data[0] >> 8) & 0xFF;
	SCALES_DATA[1] = (data[1] >> 8) & 0xFF;
	SCALES_DATA[2] = (data[2] >> 8) & 0xFF;
	// right
	SCALES_DATA[3] = data[0] & 0xFF;
	SCALES_DATA[4] = data[1] & 0xFF;
	SCALES_DATA[5] = data[2] & 0xFF;

}

uint8_t HX711_isReady(HX711_TypeDef* cell) {
	return HAL_GPIO_ReadPin(cell->DOUT_Port, cell->DOUT_Pin) == GPIO_PIN_RESET;
}

void HX711_powerUp(HX711_TypeDef* cell) {
	HAL_GPIO_WritePin(cell->SCK_Port, cell->SCK_Pin, GPIO_PIN_RESET);
}

void HX711_powerDown(HX711_TypeDef* cell) {
	HAL_GPIO_WritePin(cell->SCK_Port, cell->SCK_Pin, GPIO_PIN_RESET);
	asm("NOP\n\tNOP\n\tNOP\n\tNOP\n\tNOP\n\tNOP");
	HAL_GPIO_WritePin(cell->SCK_Port, cell->SCK_Pin, GPIO_PIN_SET);
	// it will power down if SCK remains high for 60us
	// this will be the case if no more commands from the host override it
}
