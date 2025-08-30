/*
 * scales.cpp
 *
 *  Created on: Aug 2, 2025
 *      Author: cav
 */
#include "cmsis_os.h"
#include "scales.h"
#include "utils.h"
#include "camel.h"
#include "eeprom.h"
#include <cstring>

// the CRC device
extern CRC_HandleTypeDef hcrc;
extern osEventFlagsId_t eventFlagsId;

void HX71xDevice::init() {
  // initializes the device in default settings
  tareCount = 0;
  tareIndex = 0;
  leftTareOffset = 0;
  rightTareOffset = 0;
  // calibration
  leftCalCount = 0;
  leftCalIndex = 0;
  leftCalWeight = 0.0;
  rightCalCount = 0;
  rightCalIndex = 0;
  rightCalWeight = 0.0;
  calIndex = 0;

  dev.config.value = CAMEL_DEFAULT_CONFIG;
  dev.count = 0;
  dev.mode = 0;
  parseConfig();
  for(int i=0; i<CAMEL_MAX_CAL_COUNT; i++) {
    calData[i].Left.scale = 1.0;
    calData[i].Left.value = 0;
    calData[i].Left.flag = 0;
    calData[i].Left.weight = 0.0;
    calData[i].Right.scale = 1.0;
    calData[i].Right.value = 0;
    calData[i].Right.flag = 0;
    calData[i].Right.weight = 0.0;
  }
  readConfig();

}

void HX71xDevice::readConfig() {
  uint8_t config[4] = { 0, 0, 0, 0 };
  config[0] = eeprom_read_byte(CAMEL_CONFIG_EEPROM_START);
  config[1] = eeprom_read_byte(CAMEL_CONFIG_EEPROM_START+1);
  config[2] = eeprom_read_byte(CAMEL_CONFIG_EEPROM_START+2);
  config[3] = eeprom_read_byte(CAMEL_CONFIG_EEPROM_START+3);
  uint8_t crc = GENCRC(config, 3);
  if ( crc == config[3] ) {
    // config is valid
    dev.config.value = config[0];
    dev.count = config[1] & 0x0F;
    dev.mode = (config[1] >> 4) & 0x0F;
    // reparse anyway
    parseConfig();
    if ( dev.count > CAMEL_MAX_CAL_COUNT ) {
      dev.count = 0;
    }
    if ( dev.mode > 2 ) {
      dev.mode = 0;
    }
  }
  if ( dev.count > 0 ) {
    for(uint32_t i=0; i<dev.count; i++) {
      uint32_t loffset = CAMEL_EEPROM_START + (i << 5); // i * 32
      uint32_t roffset = loffset + 16;
      uint32_t v[4] = {0, 0, 0, 0};
      v[0] = eeprom_read_word(loffset);
      v[1] = eeprom_read_word(loffset+4);
      v[2] = eeprom_read_word(loffset+8);
      v[3] = eeprom_read_word(loffset+12);
      uint8_t crc = GENCRC(v, 15);
      uint8_t vcrc = (v[3] >> 24) & 0xFF;
      if ( crc == vcrc ) {
        // to avoid compiler warning (type punning)
        // calData[i].Left.scale = * (float *) (&v[0]);
        memcpy(&calData[i].Left.scale, &v[0], 4);
        calData[i].Left.value = v[1];
        // calData[i].Left.weight = * (float *) &v[2];
        memcpy(&calData[i].Left.weight, &v[2], 4);
        calData[i].Left.flag = v[3] & 0x00FFFFFF;
        calData[i].Left.crc = vcrc;
      }
      v[0] = eeprom_read_word(roffset);
      v[1] = eeprom_read_word(roffset + 4);
      v[2] = eeprom_read_word(roffset + 8);
      v[3] = eeprom_read_word(roffset + 12);
      crc = GENCRC(v, 15);
      vcrc = (v[3] >> 24) & 0xFF;
      if (crc == vcrc) {
        // calData[i].Right.scale = *(float*) &v[0];
        memcpy(&calData[i].Right.scale, &v[0], 4);
        calData[i].Right.value = v[1];
        // calData[i].Right.weight = *(float*) &v[2];
        memcpy(&calData[i].Right.weight, &v[2], 4);
        calData[i].Right.flag = v[3] & 0x00FFFFFF;
        calData[i].Right.crc = vcrc;
      }
    }
  }
  osEventFlagsSet(eventFlagsId, EVENT_CONFIG_LOAD_READY);
}

void HX71xDevice::writeConfig() {
  uint8_t config[4];
  memcpy(config, &dev, 3);
  config[3]= GENCRC(config, 3);
  for(uint8_t i = 0; i<4; i++) {
    eeprom_write_byte(CAMEL_CONFIG_EEPROM_START + i, config[i]);
  }

  if ( dev.count > 0 ) {
    for(uint32_t i=0; i<dev.count; i++) {
      uint32_t loffset = CAMEL_EEPROM_START + (i << 5); // i * 32
      uint32_t roffset = loffset + 16;
      uint32_t v[4] = {0, 0, 0, 0};

      memcpy( &v[0], &calData[i].Left.scale, 4);
      v[1] = calData[i].Left.value;
      memcpy(&v[2], &calData[i].Left.weight, 4);
      v[3] = calData[i].Left.flag;
      uint8_t crc = GENCRC(v, 15);
      v[3] |= (((uint32_t)crc << 24) & 0xFF000000);
      calData[i].Left.crc = crc;
      for (uint8_t i = 0; i < 4; i++) {
        eeprom_write_word(loffset + (i<<2), v[i]);
      }

      memcpy(&v[0], &calData[i].Right.scale, 4);
      v[1] = calData[i].Right.value;
      memcpy(&v[2], &calData[i].Right.weight, 4);
      v[3] = calData[i].Right.flag;
      crc = GENCRC(v, 15);
      v[3] |= (((uint32_t) crc << 24) & 0xFF000000);
      calData[i].Right.crc = crc;
      for (uint8_t i = 0; i < 4; i++) {
        eeprom_write_word(roffset + (i << 2), v[i]);
      }
    }
  }
  osEventFlagsSet(eventFlagsId, EVENT_CONFIG_SAVE_READY);
}

void HX71xDevice::setCellConfiguration(uint8_t cnf) {
  dev.config.value = cnf;
  parseConfig();
  osEventFlagsSet(eventFlagsId, EVENT_CONFIG_READY);
}

void HX71xDevice::setCalibrationCount(uint8_t count) {
  dev.count = count;
  osEventFlagsSet(eventFlagsId, EVENT_CAL_COUNT_READY);
}

void HX71xDevice::setReadMode(uint8_t mode) {
  dev.mode = mode;
  osEventFlagsSet(eventFlagsId, EVENT_READ_MODE_READY);
}

bool HX71xDevice::calibrate(CellEnum cell, uint8_t index, float weight, uint8_t count) {
  if (count == 0) {
    return false;
  }
  if (index >= 0 && index < CAMEL_MAX_CAL_COUNT) {
    if (count > 15) {
      count = 15;
    }
    calIndex = index;
    if (cell == LEFT) {
      leftCalIndex = 0;
      leftCalWeight = weight;
      leftCalCount = count;
    } else {
      rightCalIndex = 0;
      rightCalWeight = weight;
      rightCalCount = count;
    }
    return true;
  }
  return false;
}

void HX71xDevice::parseConfig() {
#ifdef CAMEL1
    // HX711
    // left cell gain
    if ( dev.config.V.leftGain == 1) {
      dev.leftProg = 1; // 128, channel A
    } else {
      dev.leftProg = 3; // 64, channel A
    }
    // right cell gain
    if (dev.config.V.rightGain == 1)) {
      dev.rightProg = 1; // 128, channel A
    } else {
      dev.rightProg = 3; // 64, channel A
    }
#else
  // HX712 CAMEL2+
  if ( dev.config.V.leftRate == 0 ) {
    if ( dev.config.V.leftGain == 0 ) {
      dev.leftProg = 1; // 128/10Hz
    } else {
      dev.leftProg = 4; // 256/10Hz
    }
  } else {
    if (dev.config.V.leftGain == 0) {
      dev.leftProg = 3; // 128/40Hz
    } else {
      dev.leftProg = 5; // 256/40Hz
    }
  }
  if (dev.config.V.rightRate == 0) {
    if (dev.config.V.rightGain == 0) {
      dev.rightProg = 1; // 128/10Hz
    } else {
      dev.rightProg = 4; // 256/10Hz
    }
  } else {
    if (dev.config.V.rightGain == 0) {
      dev.rightProg = 3; // 128/40Hz
    } else {
      dev.rightProg = 5; // 256/40Hz
    }
  }
#endif
}

// reads 8-bits of data from the HX71x's
CellReadByteValue HX71xDevice::shiftIn() {
  uint16_t lvalue = 0;
  uint16_t rvalue = 0;
  CellReadByteValue result;
  uint8_t i;

  for (i = 0; i < 8; ++i) {
    // We do it this way to give more time to the signals,
    // At 32MHz MCU clock, we need at least 4 clock cycles to meet minimum value required by HX711 (0.1us)
    // for rising edge to value sampling
    // asm("NOP\n\tNOP\n\tNOP\n\tNOP");
    if (dev.config.V.leftEnable) {
      HAL_GPIO_WritePin(leftCell.SCK_Port, leftCell.SCK_Pin, GPIO_PIN_SET);
    }
    if (dev.config.V.rightEnable) {
      HAL_GPIO_WritePin(rightCell.SCK_Port, rightCell.SCK_Pin, GPIO_PIN_SET);
    }
    asm("NOP\n\tNOP\n\tNOP\n\tNOP\n\tNOP\n\tNOP");
    // delay
    if (dev.config.V.leftEnable) {
      lvalue |= HAL_GPIO_ReadPin(leftCell.DOUT_Port, leftCell.DOUT_Pin) << (7 - i);
    }
    if (dev.config.V.rightEnable) {
      rvalue |= HAL_GPIO_ReadPin(rightCell.DOUT_Port, rightCell.DOUT_Pin) << (7 - i);
    }
    // asm("NOP\n\tNOP\n\tNOP\n\tNOP");
    if (dev.config.V.leftEnable) {
      HAL_GPIO_WritePin(leftCell.SCK_Port, leftCell.SCK_Pin, GPIO_PIN_RESET);
    }
    if (dev.config.V.rightEnable) {
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

void HX71xDevice::read() {
  // cells should be in ready state if enabled
  CellReadByteValue data[3] = { {0,0}, {0,0}, {0,0} };

  // as long as interrupts don't take longer than 60us, interrupts can be enabled
  // disable external interrupts while we read the sample value
  HAL_NVIC_DisableIRQ(EXTI0_1_IRQn); // left HX71x interrupt DOUT1
  HAL_NVIC_DisableIRQ(EXTI4_15_IRQn);// right HX71x interrupt DOUT2

  //make sure clock is enabled
  if (dev.config.V.leftEnable ) {
    HAL_GPIO_WritePin(leftCell.SCK_Port, leftCell.SCK_Pin, GPIO_PIN_RESET);
  }
  if (dev.config.V.rightEnable ) {
    HAL_GPIO_WritePin(rightCell.SCK_Port, rightCell.SCK_Pin, GPIO_PIN_RESET);
  }

  // Pulse the clock pin 24 times to read the data. MSB first.
  data[2] = shiftIn();
  data[1] = shiftIn();
  data[0] = shiftIn();

  // Set the rate and the gain factor for the next reading using the clock pin.
  // PROG should be 1 or 3 for hx711 or 1,3,4 or 5 for hx712
  for (unsigned int i = 0; i < dev.leftProg || i < dev.rightProg; i++) {
    asm("NOP\n\tNOP\n\tNOP\n\tNOP");
    if ( dev.config.V.leftEnable && i < dev.leftProg) {
      HAL_GPIO_WritePin(leftCell.SCK_Port, leftCell.SCK_Pin, GPIO_PIN_SET);
    }
    if ( dev.config.V.rightEnable && i < dev.rightProg) {
      HAL_GPIO_WritePin(rightCell.SCK_Port, rightCell.SCK_Pin, GPIO_PIN_SET);
    }
    asm("NOP\n\tNOP\n\tNOP\n\tNOP\n\tNOP\n\tNOP");
    if (dev.config.V.leftEnable && i < dev.leftProg ) {
      HAL_GPIO_WritePin(leftCell.SCK_Port, leftCell.SCK_Pin, GPIO_PIN_RESET);
    }
    if (dev.config.V.rightEnable && i < dev.rightProg ) {
      HAL_GPIO_WritePin(rightCell.SCK_Port, rightCell.SCK_Pin, GPIO_PIN_RESET);
    }
  }
  // ADCs should start sampling the next value now
  // We can then perform other operations while ADCs work

  // enable interrupts, DOUT lines go low when conversion ready
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn); // left HX71x interrupt DOUT1
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);// right HX71x interrupt DOUT2


  // copy raw data to global buffer
  // left
  unsigned long v  =
      (static_cast<unsigned long>(data[2].leftValue) << 16) |
      (static_cast<unsigned long>(data[1].leftValue) << 8) |
      static_cast<unsigned long>(data[0].leftValue);
  if ( (data[2].leftValue & 0x80) != 0 ) {
    v |= 0xFF000000; // extend sign
  }
  leftValue = static_cast<long>(v);
  // right
  v =
      (static_cast<unsigned long>(data[2].rightValue) << 16) |
      (static_cast<unsigned long>(data[1].rightValue) << 8) |
      static_cast<unsigned long>(data[0].rightValue);
  if ( (data[2].rightValue & 0x80) != 0 ) {
    v |= 0xFF000000;
  }
  rightValue = static_cast<long>(v);

  // compute the scaled calibrated value
  float lvalue = 0.0;
  float rvalue = 0.0;
  if (dev.config.V.leftEnable ) {
    float leftScale = 1;
    int32_t value = leftValue;
    // find the calibration scale factor for target value
    // calibration points should be in ascending absolute values
    // closest to current reading is chosen
    for (int i = 0; i < dev.count; i++) {
      leftScale = calData[i].Left.scale;
      int32_t cvalue = calData[i].Left.value;
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
    // left tare
    if ( tareCount > 0 ) {
      if ( tareIndex == 0 ) {
        leftTareOffset = leftValue;
      } else {
        // average
        leftTareOffset = (leftTareOffset + leftValue) / 2;
      }
    }
    // compute value
    lvalue = (leftValue - leftTareOffset) * leftScale;

    // compute new left scale factor if in calibration mode
    if ( leftCalCount > 0 ) {
      // this is wrong if LEFT_CAL_INDEX == 0, this is the first value
      // so we should not look at the previous value
      // we may have garbage in CAL_DATA to start with
      float scale = leftScale;

      if ( leftCalIndex > 0 ) { // otherwise this is the first value
        int32_t cvalue = calData[calIndex].Left.value;
        // average raw values by accumulating
        value = (cvalue + value) / 2;
      }

      if (calIndex == 0) {
        // use only this value
        if ( (value - leftTareOffset) != 0 ) {
          scale = leftCalWeight / (value - leftTareOffset);
        } else {
          scale = 1;
        }
      } else {
        // use this value and the previous calibration point to compute the current scale
        // previous calibration point must be set before this

        // get the index - 1 (previous) raw value
        int32_t pvalue = calData[calIndex-1].Left.value;
        // get the previous weight (index -1)
        float w0 = calData[calIndex-1].Left.weight;
        // finally, compute the current scale value,
        // which is the slope of the line segment from previous calibration point to current point
        if ( (value - pvalue) != 0 ) {
          scale = (leftCalWeight - w0) / (value - pvalue);
        } else {
          scale = 1;
        }
      }
      // copy back to memory
      calData[calIndex].Left.scale = scale;
      calData[calIndex].Left.value = value;
      calData[calIndex].Left.weight = leftCalWeight;
      calData[calIndex].Left.flag = 0;
      calData[calIndex].Left.crc = GENCRC(&(calData[calIndex].Left), sizeof(CalibrationPoint)-1);
      /*
      *(float*) (CAL_DATA + CAL_OFFSET) = scale;
      CAL_DATA[CAL_OFFSET + 4] = value & 0xFF;
      CAL_DATA[CAL_OFFSET + 5] = (value >> 8) & 0xFF;
      CAL_DATA[CAL_OFFSET + 6] = (value >> 16) & 0xFF;
      CAL_DATA[CAL_OFFSET + 7] = GENCRC(CAL_DATA + CAL_OFFSET, 7); // CRC-8
      */
    }

  }
  if ( dev.config.V.rightEnable ) {
    int32_t value = rightValue;
    float rightScale = 1;
    for (int i = 0; i < dev.count; i++) {
      rightScale = calData[i].Right.scale;
      int32_t cvalue = calData[i].Right.value;
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
    if (tareCount > 0) {
      if (tareIndex == 0) {
        rightTareOffset = value;
      } else {
        // average
        rightTareOffset = (rightTareOffset + value) / 2;
      }
    }

    rvalue = (value - rightTareOffset) * rightScale;

    if (rightCalCount > 0) {

      float scale = rightScale;
      if ( rightCalIndex > 0 ) {
        int32_t cvalue = calData[calIndex].Right.value;
        // average raw values cumulatively
        value = (value + cvalue) / 2;
      }
      if (calIndex == 0) {
        // there is no previous calibration point (or is zero, the tare value)
        if ( value - rightTareOffset != 0 ) {
          scale = rightCalWeight / (value - rightTareOffset);
        } else {
          scale = 1;
        }
      } else {
        // get the index - 1 (previous) raw value
        int32_t pvalue = calData[calIndex-1].Right.value;
        // get index - 1 (previous, less weight) calibrated value
        float w0 = calData[calIndex-1].Right.weight;
        // finally, compute the current scale value,
        // which is the slope of the line segment from previous calibration point to current point
        if ( value - pvalue != 0 ) {
          scale = (rightCalWeight - w0) / (value - pvalue);
        } else {
          scale = 1;
        }
      }

      // store right calibration data
      calData[calIndex].Right.value = value;
      calData[calIndex].Right.scale = scale;
      calData[calIndex].Right.weight = rightCalWeight;
      calData[calIndex].Right.flag = 0;
      calData[calIndex].Right.crc = GENCRC(&(calData[calIndex].Right), sizeof(CalibrationPoint)-1);
      /*
      *(float*) (CAL_DATA + CAMEL_CAL_RIGHT_OFFSET + CAL_OFFSET) = scale;
      CAL_DATA[CAL_OFFSET + CAMEL_CAL_RIGHT_OFFSET + 4] = value & 0xFF;
      CAL_DATA[CAL_OFFSET + CAMEL_CAL_RIGHT_OFFSET + 5] = (value >> 8) & 0xFF;
      CAL_DATA[CAL_OFFSET + CAMEL_CAL_RIGHT_OFFSET + 6] = (value >> 16) & 0xFF;
      CAL_DATA[CAL_OFFSET + CAMEL_CAL_RIGHT_OFFSET + 7] = GENCRC(CAL_DATA + CAMEL_CAL_RIGHT_OFFSET + CAL_OFFSET, 7); // CRC-8
      */
    }

  }
  // set the current weight value
  weight = lvalue + rvalue;

  if ( tareCount > 0 ) {
    tareIndex++;
    if ( tareIndex >= tareCount ) {
      // finish tare operation and reset values
      tareCount = 0;
      tareIndex = 0;
      // send a event notification
      osEventFlagsSet(eventFlagsId, EVENT_TARE_READY);
    }
  }
  if ( leftCalCount > 0 ) {
    leftCalIndex++;
    if ( leftCalIndex >= leftCalCount ) {
      leftCalCount = 0;
      leftCalIndex = 0;
      if ( dev.config.V.leftEnable ) {
        // this will store calibration data to EEPROM
        // disable this for now, handle it in the command processor
        // FUNC_FLAG = LEFT_MASK;
        // send an event notification
        osEventFlagsSet(eventFlagsId, EVENT_CAL_LEFT_READY);
      }
    }
  }
  if (rightCalCount > 0) {
    rightCalIndex++;
    if (rightCalIndex >= rightCalCount) {
      rightCalCount = 0;
      rightCalIndex = 0;
      if ( dev.config.V.rightEnable ) {
        // this will store calibration data to EEPROM
        // disable this for now, handle it in the command processor
        // FUNC_FLAG = RIGHT_MASK;
        // send an event notification
        osEventFlagsSet(eventFlagsId, EVENT_CAL_RIGHT_READY);
      }
    }
  }

}

void HX71xDevice::tare(uint8_t times) {
  if ( times > 0 ) {
    if ( times > 15 ) {
      times = 15;
    }
    tareIndex = 0;
    tareCount = times;
  }
}

bool HX71xDevice::isReady(HX71xPortConfiguration* cell) {
  return HAL_GPIO_ReadPin(cell->DOUT_Port, cell->DOUT_Pin) == GPIO_PIN_RESET;
}

void HX71xDevice::powerUp(CellEnum cell) {
  if ( cell == LEFT ) {
    HAL_GPIO_WritePin(leftCell.SCK_Port, leftCell.SCK_Pin, GPIO_PIN_RESET);
  } else {
    HAL_GPIO_WritePin(rightCell.SCK_Port, rightCell.SCK_Pin, GPIO_PIN_RESET);
  }
}

void HX71xDevice::powerDown(CellEnum cell) {
  if ( cell == LEFT ) {
    HAL_GPIO_WritePin(leftCell.SCK_Port, leftCell.SCK_Pin, GPIO_PIN_RESET);
    asm("NOP\n\tNOP\n\tNOP\n\tNOP\n\tNOP\n\tNOP");
    HAL_GPIO_WritePin(leftCell.SCK_Port, leftCell.SCK_Pin, GPIO_PIN_SET);
  } else {
    HAL_GPIO_WritePin(rightCell.SCK_Port, rightCell.SCK_Pin, GPIO_PIN_RESET);
    asm("NOP\n\tNOP\n\tNOP\n\tNOP\n\tNOP\n\tNOP");
    HAL_GPIO_WritePin(rightCell.SCK_Port, rightCell.SCK_Pin, GPIO_PIN_SET);
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  // notify load cell conversions are ready
  if ( eventFlagsId != NULL ) {
    if ( GPIO_Pin == LEFT_DOUT_Pin ) {
      osEventFlagsSet(eventFlagsId, EVENT_LEFT_READY);
    } else if ( GPIO_Pin == RIGHT_DOUT_Pin ) {
      osEventFlagsSet(eventFlagsId, EVENT_RIGHT_READY);
    }
  }
}
