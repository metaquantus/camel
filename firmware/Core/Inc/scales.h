/*
 * scales.h
 *
 *  Created on: Aug 1, 2025
 *      Author: cav
 */

#ifndef INC_SCALES_H_
#define INC_SCALES_H_

#include "main.h"

// cells are reversed in PCB rev 2+ to optimise layout
#define LEFT_SCK_Pin        SCK1_Pin
#define LEFT_SCK_GPIO_Port  SCK1_GPIO_Port
#define LEFT_DOUT_Pin       DOUT1_Pin
#define LEFT_DOUT_GPIO_Port DOUT1_GPIO_Port

#define RIGHT_DOUT_Pin       DOUT2_Pin
#define RIGHT_DOUT_GPIO_Port DOUT2_GPIO_Port
#define RIGHT_SCK_Pin        SCK2_Pin
#define RIGHT_SCK_GPIO_Port  SCK2_GPIO_Port

#define CAMEL_DEFAULT_CONFIG 0x33
#define CAMEL_MAX_CAL_COUNT  4


/**
 * A calibration point used to compute the real weight value from a raw value read from the cell's ADC.
 * Up to 4 calibration points can be stored in the EEPROM. Ideally the weight cell response should be linear.
 * So only one point would be necessary. However, This builds up to a 4 segment line for better accuracy.
 * There's room in the EEPROM to store more points but it seems unnecessary,
 * typical cells response is almost linear within their range.
 */
struct CalibrationPoint {
  float scale;    // scale factor
  int32_t value;  // cell value, HX712/HX711 ADCs have 24bits resolution so only 3 bytes sent to host
  float weight;   // original calibration weight, for convenience
  uint32_t flag : 24; // Miscellaneous flags, not currently used, padding only
  uint32_t crc : 8;   // CRC-8 for validating data read from EEPROM at boot.
};

struct CalibrationEntry {
  CalibrationPoint Left;
  CalibrationPoint Right;
};

struct CellConfiguration {
  uint8_t rightEnable : 1;
  uint8_t rightGain: 1;  // 0: 128, 1: 256 for HX712 or 0: 64, 1: 128 for HX711
  uint8_t rightRate: 1;  // 0: 10Hz, 1: 40Hz for HX712
  uint8_t rightModified: 1; // for keeping track of applied changes
  uint8_t leftEnable : 1;
  uint8_t leftGain : 1;
  uint8_t leftRate : 1;
  uint8_t leftModified: 1;
};

/**
 * For convenience as configuration is moved around compacted in a single byte.
 */
union CellConfigurationUnion {
  uint8_t value;
  CellConfiguration V;
};

/**
 * Device configuration stored in EEPROM (4-bytes)
 */
struct DeviceConfiguration {
  CellConfigurationUnion config;  // cell configuration 4-bit MSB for left, 4-bit LSB for right
  uint8_t count : 4;   // calibration count 0 to 4, calibration points in CAL_DATA and EEPROM
  uint8_t mode : 4;    // read mode for sending data to host: 0 (6bytes cell value only), 1 (4bytes weight value only),
                       // 2 (10bytes, both)
  uint8_t leftProg : 4;   // number of pulses used to program the left HX71x into the correct gain and sampling rate:
  uint8_t rightProg : 4;  // number of pulses used to program the right HX71x into the correct gain and sampling rate:
                          // 1, 2, 3, 4, 5: 128/10Hz, BAT/40Hz, 128/40Hz, 256/10Hz, 256/40Hz
  uint8_t crc;         // CRC-8
};

struct HX71xPortConfiguration {
  GPIO_TypeDef *DOUT_Port;
  uint16_t DOUT_Pin;
  GPIO_TypeDef *SCK_Port;
  uint16_t SCK_Pin;
};

struct CellReadByteValue {
  uint8_t leftValue;
  uint8_t rightValue;
};

enum CellEnum {
  LEFT,
  RIGHT
};

class HX71xDevice {
private:
  HX71xPortConfiguration leftCell = { LEFT_DOUT_GPIO_Port, LEFT_DOUT_Pin, LEFT_SCK_GPIO_Port, LEFT_SCK_Pin };
  HX71xPortConfiguration rightCell= { RIGHT_DOUT_GPIO_Port, RIGHT_DOUT_Pin, RIGHT_SCK_GPIO_Port, RIGHT_SCK_Pin };
  // config stored in EEPROM also
  DeviceConfiguration dev = {0x33, 0, 0, 4, 4, 0xff};
  // cache, this is stored in EEPROM also
  CalibrationEntry calData[CAMEL_MAX_CAL_COUNT];

  // current readings
  int32_t leftValue;
  int32_t rightValue;
  float weight;
  CellReadByteValue shiftIn();
  bool isReady(HX71xPortConfiguration* cell);
  void parseConfig();
  void readConfig();
  void writeConfig();
  // tare
  uint8_t tareCount=0;
  uint8_t tareIndex=0;
  int32_t leftTareOffset=0;
  int32_t rightTareOffset=0;
  // calibration
  uint8_t leftCalCount=0;
  uint8_t leftCalIndex=0;
  float leftCalWeight=0.0;
  uint8_t rightCalCount=0;
  uint8_t rightCalIndex=0;
  float rightCalWeight=0.0;

  uint8_t calIndex=0; // data point to calibrate or read/write 0 - 3

public:
  void init();

  void setCalibrationPoint(CalibrationEntry entry, uint8_t index);
  CalibrationEntry getCalibrationPoint(uint8_t index) {
    return calData[index];
  }

  void setCellConfiguration(uint8_t cnf);

  DeviceConfiguration getDeviceConfig() {
    return dev;
  }

  bool calibrate(CellEnum cell, uint8_t index, float weight, uint8_t count = 1);

  void setCalibrationCount(uint8_t count);
  void setReadMode(uint8_t mode);

  uint8_t getCalibrationCount() {
    return dev.count;
  }

  uint8_t getReadMode() {
    return dev.mode;
  }

  bool isCellEnabled(CellEnum cell) {
    if ( cell == LEFT ) {
      return dev.config.V.leftEnable;
    } else {
      return dev.config.V.rightEnable;
    }
  }

  bool isCellReady(CellEnum cell) {
    if ( cell == LEFT ) {
      return isReady(&leftCell);
    } else {
      return isReady(&rightCell);
    }
  }

  // read cells, they must be in ready state
  void read();


  int32_t getValue(CellEnum cell) {
    if ( cell == LEFT ) {
      return leftValue;
    } else {
      return rightValue;
    }
  }

  float getWeight() {
    // weight is the sum of both left and right values
    return weight;
  }

  int32_t getTareOffset(CellEnum cell) {
    if ( cell == LEFT) {
      return leftTareOffset;
    } else {
      return rightTareOffset;
    }
  }

  void powerUp(CellEnum cell);

  void powerDown(CellEnum cell);

  void tare(uint8_t times = 1);

  void save() {
    writeConfig();
  }

};

#endif /* INC_SCALES_H_ */
