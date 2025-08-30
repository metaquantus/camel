/*
 * camel.h
 *
 *  Created on: Aug 17, 2025
 *      Author: cav
 */

#ifndef INC_CAMEL_H_
#define INC_CAMEL_H_

#define EVENT_LEFT_READY        0x00000002U
#define EVENT_RIGHT_READY       0x00000004U
#define EVENT_CMD_READY         0x00000008U
#define EVENT_TARE_READY        0x00000010U
#define EVENT_CAL_LEFT_READY    0x00000020U
#define EVENT_CAL_RIGHT_READY   0x00000040U
#define EVENT_CONFIG_READY      0x00000080U
#define EVENT_CAL_COUNT_READY   0x00000100U
#define EVENT_READ_MODE_READY   0x00000200U
#define EVENT_CONFIG_LOAD_READY 0x00000400U
#define EVENT_CONFIG_SAVE_READY 0x00000800U

#define SCALES_POLL_INTERVAL 5
#define COMMAND_QUEUE_SIZE 16

enum Command : uint8_t {
  NIL,
  CONFIG,
  SHOW_CONFIG,
  SHOW_BOOT,
  CAL_COUNT,
  READ_MODE,
  TARE,
  CAL_LEFT,
  CAL_RIGHT,
  SAVE_CONFIG,
  SAVE_CAL,
  SAVE_ALL
};

/*
struct ConfigCommandArg {
  uint8_t config;
};

struct CountCommandArg {
  uint8_t count;
};

struct ReadModeCommandArg {
  uint8_t mode;
};
*/

struct TareCommandArg {
  uint8_t times;
};

struct CalibrationCommandArg {
  uint8_t count : 4;
  uint8_t index : 3;
  uint8_t isLeft : 1;
  float weight;
};

union CommandArgUnion {
  uint8_t config;
  uint8_t count;
  uint8_t mode;
  TareCommandArg tare;
  CalibrationCommandArg cal;
};

struct CommandMessage {
  Command command = NIL;
  CommandArgUnion arg;
};

#endif /* INC_CAMEL_H_ */
