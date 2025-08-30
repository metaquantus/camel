/*
 * indicator.h
 *
 *  Created on: Aug 29, 2025
 *      Author: cav
 */

#ifndef INC_INDICATOR_H_
#define INC_INDICATOR_H_

enum Indication : uint8_t {
  BOOT,
  CONFIG_LEFT,
  CONFIG_RIGHT,
  CONFIG_BOTH
};

#define INDICATOR_QUEUE_SIZE 8


#endif /* INC_INDICATOR_H_ */
