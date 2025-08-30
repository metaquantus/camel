/*
 * camel.cpp
 *
 * The main task of this board is to configure and read the load cells periodically.
 *
 * This task receives configuration changes from the host comms or the console and
 * sends a notification when a new value is ready.
 *
 *
 *  Created on: Aug 4, 2025
 *      Author: cav
 */
#include "main.h"
#include "cmsis_os.h"
#include "camel.h"
#include "scales.h"
#include "indicator.h"

/**
 * The device.
 * It handles both cells left and right;
 */
HX71xDevice hx71x;

osEventFlagsId_t eventFlagsId;
osMessageQueueId_t commandQueue;
extern osMessageQueueId_t indicatorQueue;

// the current values
int32_t LEFT_VALUE = 0;
int32_t RIGHT_VALUE = 0;
float WEIGHT = 0.0;

void executeCommand(CommandMessage cmd);
void showBoot();
void showConfig();

/*
 * This task runs at normal priority higher than other tasks so that
 * the cell readings are not preempted.
 */
extern "C" void StartDefaultTask(void *argument)
{
  eventFlagsId = osEventFlagsNew(NULL);

  commandQueue = osMessageQueueNew(COMMAND_QUEUE_SIZE, sizeof(CommandMessage), nullptr);

  /*
  if (eventFlagsId == NULL) {
    ; // Event Flags object not created, we'll rely on polling only
  }
  */

  hx71x.init();
  // give some time for other threads to initialize
  osDelay(50);

  showBoot();
  osDelay(200);
  showConfig();

  for(;;)
  {

    // wait for both cells to be ready, if both enabled, and read them at the same time
    // so next time they both will be ready about the same time
    bool leftReady = hx71x.isCellEnabled(LEFT) && hx71x.isCellReady(LEFT);
    bool rightReady = hx71x.isCellEnabled(RIGHT) && hx71x.isCellReady(RIGHT);
    if ((leftReady && rightReady) ||
        (!hx71x.isCellEnabled(RIGHT) && leftReady) ||
        (!hx71x.isCellEnabled(LEFT) && rightReady)) {

      hx71x.read();

      if ( hx71x.isCellEnabled(LEFT) ) {
        LEFT_VALUE = hx71x.getValue(LEFT);
      }
      if ( hx71x.isCellEnabled(RIGHT) ) {
        RIGHT_VALUE = hx71x.getValue(RIGHT);
      }
      WEIGHT = hx71x.getWeight();
    }

    // process available commands, don't block
    CommandMessage cmd;
    while(osMessageQueueGet(commandQueue, &cmd, 0U, 0U) == osOK) {
      executeCommand(cmd);
    }

    // Wait for next reading, also give chance to other tasks to run
    // at 40Hz sampling rate, next reading will be in 25ms
    // at 10Hz sampling rate, next reading will be in 100ms
    if ( eventFlagsId != NULL ) {
      // this should unblock as soon as one of the cells is ready or timeout expires
      osEventFlagsWait(eventFlagsId, EVENT_LEFT_READY | EVENT_RIGHT_READY | EVENT_CMD_READY, osFlagsWaitAny, SCALES_POLL_INTERVAL);
    } else {
      osDelay(1);
    }

  }
}

void executeCommand(CommandMessage cmd) {
  switch (cmd.command) {
  case TARE:
    hx71x.tare(cmd.arg.tare.times);
    break;
  case CONFIG:
    hx71x.setCellConfiguration(cmd.arg.config);
    break;
  case SHOW_CONFIG:
    showBoot();
    showConfig();
    break;
  case SHOW_BOOT:
    showBoot();
    break;
  case CAL_COUNT:
    hx71x.setCalibrationCount(cmd.arg.count);
    break;
  case READ_MODE:
    hx71x.setReadMode(cmd.arg.mode);
    break;
  case CAL_LEFT:
    hx71x.calibrate(LEFT, cmd.arg.cal.index, cmd.arg.cal.weight, cmd.arg.cal.count);
    break;
  case CAL_RIGHT:
    hx71x.calibrate(RIGHT, cmd.arg.cal.index, cmd.arg.cal.weight, cmd.arg.cal.count);
    break;
  case SAVE_ALL:
    hx71x.save();
    break;
  default:
    break;
  }
}

void showBoot() {
  Indication signal = BOOT;
  osMessageQueuePut(indicatorQueue, &signal, 0U, 10);
  osDelay(200);
}

void showConfig() {
  // show configuration via LED
  Indication signal;
  if (hx71x.isCellEnabled(LEFT) && hx71x.isCellEnabled(RIGHT)) {
    signal = CONFIG_BOTH;
    osMessageQueuePut(indicatorQueue, &signal, 0U, 10);
  } else if (hx71x.isCellEnabled(LEFT)) {
    signal = CONFIG_LEFT;
    osMessageQueuePut(indicatorQueue, &signal, 0U, 10);
  } else if (hx71x.isCellEnabled(RIGHT)) {
    signal = CONFIG_RIGHT;
    osMessageQueuePut(indicatorQueue, &signal, 0U, 10);
  }
}

