/*
 * indicator.cpp
 *
 *  Created on: Aug 29, 2025
 *      Author: cav
 */
#include "main.h"
#include "cmsis_os.h"
#include "indicator.h"

osMessageQueueId_t indicatorQueue;

void executeIndication(Indication cmd);
void blink(uint32_t offTime, uint32_t onTime, uint16_t count);

extern "C" void StartIndicatorTask(void *argument)
{
  indicatorQueue = osMessageQueueNew(INDICATOR_QUEUE_SIZE, sizeof(Indication), nullptr);

  for(;;)
  {
    Indication cmd;
    osStatus_t status = osMessageQueueGet(indicatorQueue, &cmd, 0U, osWaitForever);
    if ( status == osOK ) {
      executeIndication(cmd);
    }
    osDelay(1);
  }
}

void executeIndication(Indication cmd) {
  switch (cmd) {
  case BOOT:
    blink(300, 150, 3);
    break;
  case CONFIG_LEFT:
    blink(1000, 1000, 1);
    break;
  case CONFIG_RIGHT:
    blink(1000, 1000, 2);
    break;
  case CONFIG_BOTH:
    blink(1000, 1000, 3);
    break;
  default:
    break;
  }
}

void blink(uint32_t offTime, uint32_t onTime, uint16_t count) {
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
  osDelay(100);
  for(uint16_t i=0; i<count; i++) {
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
    osDelay(onTime);
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
    osDelay(offTime);
  }
}

