/*
 * serial.cpp
 *
 *  Created on: Aug 15, 2025
 *      Author: cav
 */
#include "serial.h"
#include <cstring>
#include <cstdio>
#include <cstdarg>

#if SERIAL_MAX_INSTANCES == 2
Serial* Serial::_instances[SERIAL_MAX_INSTANCES] = { nullptr, nullptr };
#endif
uint8_t Serial::_instanceIndex = 0;

/**
 * To support = assignments from c strings, (using implicit copy constructor)
 */
SerialMessage::SerialMessage(const char* str) {
  size = strlen(str);
  // truncates if larger than buffer size
  if ( size > SERIAL_MESSAGE_SIZE ) {
    size = SERIAL_MESSAGE_SIZE;
  }
  memcpy(text, str, size);

}

// not needed if SERIAL_MAX_INSTANCES == 2
void Serial::initInstances() {
  for (uint8_t i = 0; i < SERIAL_MAX_INSTANCES; i++) {
    _instances[i] = nullptr;
  }
}

void Serial::init(void) {
  // initiate static state so we can channel the ISR to
  // the proper Serial instance
  serialIndex = _instanceIndex++;
  if (_instanceIndex >= SERIAL_MAX_INSTANCES) {
    _instanceIndex = 0;
  }
  _instances[serialIndex] = this;

  sendQueue = osMessageQueueNew(SERIAL_SEND_QUEUE_SIZE, sizeof(SerialMessage), nullptr);
}

osStatus_t Serial::print(const char* str, uint32_t timeout) {
  SerialMessage msg(str);
  return send(&msg, timeout);
}

int Serial::printf(uint32_t timeout, const char *format, ...) {
  SerialMessage msg;
  va_list args;
  va_start(args, format);
  int n = vsnprintf((char*) msg.text, SERIAL_MESSAGE_SIZE, format, args);
  va_end(args);
  if (n > 0) {
    msg.size = n;
    osStatus_t status = send(&msg, timeout);
    if (status != osOK) {
      return status;
    }
  }
  return n;
}

// note that this can't be called from ISRs
int Serial::printf(const char *format, ...) {
  SerialMessage msg;
  va_list args;
  va_start(args, format);
  int n = vsnprintf((char *)msg.text, SERIAL_MESSAGE_SIZE, format, args);
  va_end(args);
  if ( n > 0 ) {
    msg.size = n;
    osStatus_t status = send(&msg, osWaitForever);
    if ( status != osOK ) {
      return status;
    }
  }
  return n;
}

// can't be called from ISR if timeout != 0
osStatus_t Serial::send(const SerialMessage *msg, uint32_t timeout) {
  osStatus_t status = osMessageQueuePut(sendQueue, msg, 0U, timeout);
  runSendQueue();
  return status;
}

void Serial::runSendQueue() {
  /* this would block even from an ISR, rely on the message queue instead
  int i = 0;
  // while (HAL_UART_GetState(&huart2) != HAL_UART_STATE_READY)
  while (huart->gState != HAL_UART_STATE_READY) {
    if (i > SERIAL_TX_TIMEOUT) {
      break;
    }
    HAL_Delay(1);
    i++;
  }
  */
  if (huart->gState == HAL_UART_STATE_READY) {
    osStatus_t status = osMessageQueueGet(sendQueue, &sendMsg, 0U, 0U);
    if ( status == osOK) {
      HAL_UART_Transmit_DMA(huart, sendMsg.text, sendMsg.size);
    }
  }
}

void Serial::txCpltCallback() {
  runSendQueue();
}

HAL_StatusTypeDef Serial::startCharReceive() {
  if ( receiving ) {
    return HAL_OK;
  }
  int i = 0;
  // while (HAL_UART_GetState(&huart2) != HAL_UART_STATE_READY) {
  while (huart->RxState != HAL_UART_STATE_READY) {
    if (i > SERIAL_RX_TIMEOUT ) {
      break;
    }
    HAL_Delay(1);
    i++;
  }
  if (huart->RxState == HAL_UART_STATE_READY) {
    // request receive in DMA mode
    HAL_StatusTypeDef s = HAL_UART_Receive_DMA(huart, rxBuf, 2);
    if ( s == HAL_OK ) {
      receiving = true;
    }
    return s;
  }
  return HAL_BUSY;
}

HAL_StatusTypeDef Serial::stopCharReceive() {
  if ( receiving ) {
    HAL_StatusTypeDef s = HAL_UART_DMAStop(huart);
    if ( s == HAL_OK ) {
      receiving = false;
    }
    return s;
  }
  return HAL_ERROR;
}

void Serial::rxHalfCpltCallback() {
  rxCharCpltCallback(rxBuf[0]);
}

void Serial::rxCpltCallback() {
  rxCharCpltCallback(rxBuf[1]);
}


void Serial::onTxCpltCallback(UART_HandleTypeDef *uart) {
  // find instance, we only have a couple of instances
  for (uint8_t i = 0; i < SERIAL_MAX_INSTANCES; i++) {
    if (_instances[i] != nullptr && _instances[i]->huart == uart) {
      _instances[i]->txCpltCallback();
      break;
    }
  }
}

void Serial::onRxCpltCallback(UART_HandleTypeDef *uart) {
  for (uint8_t i = 0; i < SERIAL_MAX_INSTANCES; i++) {
    if (_instances[i] != nullptr && _instances[i]->huart == uart) {
      _instances[i]->rxCpltCallback();
      break;
    }
  }
}

void Serial::onRxHalfCpltCallback(UART_HandleTypeDef *uart) {
  for (uint8_t i = 0; i < SERIAL_MAX_INSTANCES; i++) {
    if (_instances[i] != nullptr && _instances[i]->huart == uart) {
      _instances[i]->rxHalfCpltCallback();
      break;
    }
  }
}

extern "C" {

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart) {
  Serial::onRxHalfCpltCallback(huart);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  Serial::onRxCpltCallback(huart);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  Serial::onTxCpltCallback(huart);
}

}
