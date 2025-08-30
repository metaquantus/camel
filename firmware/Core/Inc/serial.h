/*
 * serial.h
 *
 *  Created on: Aug 15, 2025
 *      Author: cav
 */

#ifndef INC_SERIAL_H_
#define INC_SERIAL_H_

#include "main.h"
#include "cmsis_os.h"

// keep message small
#define SERIAL_MESSAGE_SIZE 78
#define SERIAL_MAX_INSTANCES 2
#define SERIAL_SEND_QUEUE_SIZE 8
#define SERIAL_RX_TIMEOUT 10000
#define SERIAL_TX_TIMEOUT 10000

#ifdef __cplusplus

// simple text message
struct SerialMessage {
  uint16_t size;
  uint8_t text[SERIAL_MESSAGE_SIZE];

  SerialMessage() {}
  SerialMessage(const char* str);

};

// Mainly use DMA mode to free CPU
class Serial {
private:
  UART_HandleTypeDef* huart;
  osMessageQueueId_t sendQueue;
  SerialMessage sendMsg;
  uint8_t rxBuf[2]; // for receiving chars in DMA mode

  static Serial* _instances[SERIAL_MAX_INSTANCES]; // max 2 instances
  static uint8_t _instanceIndex;
  uint8_t serialIndex;

  void runSendQueue();
  void txCpltCallback();
  void rxCpltCallback();
  void rxHalfCpltCallback();

protected:
  bool receiving;

public:
  Serial(UART_HandleTypeDef* u) : huart(u), receiving(false) { }
  static void initInstances();
  void init();

  bool isReceiving() { return receiving; }

  /**
   * Send text to serial using a message queue.
   * If queue is full it will block until available
   * depending on timeout argument.
   * Call will return upon queuing of message and transfer will be done in DMA mode.
   * Return the status of osMessageQueuePut.
   *
   */
  osStatus_t send(const SerialMessage* msg, uint32_t timeout = osWaitForever);
  osStatus_t print(const char* msg, uint32_t timeout = osWaitForever);

  /**
   * Like printf but will wait until queue has room for the message.
   * Returns the osStatus_t if error queuing message
   */
  int printf(const char * format, ... );
  int printf(uint32_t timeout, const char * format, ... );

  /**
   *  Start receiving chars from serial, one at a time.
   *  It uses DMA mode in loop mode.
   */
  HAL_StatusTypeDef startCharReceive();

  /**
   * Stop receiving chars from serial.
   */
  HAL_StatusTypeDef stopCharReceive();

  /**
   *  Callback when char is available.
   *  This will be running in a ISR.
   */
  virtual void rxCharCpltCallback(uint8_t ch) {
    // to override
  }

  // to bridge to HAL callbacks
  static void onTxCpltCallback(UART_HandleTypeDef* uart);
  static void onRxCpltCallback(UART_HandleTypeDef* uart);
  static void onRxHalfCpltCallback(UART_HandleTypeDef* uart);
};

#endif

#endif /* INC_SERIAL_H_ */
