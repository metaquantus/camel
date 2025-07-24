/*
 * camel_uart.c
 *
 *  Created on: Jul 10, 2025
 *      Author: cav
 */
#ifdef CAMEL_UART
#include "stm32l0xx_hal.h"
#include "main.h"
#include "camel_uart.h"
#include "utils.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* #define CAMEL_UART_TIMEOUT         150000 */
#define CAMEL_UART_SEND_QUEUE_SIZE 16

extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern uint8_t FUNC_FLAG;
extern uint32_t CMD_FLAG;

extern uint8_t SCALES_CONFIG;
extern uint8_t TARE_TIMES;
extern uint8_t TARE_INDEX;
extern uint8_t LEFT_CAL_TIMES;
extern uint8_t LEFT_CAL_INDEX;
extern float LEFT_CAL_VALUE;
extern uint8_t RIGHT_CAL_TIMES;
extern uint8_t RIGHT_CAL_INDEX;
extern float RIGHT_CAL_VALUE;
extern long LEFT_TARE_OFFSET;
extern long RIGHT_TARE_OFFSET;
extern float SCALES_VALUE;
extern CRC_HandleTypeDef hcrc;


// serial communications
uint8_t isSent = 1;
// uint8_t cmdDone = 1;
uint8_t dataBuf[UART_TX_DATA_SIZE];
uint8_t rxBuf[2];
uint8_t txBuf[4]; // in case we need to do some text formatting
uint8_t textBuf[UART_RX_DATA_SIZE];
uint8_t cmdBuf[UART_RX_DATA_SIZE];
int textIndex = 0;
// simple wrap around queue
UartSendItem_TypeDef sendQueue[CAMEL_UART_SEND_QUEUE_SIZE];
uint8_t sendStartIndex = 0;
uint8_t sendEndIndex = 0;
char *version = CAMEL_VERSION_STRING;
char *prompt = "\r\n> ";
char *linefeed = "\r\n";
char *cmdText = "CMD: ";

void showVersion() {
  sendText((uint8_t *)version, CAMEL_VERSION_STRING_LEN);
}

void showPrompt() {
  sendText((uint8_t *)prompt, 4);
}

void showLinefeed() {
  sendText((uint8_t *)linefeed, 2);
}

/**
 * Send text to serial interface.
 */
void sendText(uint8_t *buf, uint16_t size) {
  // queue the transmission
  sendQueue[sendEndIndex].buf = buf;
  sendQueue[sendEndIndex].size = size;
  sendEndIndex++;
	if ( sendEndIndex >= CAMEL_UART_SEND_QUEUE_SIZE ) {
	  sendEndIndex = 0;
	}
	processSendQueue();
}

void processSendQueue() {
  if ( (sendStartIndex != sendEndIndex) && isSent == 1) {
    int i=0;
    int timeout = 10000;
    // while (HAL_UART_GetState(&huart2) != HAL_UART_STATE_READY)
    while (huart2.gState != HAL_UART_STATE_READY)
    {
      if ( i > timeout) {
        break;
      }
      HAL_Delay(500);
      i++;
    }
    isSent = 0;
    if (huart2.gState == HAL_UART_STATE_READY ) {
      HAL_UART_Transmit_DMA(&huart2, sendQueue[sendStartIndex].buf, sendQueue[sendStartIndex].size);
      sendStartIndex++;
      if (sendStartIndex >= CAMEL_UART_SEND_QUEUE_SIZE) {
        sendStartIndex = 0;
      }
    }
  }
}

void uartInit() {
  textIndex = 0;
  sendStartIndex = 0;
  sendEndIndex = 0;
  showVersion();
  showPrompt();
  //HAL_Delay(1);
  receiveChar();
}

/**
 * Start receiving characters from the serial interface.
 *
 */
void receiveChar() {
  // HAL_UART_Receive_IT(&huart2, rxBuf, 1);
  /*
  // DMA set in circular mode, receiving forever
  uint8_t ready = 0;
  for (int i = 0; i < 1000; i++) {
    // HAL_UART_STATE_READY
    // state = HAL_UART_GetState (&huart2);
    if (huart2.RxState == HAL_UART_STATE_READY) {
      ready = 1;
      break;
    }
    HAL_Delay(1);
  }
  if (ready) {
    HAL_UART_Receive_DMA(&huart2, rxBuf, 2);
  }
  */
  // HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rxBuf, 2);
  int i=0;
  int timeout = 20000;
  // while (HAL_UART_GetState(&huart2) != HAL_UART_STATE_READY) {
  while (huart2.RxState != HAL_UART_STATE_READY) {
    if (i > timeout) {
      break;
    }
    HAL_Delay(500);
    i++;
  }
  if ( huart2.RxState  == HAL_UART_STATE_READY )  {
    if ( HAL_UART_Receive_DMA(&huart2, rxBuf, 2) != HAL_OK ) {
      Error_Handler();
    }
  }
}

/**
 * Receive one character, handle backspace and delete.
 * On carriage return, pass line to process command.
 * And start receiving next char.
 * It will keep receiving chars forever in interrupt mode.
 */
void processChar(uint8_t ch) {
  uint16_t tsize = 1;
  txBuf[0] = ch;
  uint32_t parsedCmd = 0;
  if (ch == '\b' ) { // handle BS
    if (textIndex > 0) {
      textIndex--;
      // send a space to erase current char
      // some ANSI sequence instead?, but this is more portable
      txBuf[1] = ' ';
      txBuf[2] = '\b';
      tsize = 3;
    } else {
      tsize = 0;
    }
  } else if ( ch == 127 ) {
    // ignore DEL chars, but send a space
    txBuf[0] = ' ';
    txBuf[1] = '\b';
    tsize = 2;
  } else {
    textBuf[textIndex] = ch;
  }

  if (ch == '\r' || ch == '\n') { // check both
    // copy line to command buffer
    // and keep receiving in textBuff
    if (textIndex > 0) {
      memcpy(cmdBuf, textBuf, textIndex);
      cmdBuf[textIndex] = 0;
      textIndex = 0;
      parsedCmd = CMD_READY_MASK;
    }
  } else if (ch != 127 && ch != '\b') {
    textIndex++;
  }
  // echo
  if (tsize > 0) {
    sendText(txBuf, tsize);
  }
  if (textIndex >= UART_RX_DATA_SIZE) {
    textIndex = 0; // wrap around
  }

  CMD_FLAG = parsedCmd;

  // receive next chart
  // receiveChar();

	/*
    char ch;
    if (Serial.available() > 0) {
        ch = Serial.read();
        chBuf[chIndex] = ch;
        Serial.printf("%c", ch);
        if ( ch == '\r') {
            Serial.print("\n");
            memcpy(cmdBuf, chBuf, chIndex);
            cmdBuf[chIndex] = 0;
            processCmd();
            chIndex = 0;
        } else {
            chIndex++;
        }
        if ( chIndex >= CH_BUF_SIZE) {
            chIndex = 0;
        }

    }
    */
}

void processCmd(void) {

  if ( (CMD_FLAG & CMD_TARE_WAIT_MASK) != 0 ) {
    if ( TARE_TIMES > 0 ) {
      // wait until tare complete
      return;
    } else {
      CMD_FLAG = 0;
      sprintf((char *)dataBuf, "\r\nTARE: left=%ld, right=%ld\r\n", LEFT_TARE_OFFSET, RIGHT_TARE_OFFSET);
      sendText(dataBuf, strlen((const char *)dataBuf));
      showPrompt();
      return;
    }
  }

  CMD_FLAG = 0;
  char delim[5] = " \t\r\n";
  char *token = strtok((char *)cmdBuf, delim);
  if ( !strcmp(token, "c") || !strcmp(token, "config") ) {
    char *token2 = strtok(NULL, delim);
    if (token2 != NULL) {
      char* endptr;
      unsigned int c = strtoul(token2, &endptr, 16);
      if (c > 0) {
        sprintf((char*) dataBuf, "\r\nConfiguring Camel = %02X", c);
        sendText(dataBuf, strlen((const char*) dataBuf));
        SCALES_CONFIG = c & 0xFF;
        FUNC_FLAG = CONFIG_MASK;
      } else {
        printf((char*) dataBuf, "\r\nCamel configuration = %02X", SCALES_CONFIG);
        sendText(dataBuf, strlen((const char*) dataBuf));
      }
      /*
      int n = sscanf(token2, "%x", &c);
      if (n == 1) {
        // Serial.printf("Configuring Camel = %02X\r\n", c);
        sprintf((char *)dataBuf, "\r\nConfiguring Camel = %02X", c);
        sendText(dataBuf, strlen((const char *)dataBuf));
        SCALES_CONFIG = c & 0xFF;
        FUNC_FLAG = CONFIG_MASK;
      } else if (n == EOF) {
        sprintf((char *)dataBuf, "\r\nCamel configuration = %02X", SCALES_CONFIG);
        sendText(dataBuf, strlen((const char *)dataBuf));
      }
      */

    } else {
      sprintf((char*) dataBuf, "\r\nCamel configuration = %02X", SCALES_CONFIG);
      sendText(dataBuf, strlen((const char*) dataBuf));
    }
    showPrompt();
  } else if ( !strcmp(token, "v") ) {
    showLinefeed();
    showVersion();
    showLinefeed();
    showPrompt();
  }
  /*
  else if ( !strcmp(token, "crc") ) {
    static uint8_t usage = 0;
    char* token2 = strtok(NULL, delim);
    if ( token2 != NULL ) {
      uint8_t buf[4] = {0, 0, 0, 0};
      uint32_t a, b, c;
      int n = sscanf(token2, "%02X%02X%02X", &a, &b, &c);
      if ( n != EOF) {
        buf[0] = c & 0xFF;
        buf[1] = b & 0xFF;
        buf[2] = a & 0xFF;
        buf[3] = usage++;
        uint8_t crc = GENCRC(buf, 3);
        sprintf((char *)dataBuf, "\r\nCRC = %02X", crc);
        sendText(dataBuf, strlen((const char *)dataBuf));
      }
    }
    showPrompt();
  } */
  /*
  else if ( !strcmp(token, "cl")) {
    char* token2 = strtok(NULL, delim);
    if ( token2 != NULL ) {
      unsigned int offset = 0;
      int n = sscanf(token2, "%d", &offset);
      if ( n!= EOF ){
        float weight = 0.0;
        char* token3 = strtok(NULL, delim);
        if ( token3 != NULL ) {
          // n = sscanf(token3, "%f", &weight);
          // weight = (float)atof(token3);
          weight = atoff(token3);
          uint8_t * wptr = (uint8_t *)&weight;
          uint8_t a = * (wptr + 3);
          uint8_t b = * (wptr + 2);
          uint8_t c = * (wptr + 1);
          uint8_t d = * (wptr);
          char str[16];
          fftoa(weight, str, 2);
          sprintf((char*)dataBuf, "\r\nOffset = %d, FLOAT = %02X%02X%02X%02X %s %d", offset, a, b, c, d, str, (unsigned int)weight);
          sendText(dataBuf, strlen((const char *)dataBuf));
        }
      }
    }
    showPrompt();
  }*/
  else if (!strcmp(token, "t") || !strcmp(token, "tare")) {
    char *token2 = strtok(NULL, delim);
    if (token2 == NULL) {
      TARE_INDEX = 0;
      TARE_TIMES = 1;
    } else {
      int times = atoi(token2);
      /*
      int n = sscanf(token2, "%d", &times);
      if (n == EOF) {
        times = 1;
      } else if (times < 0 || times > 16) {
        times = 1;
      }
      */
      if ( times > 0 ) {
        if ( times > 16 ) {
          times = 1;
        }
        TARE_INDEX = 0;
        TARE_TIMES = times;
      }

    }
    if ( TARE_TIMES > 0 ) {
      // wait for result
      CMD_FLAG = CMD_TARE_WAIT_MASK;
    }
  } else {
    sendText((uint8_t*) cmdText, 5);
    sendText(cmdBuf, strlen((const char*) cmdBuf));
    showPrompt();
  }

}



/*
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t offset) {

    static uint16_t last_offset = 0;

    //printf("Offset %d\n", offset);
    // led_flash();

    // Ignore if called twice (which will happen on every half buffer)
    if (offset != last_offset) {

        // If wrap around reset last_size
        if (offset < last_offset)
            last_offset = 0;

        while (last_offset < offset) {
            // process_character((char) dma_buffer[last_offset]);
          processChar(rxBuf[last_offset]);
            ++last_offset;
        }
    }
}
*/

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart) {
  processChar(rxBuf[0]);
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	processChar(rxBuf[1]);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	isSent = 1;
	processSendQueue();
}

#endif

