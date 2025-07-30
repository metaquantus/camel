/*
 * camel_uart.h
 *
 *  Created on: Jul 10, 2025
 *      Author: cav
 */

#ifndef INC_CAMEL_UART_H_
#define INC_CAMEL_UART_H_

#ifdef CAMEL_UART
#define UART_TX_DATA_SIZE 80
#define UART_RX_DATA_SIZE 80

#define CMD_READY_MASK          1
#define CMD_TARE_WAIT_MASK      0x00800000UL
#define CMD_LEFT_CAL_WAIT_MASK  0x00400000UL
#define CMD_RIGHT_CAL_WAIT_MASK 0x00400000UL

typedef struct {
  uint8_t *buf;
  uint16_t size;
} UartSendItem_TypeDef ;

void uartInit();
void sendText(uint8_t *buf, uint16_t size);
void receiveChar(void);
void processChar(uint8_t ch);
void processSendQueue(void);
void processCmd(void);
void showVersion(void);
void showPrompt(void);

#endif /* CAMEL_UART */

#endif /* INC_CAMEL_UART_H_ */
