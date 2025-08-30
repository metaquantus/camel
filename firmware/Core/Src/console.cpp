/*
 * console.cpp
 *
 *  Created on: Jul 31, 2025
 *      Author: cav
 */
#include "main.h"
#include "cmsis_os.h"
#include "serial.h"
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <climits>
#include "scales.h"
#include "camel.h"

#define CAMEL_VERSION "4.0"
#define CAMEL_TITLE_STRING "\r\nCamel Dual HX712 Weigh Scales ADC"
#define CAMEL_TITLE_STRING_LEN 35
#define CAMEL_VERSION_STRING "\r\nVersion: 4.0"
#define CAMEL_VERSION_STRING_LEN 14
#define PROMPT_STRING "\r\n> "
#define LINEFEED "\r\n"
#define CMD_LINE_MAX_LEN 80
#define LINE_WAIT_FLAG 0x00000001U

extern UART_HandleTypeDef huart1;
extern HX71xDevice hx71x;
extern int32_t LEFT_VALUE;
extern int32_t RIGHT_VALUE;
extern float WEIGHT;
extern osMessageQueueId_t commandQueue;
extern osEventFlagsId_t eventFlagsId;

class SerialConsole : public Serial {
private:
  SerialMessage txBuf;
  uint8_t textBuf[CMD_LINE_MAX_LEN];
  uint16_t textIndex;
  char *cmdBuf;
  osThreadId_t myTaskId;

public:
  SerialConsole(UART_HandleTypeDef* uart) : Serial(uart), textIndex(0), cmdBuf(nullptr) {}
  void rxCharCpltCallback(uint8_t ch) override;
  /**
   * Reads a line from the console, it will block until line is received.
   * Line is identified by ending on \r or \n
   */
  void readLine(char *line);
};

SerialConsole console (&huart1);
// SerialMessage msg;

void showVersion(void);
void showPrompt(void);
void showLinefeed(void);
void processCmd(char* cmd);

// This is a low priority task, it should not preempt the default task
extern "C" void StartConsoleTask(void *argument)
{
  char cmdBuf[CMD_LINE_MAX_LEN];

  console.init();
  showVersion();
  showPrompt();
  for(;;)
  {
    console.readLine(cmdBuf);
    processCmd(cmdBuf);
    osDelay(1);
  }
}

void showVersion() {
  // msg = CAMEL_TITLE_STRING;
  console.print(CAMEL_TITLE_STRING);
  // msg = CAMEL_VERSION_STRING;
  console.print(CAMEL_VERSION_STRING);
}

void showPrompt() {
  // msg = PROMPT_STRING;
  console.print(PROMPT_STRING);
}

void showLinefeed() {
  // msg = LINEFEED;
  console.print(LINEFEED);
}


void processCmd(char* cmdBuf) {

  // CMD_FLAG = 0;
  char delim[5] = " \t\r\n";
  char *token = strtok(cmdBuf, delim);
  if ( !strcmp(token, "c") || !strcmp(token, "config") ) {
    /*
     * syntax: c <value>
     * Configures cells, value in hex, e.g. 33
     * If value missing, display current configuration.
     */
    char *token2 = strtok(NULL, delim);
    if (token2 != NULL) {
      unsigned int c = strtoul(token2, nullptr, 16);
      if (c != ULONG_MAX ) {
        // sprintf(dataBuf, "\r\nConfiguring Camel = %02X", c);
        console.printf("\r\nConfiguring Camel = %02X", c);
        CommandMessage cmd;
        cmd.command = CONFIG;
        cmd.arg.config = c;
        osStatus_t status = osMessageQueuePut(commandQueue, &cmd, 0U, 100);
        if (status == osOK) {
          uint32_t flag = osEventFlagsWait(eventFlagsId, EVENT_CONFIG_READY, osFlagsWaitAny, 2000);
          if ( (flag & EVENT_CONFIG_READY) !=  0) {
            console.printf("\r\nNew Camel configuration = %02X", hx71x.getDeviceConfig().config.value);
            cmd.command = SHOW_CONFIG;
            osMessageQueuePut(commandQueue, &cmd, 0U, 10);
          }
        }
      } else {
        // sprintf(dataBuf, "\r\nCamel configuration = %02X", hx71x.getDeviceConfig().config.value);
        console.printf("\r\nCamel configuration = %02X", hx71x.getDeviceConfig().config.value);
      }

    } else {
      // sprintf(dataBuf, "\r\nCamel configuration = %02X",  hx71x.getDeviceConfig().config.value);
      console.printf("\r\nCamel configuration = %02X",  hx71x.getDeviceConfig().config.value);
    }
    showPrompt();
  } else if ( !strcmp(token, "save") ) {
    CommandMessage cmd;
    cmd.command = SAVE_ALL;
    osStatus_t status = osMessageQueuePut(commandQueue, &cmd, 0U, 100);
    if (status == osOK) {
      uint32_t flag = osEventFlagsWait(eventFlagsId, EVENT_CONFIG_SAVE_READY, osFlagsWaitAny, 2000);
      if ((flag & EVENT_CONFIG_SAVE_READY) != 0) {
        console.print("\r\nCamel configuration saved");
        cmd.command = SHOW_BOOT;
        osMessageQueuePut(commandQueue, &cmd, 0U, 10);
      }
    }
    showLinefeed();
    showPrompt();
  } else if ( !strcmp(token, "ca") || !strcmp(token, "print") || !strcmp(token, "p")) {
    // show all configuration
    DeviceConfiguration dev = hx71x.getDeviceConfig();
    uint8_t count = hx71x.getCalibrationCount();
    console.printf("\r\nCamel configuration = %02X", dev.config.value);
    console.printf("\r\n\tLEFT:  enabled = %d, gain = %d, rate = %dHz",
        dev.config.V.leftEnable, dev.config.V.leftGain ? 256 : 128, dev.config.V.leftRate ? 40 : 10);
    console.printf("\r\n\tRIGHT: enabled = %d, gain = %d, rate = %dHz",
            dev.config.V.rightEnable, dev.config.V.rightGain ? 256 : 128, dev.config.V.rightRate ? 40 : 10);
    console.printf("\r\nRead mode = %d, calibration count = %d", hx71x.getReadMode(), count);
    if (count > 0 ) {
      for(uint8_t i=0; i<count; i++) {
        CalibrationEntry entry = hx71x.getCalibrationPoint(i);
        showLinefeed();
        if ( hx71x.isCellEnabled(LEFT) ) {
          console.printf("\r\nLEFT[%d]: value = %d, scale = %f, weight = %f", i, entry.Left.value, entry.Left.scale, entry.Left.weight);
        }
        if (hx71x.isCellEnabled(RIGHT)) {
          console.printf("\r\nRIGHT[%d]: value = %d, scale = %f, weight = %f", i, entry.Right.value, entry.Right.scale,
              entry.Right.weight);
        }
      }
    }
    showLinefeed();
    showPrompt();
  } else if ( !strcmp(token, "v") ) {
    showLinefeed();
    showVersion();
    showLinefeed();
    showPrompt();
  } else if ( !strcmp(token, "w") ) {
    // char wbuf[64];
    // fftoa(SCALES_VALUE, wbuf, 2);
    console.printf("\r\nLeft value = %ld, Right value = %ld", LEFT_VALUE, RIGHT_VALUE);
    // console.print(dataBuf);
    // sprintf(dataBuf, "\r\nWeight = %.2f", WEIGHT);
    console.printf("\r\nWeight = %.2f", WEIGHT);
    showPrompt();
  }

  else if ( !strcmp(token, "count") ) {
    // calibration count
    char *token2 = strtok(NULL, delim);
    if (token2 != NULL) {
      unsigned int count = atoi(token2);
      if ( count < 0 || count > 4) {
        count = 0;
      }
      console.printf("\r\nSetting calibration count = %d", count);
      CommandMessage cmd;
      cmd.command = CAL_COUNT;
      cmd.arg.count = count;
      osStatus_t status = osMessageQueuePut(commandQueue, &cmd, 0U, 100);
      if (status == osOK) {
        uint32_t flag = osEventFlagsWait(eventFlagsId, EVENT_CAL_COUNT_READY, osFlagsWaitAny, 2000);
        if ( (flag & EVENT_CAL_COUNT_READY) != 0 ) {
          console.printf("\r\nCalibration count = %d", hx71x.getCalibrationCount());
        }
      }
    } else {
      console.printf("\r\nCalibration count = %d", hx71x.getCalibrationCount());
    }
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

  else if ( !strcmp(token, "cl") || !strcmp(token, "cr")) {
    // syntax: cl <offset> <weight> <times>
    // e.g. cl 0 20.4 3
    // times is optional, default to 1, if weight is missing display current calibration at offset
    uint8_t isLeft = !strcmp(token, "cl");
    uint8_t show = 0;
    unsigned int offset = 0;
    char* token2 = strtok(NULL, delim);
    if ( token2 != NULL ) {
      offset = atoi(token2);
      if ( offset < 0 ) {
        offset = 0;
      } else if ( offset > 3 ) {
        offset = 3;
      }
      float weight = 0.0;
      char *token3 = strtok(NULL, delim);
      if (token3 != NULL) {
        // n = sscanf(token3, "%f", &weight);
        // weight = (float)atof(token3);
        weight = (float)atof(token3);
        char *token4 = strtok(NULL, delim);
        int times = 1;
        if ( token4 != NULL) {
          times = atoi(token4);
        }
        if ( times < 0 || times > 16) {
          times = 3;
        }
        CommandMessage cmd;
        cmd.command = CAL_LEFT;
        cmd.arg.cal.count = times;
        cmd.arg.cal.index = (uint8_t)offset;
        cmd.arg.cal.weight = weight;
        cmd.arg.cal.isLeft = 1;
        uint32_t flag = EVENT_CAL_LEFT_READY;
        if ( isLeft ) {
          console.printf("\r\nCalibrating LEFT cell, offset=%d weight=%.2f", offset, weight);
        } else {
          console.printf("\r\nCalibrating RIGHT cell, offset=%d weight=%.2f", offset, weight);
          cmd.command = CAL_RIGHT;
          cmd.arg.cal.isLeft = 0;
          flag = EVENT_CAL_RIGHT_READY;
        }
        osStatus_t status = osMessageQueuePut(commandQueue, &cmd, 0U, 100);
        if (status == osOK) {
          uint32_t rflag = osEventFlagsWait(eventFlagsId, flag, osFlagsWaitAny, 2000);
          if ( (rflag & flag) != 0 ) {
            // console.printf("\r\nTARE: left=%ld, right=%ld\r\n", hx71x.getTareOffset(LEFT), hx71x.getTareOffset(RIGHT));
            show = 1;
          }
        }
      } else {
        show = 1;
      }

    } else {
      show = 1;
    }
    if ( show ) {
      if ( offset < 0 || offset > 3) {
        offset = 0;
      }

      CalibrationEntry cal = hx71x.getCalibrationPoint((uint8_t) offset);
      int32_t val = isLeft ? cal.Left.value : cal.Right.value;
      float scl = isLeft ? cal.Left.scale : cal.Right.scale;
      float wgt = isLeft ? cal.Left.weight : cal.Right.weight;
      console.printf("\r\nCalibration: %s index=%d value=%ld scale=%f weight=%.2f", isLeft ? "LEFT" : "RIGHT", offset,
          val, scl, wgt);
    }
    showPrompt();
  }


  else if (!strcmp(token, "t") || !strcmp(token, "tare")) {
    char *token2 = strtok(NULL, delim);
    int times = 0;
    if (token2 == NULL) {
      times = 1;
    } else {
      times = atoi(token2);
      if (times > 0) {
        if (times > 15) {
          times = 15;
        }
      }
    }
    if (times > 0) {
      CommandMessage cmd;
      cmd.command = TARE;
      cmd.arg.tare.times = times;
      osStatus_t status = osMessageQueuePut(commandQueue, &cmd, 0U, 100);
      if (status == osOK) {
        uint32_t flag = osEventFlagsWait(eventFlagsId, EVENT_TARE_READY, osFlagsWaitAny, 2000);
        if ( (flag & EVENT_TARE_READY) != 0 ) {
          console.printf("\r\nTARE: left=%ld, right=%ld\r\n", hx71x.getTareOffset(LEFT), hx71x.getTareOffset(RIGHT));
        }
      }
    }
    showPrompt();
  }
  else {
    // sendText((uint8_t*) cmdText, 5);
    // sendText(cmdBuf, strlen((const char*) cmdBuf));
    showPrompt();
  }

}


// process char from console, this is running within the ISR of the UART/DMA
void SerialConsole::rxCharCpltCallback(uint8_t ch) {
  txBuf.size = 1;
  txBuf.text[0] = ch;
  uint32_t parsedCmd = 0;
  if (ch == '\b') { // handle BS
    if (textIndex > 0) {
      textIndex--;
      // send a space to erase current char
      // some ANSI sequence instead?, but this is more portable
      txBuf.text[1] = ' ';
      txBuf.text[2] = '\b';
      txBuf.size = 3;
    } else {
      txBuf.size = 0;
    }
  } else if (ch == 127) { // handle DEL
    // ignore DEL chars, but send a space to erase current char
    txBuf.text[0] = ' ';
    txBuf.text[1] = '\b';
    txBuf.size = 2;
  } else {
    textBuf[textIndex] = ch;
  }

  if (ch == '\r' || ch == '\n') { // check both
    // copy line to command buffer
    // and keep receiving in textBuff
    if (textIndex > 0 ) {
      if (cmdBuf != nullptr) {
        memcpy(cmdBuf, textBuf, textIndex);
        cmdBuf[textIndex] = 0;
        parsedCmd = 1; // CMD_READY_MASK;
      }
      // reset buffer
      textIndex = 0;
    }
  } else if (ch != 127 && ch != '\b') {
    textIndex++;
  }
  // echo
  if (txBuf.size > 0) {
    // only 0 timeout allowed within ISR, so we may loose an echo,
    // unlikely, it's a human typing!
    send(&txBuf, 0);
  }
  if (textIndex >= CMD_LINE_MAX_LEN) {
    textIndex = 0; // wrap around
  }

  // CMD_FLAG = parsedCmd;
  if ( parsedCmd != 0 ) {
    // notify calling thread
    osThreadFlagsSet(myTaskId, LINE_WAIT_FLAG);
  }
}

// Note that the UART will keep receiving chars after this call
void SerialConsole::readLine(char *line) {
  myTaskId = osThreadGetId();
  cmdBuf = line;
  if ( !receiving ) {
    startCharReceive();
  }
  // wait until text line is read
  osThreadFlagsWait(LINE_WAIT_FLAG, osFlagsWaitAny, osWaitForever);
  cmdBuf = nullptr;
}

// TODO: move to its own file
extern "C" void StartCommTask(void *argument)
{
  for(;;)
  {
    osDelay(1);
  }
}
