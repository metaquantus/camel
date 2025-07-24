/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
// #include "camel_i2c.h"
#include "hx71x.h"
#include "eeprom.h"
#include "utils.h"
#include "camel_uart.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SCAN_FREQ 1
#define LED_PERIOD (800 / SCAN_FREQ)
// wait 2 seconds before reporting mode
#define LED_WAIT (2000 / SCAN_FREQ)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
HX71x_TypeDef leftCell = { LEFT_DOUT_GPIO_Port, LEFT_DOUT_Pin, LEFT_SCK_GPIO_Port, LEFT_SCK_Pin, 1, 1, 0 };
HX71x_TypeDef rightCell= { RIGHT_DOUT_GPIO_Port, RIGHT_DOUT_Pin, RIGHT_SCK_GPIO_Port, RIGHT_SCK_Pin, 1, 1, 0 };
uint8_t SCALES_DATA[SCALES_DATA_SIZE] = { 0, 0, 0, 0, 0, 0 };
// encoded left/right cell configuration (sampling period and gain factor)
uint8_t SCALES_CONFIG = DEFAULT_CONFIG;
// number of calibration points
uint8_t CAL_COUNT = 0;
// next calibration point offset to read or write
uint8_t CAL_OFFSET = 0;
// flag indicating next function to perform (from i2c or ADCs)
uint8_t FUNC_FLAG = 0;
// flag indicating next command to perform (from UART)
uint32_t CMD_FLAG = 0;
extern uint8_t CAL_DATA[CAMEL_CAL_DATA_SIZE];
// calibrated scaled value
float SCALES_VALUE = 0.0;
// tare offsets
long LEFT_TARE_OFFSET = 0;
long RIGHT_TARE_OFFSET = 0;
// 0: left/right raw values, 1: calibrated scaled value, 2: left/right raw values and calibrated value
uint8_t READ_VALUE_MODE = 0;
// number of times to repeat the tare reads and average
// tare function activated when TARE_TIMES > 0
uint8_t TARE_TIMES = 0;
// current tare read index
uint8_t TARE_INDEX = 0;
// number of times to repeat the left calibration reads and average
uint8_t LEFT_CAL_TIMES = 0;
// current left calibration read index
uint8_t LEFT_CAL_INDEX = 0;
// the new left calibration value
float LEFT_CAL_VALUE = 0;
// number of times to repeat the right calibration reads and average
uint8_t RIGHT_CAL_TIMES = 0;
// current right calibration read index
uint8_t RIGHT_CAL_INDEX = 0;
// the new right calibration value to store
float RIGHT_CAL_VALUE = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CRC_Init(void);
/* USER CODE BEGIN PFP */
void readConfig();
void parseConfig();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  uint16_t ledWaitCount = 0;
  uint16_t ledCount = 0;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */
  uartInit();
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
  readConfig();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if ((FUNC_FLAG & (CONFIG_MASK | LEFT_MASK | RIGHT_MASK | COUNT_MASK)) != 0) {
      parseConfig();
      // report change
      ledWaitCount = 0;
      ledCount = 0;
    }
    if (leftCell.modified) {
      if (leftCell.enabled) {
        HX71x_powerUp(&leftCell);
      } else {
        HX71x_powerDown(&leftCell);
      }
      leftCell.modified = 0;
    }
    if (rightCell.modified) {
      if (rightCell.enabled) {
        HX71x_powerUp(&rightCell);
      } else {
        HX71x_powerDown(&rightCell);
      }
      rightCell.modified = 0;
    }
    if (ledWaitCount == 0) {
      HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
    }
    // if both cells are enabled, wait for both to be ready
    // and read them together, so next time they we'll be ready
    // about the same time
    uint8_t leftReady = leftCell.enabled && HX71x_isReady(&leftCell);
    uint8_t rightReady = rightCell.enabled && HX71x_isReady(&rightCell);

    if ((leftCell.enabled && rightCell.enabled && leftReady && rightReady) || (leftCell.enabled && !rightCell.enabled && leftReady)
        || (!leftCell.enabled && rightCell.enabled && rightReady)) {
      HX71x_read(&leftCell, &rightCell);
    }
    // LED:
    // wait 2 seconds before lighting the LED to report config status
    // to give host time to initialize this board.
    // 1 pulse for leftCell enabled only,
    // 2 pulses for rightCell enabled only,
    // 3 pulses for both leftCell and rightCells enabled
    // We may get 3 pulses for the default configuration
    // if host didn't configured us quick enough.
    if (ledWaitCount < LED_WAIT) {
      ledWaitCount++;
    } else {

      if (leftCell.enabled || rightCell.enabled) {
        if (ledCount == 0) {
          HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
        } else if (ledCount == LED_PERIOD) {
          HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
        }
      }
      if (rightCell.enabled) {
        if (ledCount == LED_PERIOD * 2) {
          HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
        } else if (ledCount == LED_PERIOD * 3) {
          HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
        }
      }
      if (leftCell.enabled && rightCell.enabled) {
        if (ledCount == LED_PERIOD * 4) {
          HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
        } else if (ledCount == LED_PERIOD * 5) {
          HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
        }
      }

      if (ledCount < LED_PERIOD * 5) {
        ledCount++;
      }
    }
#ifdef CAMEL_UART
    if ( CMD_FLAG != 0 ) {
      processCmd();
    }
#endif
    // HX71x are setup at 10 samples per second (every 100ms)
    // conversion starts at end of the read,
    // we don't want to poll too often but enough to detect
    // conversion ready without loosing too much time
    // TODO: use external interrupt from DOUT lines to detect ready state.
    HAL_Delay(SCAN_FREQ);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_DISABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_DISABLE;
  hcrc.Init.GeneratingPolynomial = 151;
  hcrc.Init.CRCLength = CRC_POLYLENGTH_8B;
  hcrc.Init.InitValue = 0xFF;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00B07CB4;
  hi2c1.Init.OwnAddress1 = 234;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */
  // start listening to the host
  if (HAL_I2C_EnableListen_IT(&hi2c1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_5_6_7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_5_6_7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_5_6_7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SCK1_GPIO_Port, SCK1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SCK2_GPIO_Port, SCK2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED1_Pin */
  GPIO_InitStruct.Pin = LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SCK1_Pin */
  GPIO_InitStruct.Pin = SCK1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SCK1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DOUT1_Pin DOUT2_Pin */
  GPIO_InitStruct.Pin = DOUT1_Pin|DOUT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SCK2_Pin */
  GPIO_InitStruct.Pin = SCK2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SCK2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/*
 * Read configuration and calibration data from EEPROM
 */
void readConfig(void) {
  // 1st byte stores cells configuration, 2nd byte is calibration data point count sent by host
  // we should keep the count and send it back when requested.
  // we store also a CRC to validate EEPROM DATA
  uint8_t config[4] = {0 , 0, 0, 0};
  SCALES_CONFIG = eeprom_read_byte(CAMEL_CONFIG_EEPROM_START);
  CAL_COUNT = eeprom_read_byte(CAMEL_CONFIG_EEPROM_START+1);
  READ_VALUE_MODE = eeprom_read_byte(CAMEL_CONFIG_EEPROM_START+2);
  uint8_t scrc = eeprom_read_byte(CAMEL_CONFIG_EEPROM_START+3);
  config[0] = SCALES_CONFIG;
  config[1] = CAL_COUNT;
  config[2] = READ_VALUE_MODE;

  uint8_t crc = GENCRC(config, 3);
  if ( crc != scrc ) {
    // wrong CRC
    // load default value
    SCALES_CONFIG = DEFAULT_CONFIG;
    CAL_COUNT = 0;
    READ_VALUE_MODE = 0;
  }
  if ( CAL_COUNT > CAMEL_CAL_DATA_COUNT ) {
    CAL_COUNT = CAMEL_CAL_DATA_COUNT;
  }
  if ( READ_VALUE_MODE > 2 ) {
    READ_VALUE_MODE = 0;
  }
  TARE_TIMES = 0;
  LEFT_CAL_TIMES = 0;
  RIGHT_CAL_TIMES = 0;
  LEFT_TARE_OFFSET = 0;
  RIGHT_TARE_OFFSET = 0;

  FUNC_FLAG = CONFIG_MASK;


  // EEPROM_DATA is a shadow in SRAM of EEPROM calibration data as EEPROM NVM is slower to read and write
  // no check is done on data, the host should do the checking as format is host specific
  // memcpy(EEPROM_DATA, (const void*) (DATA_EEPROM_BASE + CAMEL_EEPROM_START), EEPROM_DATA_SIZE);
  // copy only valid data
  if ( CAL_COUNT > 0 && CAL_COUNT < CAMEL_CAL_DATA_COUNT ) {
    for(size_t i=0; i<CAL_COUNT; i++) {
      uint32_t offset = i << 3; // i * 8
      uint32_t value = eeprom_read_word(CAMEL_LEFT_EEPROM_START + offset);
      // memcpy(CAL_DATA + offset, &value, 4);
      * (uint32_t *) (CAL_DATA + offset) = value;
      value = eeprom_read_word(CAMEL_LEFT_EEPROM_START + offset + 4);
      // memcpy(CAL_DATA + offset + 4, &value, 4);
      * (uint32_t *)(CAL_DATA + offset + 4) = value;
      value = eeprom_read_word(CAMEL_RIGHT_EEPROM_START + offset);
      // memcpy(CAL_DATA + CAMEL_CAL_RIGHT_OFFSET + offset, &value, 4);
      * (uint32_t *)(CAL_DATA + CAMEL_CAL_RIGHT_OFFSET + offset) = value;
      value = eeprom_read_word(CAMEL_RIGHT_EEPROM_START + offset + 4);
      // memcpy(CAL_DATA + CAMEL_CAL_RIGHT_OFFSET + offset + 4, &value, 4);
      * (uint32_t *)(CAL_DATA + CAMEL_CAL_RIGHT_OFFSET + offset + 4) = value;
    }
  }
}

/*
   Parse the configuration data sent from the host and
   set the parameters for next conversion or power down an HX711.
   Configuration command consists of two nibbles, the most significant for the left cell,
   and the least significant for the right cell:
     00110011
   bit 0 of each nibble sets whether to enable/disable the corresponding cell
         where 0 means to power down the device, e.g. put it in sleep mode
   bit 1 sets the gain factor: 0 for 64 or 1 for 128 for HX711 (CAMEL1); 0 for 128 or 1 for 256 for HX712 (CAMEL2)
   bit 2 sets sampling rate for HX712: 0 for 10Hz or 1 for 40Hz
   e.g: 0x33 enables both cells at 128 gain factor for HX711 or 256 gain factor at 10Hz for HX712

   If the enable bit is changed from previous value, then the modified state is also set to let the main
   loop know and do the job.
 */
void parseConfig(void) {
  if ((FUNC_FLAG & LEFT_MASK) != 0) {
    eeprom_write_word(CAMEL_LEFT_EEPROM_START + CAL_OFFSET,
        (uint32_t) *(uint32_t*) (CAL_DATA + CAL_OFFSET));
    eeprom_write_word(CAMEL_LEFT_EEPROM_START + CAL_OFFSET + 4,
        (uint32_t) *(uint32_t*) (CAL_DATA + CAL_OFFSET + 4));
    // SCALES_CONFIG &= 0xFFFF;
  } else if ((FUNC_FLAG & RIGHT_MASK) != 0) {
    eeprom_write_word(CAMEL_RIGHT_EEPROM_START + CAL_OFFSET,
        (uint32_t) *(uint32_t*) (CAL_DATA + CAMEL_CAL_RIGHT_OFFSET
            + CAL_OFFSET));
    eeprom_write_word(CAMEL_RIGHT_EEPROM_START + CAL_OFFSET + 4,
        (uint32_t) *(uint32_t*) (CAL_DATA + CAMEL_CAL_RIGHT_OFFSET
            + CAL_OFFSET + 4));
    // SCALES_CONFIG &= 0xFFFF;
  } else if ((FUNC_FLAG & COUNT_MASK) != 0) {
    uint8_t config[4] = { 0, 0, 0, 0 };
    config[0] = SCALES_CONFIG;
    config[1] = CAL_COUNT;
    config[2] = READ_VALUE_MODE;
    uint8_t crc = GENCRC(config, 3);
    eeprom_write_byte(CAMEL_CONFIG_EEPROM_START, SCALES_CONFIG);
    eeprom_write_byte(CAMEL_CONFIG_EEPROM_START + 1, CAL_COUNT);
    eeprom_write_byte(CAMEL_CONFIG_EEPROM_START + 2, READ_VALUE_MODE);
    eeprom_write_byte(CAMEL_CONFIG_EEPROM_START + 3, crc);
    // SCALES_CONFIG &= 0xFFFF;

  }
  if ((FUNC_FLAG & CONFIG_MASK) != 0) {

#ifdef CAMEL1
    // HX711
    // left cell gain
    if ((SCALES_CONFIG & 0x20) != 0) {
      leftCell.GAIN = 1; // 128, channel A
    } else {
      leftCell.GAIN = 3; // 64, channel A
    }
    // right cell gain
    if ((SCALES_CONFIG & 0x02) != 0) {
      rightCell.GAIN = 1; // 128, channel A
    } else {
      rightCell.GAIN = 3; // 64, channel A
    }
#else
  // HX712 CAMEL2
  uint8_t c = (SCALES_CONFIG >> 5) & 0x03;
  switch (c) {
  case 0:
    leftCell.GAIN = 1; // 128/10Hz
    break;
  case 1:
    leftCell.GAIN = 4; // 256/10Hz
    break;
  case 2:
    leftCell.GAIN = 3; // 128/40Hz
    break;
  case 3:
    leftCell.GAIN = 5; // 256/40Hz
    break;
  }
  c = (SCALES_CONFIG >> 1) & 0x03;
  switch (c) {
    case 0:
      rightCell.GAIN = 1; // 128/10Hz
      break;
    case 1:
      rightCell.GAIN = 4; // 256/10Hz
      break;
    case 2:
      rightCell.GAIN = 3; // 128/40Hz
      break;
    case 3:
      rightCell.GAIN = 5; // 256/40Hz
      break;
    }

#endif
    // left cell enable
    if ((SCALES_CONFIG & 0x10) != 0) {
      if (!leftCell.enabled) {
        leftCell.enabled = 1;
        leftCell.modified = 1;
      }
    } else {
      if (leftCell.enabled) {
        leftCell.enabled = 0;
        leftCell.modified = 1;
      }
    }
    // right cell enable
    if ((SCALES_CONFIG & 0x01) != 0) {
      if (!rightCell.enabled) {
        rightCell.enabled = 1;
        rightCell.modified = 1;
      }
    } else {
      if (rightCell.enabled) {
        rightCell.enabled = 0;
        rightCell.modified = 1;
      }
    }
  }
  // nothing to do for TARE_MASK

  FUNC_FLAG = 0;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
    HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
    HAL_Delay(700);
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
