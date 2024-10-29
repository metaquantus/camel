/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 MetaQuantus
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  * The Camel board contains dual HX711 ADC converters for load cells in
  * a bridge configuration. The board wires the HX711s to 10 samples per
  * second using the internal oscillator. Only channel A is used in each HX711.
  * The HX711s are identified as "left" and "right" and each can be separately
  * configured to 128 or 64 gain factor or power down.
  * The MCU drives the HX711s, starting the conversions, reading the data
  * continuously and storing it for transmission. It provides I2C slave communications using
  * a simple protocol. The data is simply sent as read, no further conversions
  * or scaling is done.
  *
  * For complete functionality, the host must implement the usual weigh scales
  * functions like scaling to weight units, calibration, tare, etc.
  *
  * This program code manages the HX711s and handles the I2C slave communications.
  *
  * The board also wires up the USART to a connector and it might be possible
  * to provide some functionality like diagnostics. However, this
  * MCU only has 16KB of flash memory, so it's very tight, current code
  * without debugging info takes up 10KB, and adding the USART code may not fit.
  *
  * Total current consumption is about 8mA, and disabling one of the
  * HX711s saves about 1.5mA.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "scales.h"
#include "camel_i2c.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SCAN_FREQ 5
#define LED_PERIOD (800 / SCAN_FREQ)
// wait 2 seconds before reporting mode
#define LED_WAIT (2000 / SCAN_FREQ)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */
// at reset both cells are enabled at 128 gain factor
HX711_TypeDef leftCell = { DOUT1_GPIO_Port, DOUT1_Pin, SCK1_GPIO_Port, SCK1_Pin, 1, 1, 0 };
HX711_TypeDef rightCell= { DOUT2_GPIO_Port, DOUT2_Pin, SCK2_GPIO_Port, SCK2_Pin, 1, 1, 0 };
uint8_t SCALES_DATA[SCALES_DATA_SIZE] = { 0, 0, 0, 0, 0, 0 };
uint16_t SCALES_CONFIG = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if ( (SCALES_CONFIG & 0x8000) != 0) {
      parseConfig();
    }
	  if ( leftCell.modified ) {
		  if ( leftCell.enabled ) {
			  HX711_powerUp(&leftCell);
		  } else {
			  HX711_powerDown(&leftCell);
		  }
		  leftCell.modified = 0;
		  // report change
		  ledWaitCount = 0;
		  ledCount = 0;
	  }
		if (rightCell.modified) {
			if (rightCell.enabled) {
				HX711_powerUp(&rightCell);
			} else {
				HX711_powerDown(&rightCell);
			}
      rightCell.modified = 0;
      // report change
      ledWaitCount = 0;
      ledCount = 0;
		}
		if ( ledWaitCount == 0 ) {
		  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
		}

		// if both cells are enabled, wait for both to be ready
		// and read them together, so next time they we'll be ready
		// about the same time
	  uint8_t leftReady = leftCell.enabled && HX711_isReady(&leftCell) ;
	  uint8_t rightReady = rightCell.enabled && HX711_isReady(&rightCell);

	  if ( ( leftCell.enabled && rightCell.enabled && leftReady && rightReady ) ||
		   ( leftCell.enabled && !rightCell.enabled && leftReady ) ||
		   ( !leftCell.enabled && rightCell.enabled && rightReady ) ) {
		  HX711_read(&leftCell, &rightCell);
	  }
	  // LED:
	  // wait 2 seconds before lighting the LED to report config status
	  // to give host time to initialize this board.
	  // 1 pulse for leftCell enabled only,
	  // 2 pulses for rightCell enabled only,
	  // 3 pulses for both leftCell and rightCells enabled
	  // We may get 3 pulses for the default configuration
	  // if host didn't configured us quick enough.
	  if ( ledWaitCount < LED_WAIT ) {
	    ledWaitCount++;
	  } else {
	    if ( ledCount == 0 ) {
	      HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
	    } else if ( ledCount == LED_PERIOD ) {
	      HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
	    }
	    if ( rightCell.enabled ) {
	      if ( ledCount == LED_PERIOD * 2 ) {
	        HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
	      } else if ( ledCount == LED_PERIOD * 3 ) {
	        HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
	      }
	    }
	    if ( leftCell.enabled && rightCell.enabled ) {
        if (ledCount == LED_PERIOD * 4) {
          HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
        } else if (ledCount == LED_PERIOD * 5) {
          HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
        }
	    }
	    if ( ledCount < LED_PERIOD * 5 ) {
	      ledCount++;
	    }
	  }

	  // HX711 are setup at 10 samples per second (every 100ms)
	  // conversion starts at end of the read,
	  // we don't want to poll too often but enough to detect
	  // conversion ready without loosing too much time
	  // TODO: use external interrupt from DOUT lines to detect ready state.
	  HAL_Delay(SCAN_FREQ);
  }
  /* USER CODE END 3 */
}

/*
   We only support one receive command (master transmit):
     the configuration command.
   It consists of two nibbles, the most significant for the left cell,
   and the least significant for the right cell:
     00110011
   bit 0 of each nibble sets whether to enable/disable the corresponding cell
   bit 1 sets the gain factor: 0 for 64 or 1 for 128
   e.g: 0x33 enables both cells at 128 gain factor
   The other bits are ignored, reserved for future use, set to 0.
   If the enable bit is changed from previous value, then the modified state is also set to let the main
   loop know and do the job.
 */
void parseConfig(void) {
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
  SCALES_CONFIG &= 0x00FF;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  if (HAL_I2C_EnableListen_IT(&hi2c1) != HAL_OK)
  {
	  Error_Handler();
  }
  /* USER CODE END I2C1_Init 2 */

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
  HAL_GPIO_WritePin(SCK2_GPIO_Port, SCK2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SCK1_GPIO_Port, SCK1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED1_Pin */
  GPIO_InitStruct.Pin = LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SCK2_Pin */
  GPIO_InitStruct.Pin = SCK2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SCK2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DOUT2_Pin DOUT1_Pin */
  GPIO_InitStruct.Pin = DOUT2_Pin|DOUT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SCK1_Pin */
  GPIO_InitStruct.Pin = SCK1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SCK1_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
