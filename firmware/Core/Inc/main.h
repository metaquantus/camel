/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CAMEL_ADDRESS 0x75
#define LED1_Pin GPIO_PIN_14
#define LED1_GPIO_Port GPIOC
#define SCK1_Pin GPIO_PIN_5
#define SCK1_GPIO_Port GPIOA
#define DOUT1_Pin GPIO_PIN_6
#define DOUT1_GPIO_Port GPIOA
#define DOUT2_Pin GPIO_PIN_7
#define DOUT2_GPIO_Port GPIOA
#define SCK2_Pin GPIO_PIN_1
#define SCK2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#ifdef CAMEL_UART
#define UART_TX_DATA_SIZE 16
#endif
// HX71x has 24bit resolution, times 2 cells
// first 3 bytes for leftCell, last 3 bytes for rightCell
#define SCALES_DATA_SIZE 6
#define CONFIG_MASK 0x80
#define LEFT_MASK   0x40
#define RIGHT_MASK  0x20
#define COUNT_MASK  0x10


#ifdef CAMEL1
#define DEFAULT_CONFIG 0x33
#define LEFT_SCK_Pin        SCK2_Pin
#define LEFT_SCK_GPIO_Port  SCK2_GPIO_Port
#define LEFT_DOUT_Pin       DOUT2_Pin
#define LEFT_DOUT_GPIO_Port DOUT2_GPIO_Port

#define RIGHT_DOUT_Pin       DOUT1_Pin
#define RIGHT_DOUT_GPIO_Port DOUT1_GPIO_Port
#define RIGHT_SCK_Pin        SCK1_Pin
#define RIGHT_SCK_GPIO_Port  SCK1_GPIO_Port

#else
#define DEFAULT_CONFIG 0x11
// cells are reversed in PCB rev 2 to optimise layout
#define LEFT_SCK_Pin        SCK1_Pin
#define LEFT_SCK_GPIO_Port  SCK1_GPIO_Port
#define LEFT_DOUT_Pin       DOUT1_Pin
#define LEFT_DOUT_GPIO_Port DOUT1_GPIO_Port

#define RIGHT_DOUT_Pin       DOUT2_Pin
#define RIGHT_DOUT_GPIO_Port DOUT2_GPIO_Port
#define RIGHT_SCK_Pin        SCK2_Pin
#define RIGHT_SCK_GPIO_Port  SCK2_GPIO_Port

#endif

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
