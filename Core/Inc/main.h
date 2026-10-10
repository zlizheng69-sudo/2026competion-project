/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32h7xx_hal.h"

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
#define X1_Pin GPIO_PIN_0
#define X1_GPIO_Port GPIOC
#define X2_Pin GPIO_PIN_1
#define X2_GPIO_Port GPIOC
#define X3_Pin GPIO_PIN_2
#define X3_GPIO_Port GPIOC
#define X4_Pin GPIO_PIN_3
#define X4_GPIO_Port GPIOC
#define serialblue_TX_Pin GPIO_PIN_0
#define serialblue_TX_GPIO_Port GPIOA
#define serialblue_RX_Pin GPIO_PIN_1
#define serialblue_RX_GPIO_Port GPIOA
#define encoderA1_Pin GPIO_PIN_5
#define encoderA1_GPIO_Port GPIOA
#define encoderA2_Pin GPIO_PIN_6
#define encoderA2_GPIO_Port GPIOA
#define encoderB2_Pin GPIO_PIN_7
#define encoderB2_GPIO_Port GPIOA
#define BIN1_Pin GPIO_PIN_0
#define BIN1_GPIO_Port GPIOB
#define BIN2_Pin GPIO_PIN_1
#define BIN2_GPIO_Port GPIOB
#define OLED_SCL_Pin GPIO_PIN_10
#define OLED_SCL_GPIO_Port GPIOB
#define OLED_SDA_Pin GPIO_PIN_11
#define OLED_SDA_GPIO_Port GPIOB
#define yaw_TX_Pin GPIO_PIN_14
#define yaw_TX_GPIO_Port GPIOB
#define yaw_RX_Pin GPIO_PIN_15
#define yaw_RX_GPIO_Port GPIOB
#define camera_TX_Pin GPIO_PIN_8
#define camera_TX_GPIO_Port GPIOD
#define camera_RX_Pin GPIO_PIN_9
#define camera_RX_GPIO_Port GPIOD
#define STBY_Pin GPIO_PIN_14
#define STBY_GPIO_Port GPIOD
#define AIN2_Pin GPIO_PIN_6
#define AIN2_GPIO_Port GPIOC
#define AIN1_Pin GPIO_PIN_7
#define AIN1_GPIO_Port GPIOC
#define mid_Pin GPIO_PIN_8
#define mid_GPIO_Port GPIOC
#define set_Pin GPIO_PIN_9
#define set_GPIO_Port GPIOC
#define down_Pin GPIO_PIN_8
#define down_GPIO_Port GPIOA
#define up_Pin GPIO_PIN_9
#define up_GPIO_Port GPIOA
#define encouderB1_Pin GPIO_PIN_3
#define encouderB1_GPIO_Port GPIOB
#define IMU_SCL_Pin GPIO_PIN_6
#define IMU_SCL_GPIO_Port GPIOB
#define IMU_SDA_Pin GPIO_PIN_7
#define IMU_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
