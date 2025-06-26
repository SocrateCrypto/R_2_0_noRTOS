/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

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
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define LED2_Pin GPIO_PIN_14
#define LED2_GPIO_Port GPIOC
#define ADC_Pin GPIO_PIN_0
#define ADC_GPIO_Port GPIOA
#define LAMP_Pin GPIO_PIN_1
#define LAMP_GPIO_Port GPIOA
#define CALL_Pin GPIO_PIN_4
#define CALL_GPIO_Port GPIOA
#define COMP_Pin GPIO_PIN_5
#define COMP_GPIO_Port GPIOA
#define BIND_Pin GPIO_PIN_6
#define BIND_GPIO_Port GPIOA
#define ADC_RES_Pin GPIO_PIN_7
#define ADC_RES_GPIO_Port GPIOA
#define LEFT_Pin GPIO_PIN_0
#define LEFT_GPIO_Port GPIOB
#define RIGHT_Pin GPIO_PIN_1
#define RIGHT_GPIO_Port GPIOB
#define RS485_DERE_Pin GPIO_PIN_2
#define RS485_DERE_GPIO_Port GPIOB
#define CSN_Pin GPIO_PIN_12
#define CSN_GPIO_Port GPIOB
#define IMU_EN_Pin GPIO_PIN_8
#define IMU_EN_GPIO_Port GPIOA
#define CE_Pin GPIO_PIN_8
#define CE_GPIO_Port GPIOB
#define IRQ_Pin GPIO_PIN_9
#define IRQ_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
