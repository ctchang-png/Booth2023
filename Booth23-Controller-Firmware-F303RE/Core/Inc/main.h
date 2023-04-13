/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32f3xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define EN21_Pin GPIO_PIN_0
#define EN21_GPIO_Port GPIOA
#define EN22_Pin GPIO_PIN_1
#define EN22_GPIO_Port GPIOA
#define DIR31_Pin GPIO_PIN_4
#define DIR31_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define EN31_Pin GPIO_PIN_6
#define EN31_GPIO_Port GPIOA
#define EN32_Pin GPIO_PIN_7
#define EN32_GPIO_Port GPIOA
#define DIR32_Pin GPIO_PIN_0
#define DIR32_GPIO_Port GPIOB
#define DIR12_Pin GPIO_PIN_10
#define DIR12_GPIO_Port GPIOB
#define EN11_Pin GPIO_PIN_8
#define EN11_GPIO_Port GPIOA
#define EN12_Pin GPIO_PIN_9
#define EN12_GPIO_Port GPIOA
#define DIR11_Pin GPIO_PIN_10
#define DIR11_GPIO_Port GPIOA
#define DIR22_Pin GPIO_PIN_4
#define DIR22_GPIO_Port GPIOB
#define DIR21_Pin GPIO_PIN_5
#define DIR21_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
