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
#define ADC1_Pin GPIO_PIN_1
#define ADC1_GPIO_Port GPIOA
#define ADC2_Pin GPIO_PIN_2
#define ADC2_GPIO_Port GPIOA
#define ADC3_Pin GPIO_PIN_3
#define ADC3_GPIO_Port GPIOA
#define ADC4_Pin GPIO_PIN_4
#define ADC4_GPIO_Port GPIOA
#define START_Pin GPIO_PIN_13
#define START_GPIO_Port GPIOB
#define STOP_Pin GPIO_PIN_14
#define STOP_GPIO_Port GPIOB
#define SWITCH_Pin GPIO_PIN_5
#define SWITCH_GPIO_Port GPIOB
#define SWITCH_EXTI_IRQn EXTI9_5_IRQn
#define PELTIER_Pin GPIO_PIN_6
#define PELTIER_GPIO_Port GPIOB
#define SAMPLING_Pin GPIO_PIN_7
#define SAMPLING_GPIO_Port GPIOB
#define TG2_Pin GPIO_PIN_8
#define TG2_GPIO_Port GPIOB
#define TG2_EXTI_IRQn EXTI9_5_IRQn
#define TG1_Pin GPIO_PIN_9
#define TG1_GPIO_Port GPIOB
#define TG1_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */
#define USB_BUF_SIZE	64
#define ADC_CHANNELS 	4

#define START_TOG	HAL_GPIO_TogglePin(START_GPIO_Port, START_Pin);
#define START_ON	HAL_GPIO_WritePin(START_GPIO_Port, START_Pin, ENABLE);
#define START_OFF	HAL_GPIO_WritePin(START_GPIO_Port, START_Pin, DISABLE);

#define STOP_TOG	HAL_GPIO_TogglePin(STOP_GPIO_Port, STOP_Pin);
#define STOP_ON		HAL_GPIO_WritePin(STOP_GPIO_Port, STOP_Pin, ENABLE);
#define STOP_OFF	HAL_GPIO_WritePin(STOP_GPIO_Port, STOP_Pin, DISABLE);



/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
