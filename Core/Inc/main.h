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
#include "stm32c0xx_hal.h"

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
#define TEMP_Pin GPIO_PIN_1
#define TEMP_GPIO_Port GPIOA
#define I_SENS_FILT_Pin GPIO_PIN_2
#define I_SENS_FILT_GPIO_Port GPIOA
#define RELAY_SET_Pin GPIO_PIN_3
#define RELAY_SET_GPIO_Port GPIOA
#define RELAY_RESET_Pin GPIO_PIN_4
#define RELAY_RESET_GPIO_Port GPIOA
#define FAN_CTRL_Pin GPIO_PIN_5
#define FAN_CTRL_GPIO_Port GPIOA
#define PWM_CTRL_Pin GPIO_PIN_6
#define PWM_CTRL_GPIO_Port GPIOA
#define PID_COOLING_Pin GPIO_PIN_7
#define PID_COOLING_GPIO_Port GPIOA
#define SEL1_Pin GPIO_PIN_1
#define SEL1_GPIO_Port GPIOB
#define SEL2_Pin GPIO_PIN_2
#define SEL2_GPIO_Port GPIOB
#define SEL3_Pin GPIO_PIN_8
#define SEL3_GPIO_Port GPIOA
#define SEL3B3_Pin GPIO_PIN_3
#define SEL3B3_GPIO_Port GPIOB
#define SEL4_Pin GPIO_PIN_4
#define SEL4_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
