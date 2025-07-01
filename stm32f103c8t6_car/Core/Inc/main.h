/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define TB1_EN_Pin GPIO_PIN_14
#define TB1_EN_GPIO_Port GPIOC
#define TB2_EN_Pin GPIO_PIN_15
#define TB2_EN_GPIO_Port GPIOC
#define M2_A_Pin GPIO_PIN_0
#define M2_A_GPIO_Port GPIOA
#define M2_B_Pin GPIO_PIN_1
#define M2_B_GPIO_Port GPIOA
#define ADC1_Pin GPIO_PIN_4
#define ADC1_GPIO_Port GPIOA
#define ADC2_Pin GPIO_PIN_5
#define ADC2_GPIO_Port GPIOA
#define PPM_Pin GPIO_PIN_6
#define PPM_GPIO_Port GPIOA
#define PWM_Pin GPIO_PIN_7
#define PWM_GPIO_Port GPIOA
#define PM1_Pin GPIO_PIN_0
#define PM1_GPIO_Port GPIOB
#define PM2_Pin GPIO_PIN_1
#define PM2_GPIO_Port GPIOB
#define OUT_Pin GPIO_PIN_2
#define OUT_GPIO_Port GPIOB
#define M1_1_Pin GPIO_PIN_12
#define M1_1_GPIO_Port GPIOB
#define M1_2_Pin GPIO_PIN_13
#define M1_2_GPIO_Port GPIOB
#define M2_1_Pin GPIO_PIN_14
#define M2_1_GPIO_Port GPIOB
#define M2_2_Pin GPIO_PIN_15
#define M2_2_GPIO_Port GPIOB
#define M1_A_Pin GPIO_PIN_8
#define M1_A_GPIO_Port GPIOA
#define M1_B_Pin GPIO_PIN_9
#define M1_B_GPIO_Port GPIOA
#define IN_Pin GPIO_PIN_10
#define IN_GPIO_Port GPIOA
#define M3_1_Pin GPIO_PIN_15
#define M3_1_GPIO_Port GPIOA
#define M3_2_Pin GPIO_PIN_3
#define M3_2_GPIO_Port GPIOB
#define M4_1_Pin GPIO_PIN_4
#define M4_1_GPIO_Port GPIOB
#define M4_2_Pin GPIO_PIN_5
#define M4_2_GPIO_Port GPIOB
#define PM3_Pin GPIO_PIN_8
#define PM3_GPIO_Port GPIOB
#define PM4_Pin GPIO_PIN_9
#define PM4_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
