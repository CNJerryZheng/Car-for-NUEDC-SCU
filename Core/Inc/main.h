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
#define L_Encode_A_Pin GPIO_PIN_0
#define L_Encode_A_GPIO_Port GPIOA
#define L_Encode_B_Pin GPIO_PIN_1
#define L_Encode_B_GPIO_Port GPIOA
#define AIN1_Pin GPIO_PIN_2
#define AIN1_GPIO_Port GPIOA
#define AIN2_Pin GPIO_PIN_3
#define AIN2_GPIO_Port GPIOA
#define BIN1_Pin GPIO_PIN_4
#define BIN1_GPIO_Port GPIOA
#define BIN2_Pin GPIO_PIN_5
#define BIN2_GPIO_Port GPIOA
#define LED_OUTSIDE_Pin GPIO_PIN_6
#define LED_OUTSIDE_GPIO_Port GPIOA
#define BEEP_Pin GPIO_PIN_0
#define BEEP_GPIO_Port GPIOB
#define Trig_Pin GPIO_PIN_1
#define Trig_GPIO_Port GPIOB
#define L2_Pin GPIO_PIN_10
#define L2_GPIO_Port GPIOB
#define L1_Pin GPIO_PIN_11
#define L1_GPIO_Port GPIOB
#define M_Pin GPIO_PIN_12
#define M_GPIO_Port GPIOB
#define R1_Pin GPIO_PIN_13
#define R1_GPIO_Port GPIOB
#define R2_Pin GPIO_PIN_14
#define R2_GPIO_Port GPIOB
#define Echo_Pin GPIO_PIN_15
#define Echo_GPIO_Port GPIOB
#define CIN1_Pin GPIO_PIN_8
#define CIN1_GPIO_Port GPIOA
#define MVTX_Pin GPIO_PIN_9
#define MVTX_GPIO_Port GPIOA
#define MVRX_Pin GPIO_PIN_10
#define MVRX_GPIO_Port GPIOA
#define CIN2_Pin GPIO_PIN_11
#define CIN2_GPIO_Port GPIOA
#define DIN1_Pin GPIO_PIN_12
#define DIN1_GPIO_Port GPIOA
#define DIN2_Pin GPIO_PIN_15
#define DIN2_GPIO_Port GPIOA
#define START_KEY_Pin GPIO_PIN_3
#define START_KEY_GPIO_Port GPIOB
#define R_Encode_A_Pin GPIO_PIN_4
#define R_Encode_A_GPIO_Port GPIOB
#define R_Encode_B_Pin GPIO_PIN_5
#define R_Encode_B_GPIO_Port GPIOB
#define PWMA_Pin GPIO_PIN_6
#define PWMA_GPIO_Port GPIOB
#define PWMB_Pin GPIO_PIN_7
#define PWMB_GPIO_Port GPIOB
#define PWMC_Pin GPIO_PIN_8
#define PWMC_GPIO_Port GPIOB
#define PWMD_Pin GPIO_PIN_9
#define PWMD_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
