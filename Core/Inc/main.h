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
#define EC_BL_A_Pin GPIO_PIN_0
#define EC_BL_A_GPIO_Port GPIOA
#define EC_BL_B_Pin GPIO_PIN_1
#define EC_BL_B_GPIO_Port GPIOA
#define MP3_TX_Pin GPIO_PIN_2
#define MP3_TX_GPIO_Port GPIOA
#define MP3_RX_Pin GPIO_PIN_3
#define MP3_RX_GPIO_Port GPIOA
#define EC_BR_A_Pin GPIO_PIN_6
#define EC_BR_A_GPIO_Port GPIOA
#define EC_BR_B_Pin GPIO_PIN_7
#define EC_BR_B_GPIO_Port GPIOA
#define DEBUG_TX_Pin GPIO_PIN_10
#define DEBUG_TX_GPIO_Port GPIOB
#define DEBUG_RX_Pin GPIO_PIN_11
#define DEBUG_RX_GPIO_Port GPIOB
#define BLIN_B_Pin GPIO_PIN_12
#define BLIN_B_GPIO_Port GPIOB
#define BLIN_A_Pin GPIO_PIN_13
#define BLIN_A_GPIO_Port GPIOB
#define FIN_B_Pin GPIO_PIN_14
#define FIN_B_GPIO_Port GPIOB
#define FIN_A_Pin GPIO_PIN_15
#define FIN_A_GPIO_Port GPIOB
#define EC_F_B_Pin GPIO_PIN_8
#define EC_F_B_GPIO_Port GPIOA
#define EC_F_A_Pin GPIO_PIN_9
#define EC_F_A_GPIO_Port GPIOA
#define BRIN_B_Pin GPIO_PIN_10
#define BRIN_B_GPIO_Port GPIOA
#define BRIN_A_Pin GPIO_PIN_11
#define BRIN_A_GPIO_Port GPIOA
#define tb_LED_Pin GPIO_PIN_3
#define tb_LED_GPIO_Port GPIOB
#define BEEP_Pin GPIO_PIN_4
#define BEEP_GPIO_Port GPIOB
#define PWM_F_Pin GPIO_PIN_6
#define PWM_F_GPIO_Port GPIOB
#define PWM_BL_Pin GPIO_PIN_7
#define PWM_BL_GPIO_Port GPIOB
#define PWM_BR_Pin GPIO_PIN_8
#define PWM_BR_GPIO_Port GPIOB
#define PWM_SERVO_Pin GPIO_PIN_9
#define PWM_SERVO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
