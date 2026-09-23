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
#define OLED_SCL_Pin GPIO_PIN_0
#define OLED_SCL_GPIO_Port GPIOC
#define OLED_SDA_Pin GPIO_PIN_1
#define OLED_SDA_GPIO_Port GPIOC
#define Infrared_1_Pin GPIO_PIN_5
#define Infrared_1_GPIO_Port GPIOA
#define PA6_Echo_Pin GPIO_PIN_6
#define PA6_Echo_GPIO_Port GPIOA
#define Infrared_2_Pin GPIO_PIN_7
#define Infrared_2_GPIO_Port GPIOA
#define LEDPC5_Pin GPIO_PIN_5
#define LEDPC5_GPIO_Port GPIOC
#define Infrared_3_Pin GPIO_PIN_0
#define Infrared_3_GPIO_Port GPIOB
#define Infrared_4_Pin GPIO_PIN_1
#define Infrared_4_GPIO_Port GPIOB
#define MotorA_Pin GPIO_PIN_13
#define MotorA_GPIO_Port GPIOB
#define KEY2_Pin GPIO_PIN_12
#define KEY2_GPIO_Port GPIOA
#define KEY2_EXTI_IRQn EXTI15_10_IRQn
#define MotorB_Pin GPIO_PIN_3
#define MotorB_GPIO_Port GPIOB
#define KEY1_Pin GPIO_PIN_4
#define KEY1_GPIO_Port GPIOB
#define KEY1_EXTI_IRQn EXTI4_IRQn
#define PB5_Trig_Pin GPIO_PIN_5
#define PB5_Trig_GPIO_Port GPIOB
#define SCL_6050_Pin GPIO_PIN_8
#define SCL_6050_GPIO_Port GPIOB
#define SDA_6050_Pin GPIO_PIN_9
#define SDA_6050_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
