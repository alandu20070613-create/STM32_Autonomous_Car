/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor.h"
#include "stdio.h"
#include "string.h"
#include "pid.h"
#include "cJSON.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
//uint16_t timerCount = 0;
//short Encode1Count = 0;
//short Encode2Count = 0;
//float Motor1Speed = 0.00;
//float Motor2Speed = 0.00;

uint8_t Usart1_ReadBuf[256];
uint8_t Usart1_ReadCount = 0;
extern tPid pid_Motor1Speed;
extern tPid pid_Motor2Speed;
extern int Motor1PWM;
extern int Motor2PWM;

float distance = 0.00;
extern uint8_t Usart3ReceiveData;
extern tPid pidMPU6050_YawMovement;
extern uint8_t Key_Mode;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern TIM_HandleTypeDef htim3;

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line4 interrupt.
  */
void EXTI4_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_IRQn 0 */

  /* USER CODE END EXTI4_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(KEY1_Pin);
  /* USER CODE BEGIN EXTI4_IRQn 1 */

  /* USER CODE END EXTI4_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt.
  */
void TIM1_UP_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_IRQn 0 */

  /* USER CODE END TIM1_UP_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_IRQn 1 */

  /* USER CODE END TIM1_UP_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */

  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
if(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE)){
	if(Usart1_ReadCount >= 255) Usart1_ReadCount = 0;
	HAL_UART_Receive(&huart1, &Usart1_ReadBuf[Usart1_ReadCount++], 1, 1000);
}
  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */

  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */

  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(KEY2_Pin);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  /* USER CODE END EXTI15_10_IRQn 1 */
}

/* USER CODE BEGIN 1 */
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
//	if(htim == &htim1){ // htim1 500HZ, 2ms interruption
//		timerCount++;
//		if(timerCount % 5 == 0){ // runs every 10ms
//
//			Encode1Count = (short)__HAL_TIM_GET_COUNTER(&htim4);
//			Encode2Count = (short)__HAL_TIM_GET_COUNTER(&htim2);
//			__HAL_TIM_SET_COUNTER(&htim4, 0);
//			__HAL_TIM_SET_COUNTER(&htim2, 0);
//			Motor1Speed = (float)Encode1Count * 100 / 9.6 / 11 / 4;
//			Motor2Speed = (float)Encode2Count * 100 / 9.6 / 11 / 4;
//
//
//		}
//		if(timerCount % 10 == 0){ // runs every 20ms
//
//			distance += 0.02 * Motor1Speed * 22;
//			Motor1PWM = PID_realize(&pid_Motor1Speed, Motor1Speed);
//			Motor2PWM = PID_realize(&pid_Motor2Speed, Motor2Speed);
//			Motor_Set(Motor1PWM, Motor2PWM);
//
//			timerCount = 0;
//		}
//	}
//}

/* Using Computer USART to Control Car */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart == &huart3){
		if(Usart3ReceiveData == 'A') motorPIDSetSpeed(1, 1);   // move forward
		if(Usart3ReceiveData == 'B') motorPIDSetSpeed(-1, -1); // move backward
		if(Usart3ReceiveData == 'C') motorPIDSetSpeed(0, 0);   // stop
		if(Usart3ReceiveData == 'D') motorPIDSetSpeed(1, 2);   // turn right
		if(Usart3ReceiveData == 'E') motorPIDSetSpeed(2, 1);   // turn left
		if(Usart3ReceiveData == 'F') motorPidAccelerate();     // accelerate
		if(Usart3ReceiveData == 'G') motorPidDecelerate();     // decelerate
		if(Usart3ReceiveData == 'H'){                          // rotate 90 degrees clockwise
			if(pidMPU6050_YawMovement.target_val <= 180) pidMPU6050_YawMovement.target_val += 90;
		}
		if(Usart3ReceiveData == 'I'){                          // rotate 90 degrees counter-clockwise
			if(pidMPU6050_YawMovement.target_val >= -180) pidMPU6050_YawMovement.target_val -= 90;
		}
		if(Usart3ReceiveData == 'J'){                          // switching mode
			if(Key_Mode == 5) Key_Mode = 1; // Key_Mode are 0, 1, 2, 3, 4, 5
			else{
				Key_Mode += 1;
			}
		}
		if(Usart3ReceiveData == 'K') Key_Mode = 0;             // displaying mode
		HAL_UART_Receive_IT(&huart3, &Usart3ReceiveData, 1);
	}
}
/* USER CODE END 1 */
