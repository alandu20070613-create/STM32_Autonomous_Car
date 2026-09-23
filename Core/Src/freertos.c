/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor.h"
#include "stdio.h"
#include "string.h"
#include "pid.h"
#include "cJSON.h"
#include <string.h>
#include "oled.h"
#include "HC_SR04.h"
#include "mpu6050.h"
#include "mpuiic.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"

#include "tim.h"
#include "usart.h"
#include "gpio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//extern float Motor1Speed;
//extern float Motor2Speed;
extern char uart_buf[64];
//extern uint16_t Encode1Count;
//extern uint16_t Encode2Count;
extern tPid pid_Motor1Speed;

extern int Motor1PWM;
extern int Motor2PWM;

extern tPid pid_Motor1Speed;
extern tPid pid_Motor2Speed;
extern uint8_t Usart1_ReadBuf[256];
extern uint8_t Usart1_ReadCount;
extern float p, i, d, a, b;
extern uint8_t K1_1[64];
extern float distance;

extern tPid pidInfrared_Tracking;
extern uint8_t InfraredLevel_Read[4];
extern int8_t CurrentState;
extern int8_t PreviousState;
extern float Infraredpid_Speed;  // Infrared PID Output Speed
extern float Infraredpid_Speed1; // Infrared Final PID Output Speed - Motor 1
extern float Infraredpid_Speed2; // Infrared Final PID Output Speed - Motor 2

extern uint8_t Usart3ReceiveData;
extern uint8_t Usart3Array[80];
extern float SR04_Read;
extern float PID_Follow;
extern tPid pidFollow;

extern float pitch, roll, yaw;

extern float MPU6050pid_YawMove;
extern float MPU6050pid_YawMove1;
extern float MPU6050pid_YawMove2;
extern tPid pidMPU6050_YawMovement;

extern uint16_t timerCount;
extern short Encode1Count;
extern short Encode2Count;
extern float Motor1Speed;
extern float Motor2Speed;

extern uint8_t Key_Mode;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId StopTaskHandle;
osThreadId LEDTaskHandle;
osThreadId OLEDTaskHandle;
osThreadId Multi_ModeTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartStopTask(void const * argument);
void StartLEDTask(void const * argument);
void StartOLEDTask(void const * argument);
void StartMulti_ModeTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of StopTask */
  osThreadDef(StopTask, StartStopTask, osPriorityNormal, 0, 128);
  StopTaskHandle = osThreadCreate(osThread(StopTask), NULL);

  /* definition and creation of LEDTask */
  osThreadDef(LEDTask, StartLEDTask, osPriorityNormal, 0, 128);
  LEDTaskHandle = osThreadCreate(osThread(LEDTask), NULL);

  /* definition and creation of OLEDTask */
  osThreadDef(OLEDTask, StartOLEDTask, osPriorityNormal, 0, 128);
  OLEDTaskHandle = osThreadCreate(osThread(OLEDTask), NULL);

  /* definition and creation of Multi_ModeTask */
  osThreadDef(Multi_ModeTask, StartMulti_ModeTask, osPriorityAboveNormal, 0, 128);
  Multi_ModeTaskHandle = osThreadCreate(osThread(Multi_ModeTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartStopTask */
/**
  * @brief  Function implementing the StopTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartStopTask */
void StartStopTask(void const * argument)
{
  /* USER CODE BEGIN StartStopTask */
  /* Infinite loop */
  for(;;)
  {
	    /* Check if the task is running */
		UBaseType_t stackHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
		sprintf((char * )Usart3Array, "StartStopTask Mark:%u words\n", (unsigned int)stackHighWaterMark);
		HAL_UART_Transmit(&huart3, (uint8_t *)Usart3Array, strlen((const char * )Usart3Array), 0XFFFF);
		size_t freeHeapSize = xPortGetFreeHeapSize();
		sprintf((char * )Usart3Array, "StartStopTask Free Heap Size:%u bytes\n", (unsigned int)freeHeapSize);
		HAL_UART_Transmit(&huart3, (uint8_t *)Usart3Array, strlen((const char * )Usart3Array), 0XFFFF);

		if(Key_Mode == 0){
			motorPIDSetSpeed(0, 0);
		}
    osDelay(10);
  }
  /* USER CODE END StartStopTask */
}

/* USER CODE BEGIN Header_StartLEDTask */
/**
* @brief Function implementing the LEDTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLEDTask */
void StartLEDTask(void const * argument)
{
  /* USER CODE BEGIN StartLEDTask */
  /* Infinite loop */
  for(;;)
  {
	/* Check if the task is running */
	UBaseType_t stackHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
	sprintf((char * )Usart3Array, "StartLEDTask Mark:%u words\n", (unsigned int)stackHighWaterMark);
	HAL_UART_Transmit(&huart3, (uint8_t *)Usart3Array, strlen((const char * )Usart3Array), 0XFFFF);
	size_t freeHeapSize = xPortGetFreeHeapSize();
	sprintf((char * )Usart3Array, "StartLEDTask Free Heap Size:%u bytes\n", (unsigned int)freeHeapSize);
	HAL_UART_Transmit(&huart3, (uint8_t *)Usart3Array, strlen((const char * )Usart3Array), 0XFFFF);

	/* OLED Display */
	HAL_GPIO_TogglePin(LEDPC5_GPIO_Port, LEDPC5_Pin);
    osDelay(300);
  }
  /* USER CODE END StartLEDTask */
}

/* USER CODE BEGIN Header_StartOLEDTask */
/**
* @brief Function implementing the OLEDTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartOLEDTask */
void StartOLEDTask(void const * argument)
{
  /* USER CODE BEGIN StartOLEDTask */
  /* Infinite loop */
  for(;;)
  {
	    /* Check if the task is running */
		UBaseType_t stackHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
		sprintf((char * )Usart3Array, "StartOLEDTask Mark:%u words\n", (unsigned int)stackHighWaterMark);
		HAL_UART_Transmit(&huart3, (uint8_t *)Usart3Array, strlen((const char * )Usart3Array), 0XFFFF);
		size_t freeHeapSize = xPortGetFreeHeapSize();
		sprintf((char * )Usart3Array, "StartOLEDTask Free Heap Size:%u bytes\n", (unsigned int)freeHeapSize);
		HAL_UART_Transmit(&huart3, (uint8_t *)Usart3Array, strlen((const char * )Usart3Array), 0XFFFF);

	  /* OLED Display Info (Current Key Mode) */
	  sprintf((char * )K1_1, "Key_Mode:%d", Key_Mode);
	  OLED_ShowString(0, 2, K1_1, 16, 1); // show current key mode

	  /* OLED Display Info (Speed, Distance, Voltage) */
	  sprintf((char * )K1_1, "Motor1Speed:%f Motor2Speed:%f", Motor1Speed, Motor2Speed);
	  OLED_ShowString(0, 3, K1_1, 16, 1); // From oled.c, a function that shows position

	  sprintf((char * )K1_1, "Distance:%f", distance);
	  OLED_ShowString(0, 4, K1_1, 16, 1); // a function that shows distance traveled

//		  sprintf((char * )K1_1, "Battery_Voltage:%.2d", adcGetBatteryVoltage());
//		  OLED_ShowString(0, 2, K1_1, 16, 1); // a function that shows battery voltage

	  sprintf((char * )Usart3Array, "HC_SR04:%.2f\r\n", HC_SR04_Read());
	  OLED_ShowString(0, 5, K1_1, 16, 1); // a function that shows HC_SR04(supersonic wave) value

	  sprintf((char * )Usart3Array, "pitch:%.2f roll:%.2f yaw:%.2f\r\n", pitch, roll, yaw);
	  OLED_ShowString(0, 6, K1_1, 16, 1); // a function that shows 6050 position values

	  /* Receive 6050 data */
//	  while(mpu_dmp_get_data(&pitch, &roll, &yaw) != 0) {};

	  /* Show Motors Stop */
//	  motorPIDSetSpeed(0, 0);

      osDelay(10);
  }
  /* USER CODE END StartOLEDTask */
}

/* USER CODE BEGIN Header_StartMulti_ModeTask */
/**
* @brief Function implementing the Multi_ModeTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMulti_ModeTask */
void StartMulti_ModeTask(void const * argument)
{
  /* USER CODE BEGIN StartMulti_ModeTask */
  /* Infinite loop */
  for(;;)
  {
	    /* Check if the task is running */
		UBaseType_t stackHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
		sprintf((char * )Usart3Array, "StartMulti_ModeTask Mark:%u words\n", (unsigned int)stackHighWaterMark);
		HAL_UART_Transmit(&huart3, (uint8_t *)Usart3Array, strlen((const char * )Usart3Array), 0XFFFF);
		size_t freeHeapSize = xPortGetFreeHeapSize();
		sprintf((char * )Usart3Array, "StartMulti_ModeTask Free Heap Size:%u bytes\n", (unsigned int)freeHeapSize);
		HAL_UART_Transmit(&huart3, (uint8_t *)Usart3Array, strlen((const char * )Usart3Array), 0XFFFF);

	  if(Key_Mode == 1){

		  /* Infrared Movement With PID */
		  InfraredLevel_Read[0] = Read_Infrared_1;
		  InfraredLevel_Read[1] = Read_Infrared_2;
		  InfraredLevel_Read[2] = Read_Infrared_3;
		  InfraredLevel_Read[3] = Read_Infrared_4;

		  if(InfraredLevel_Read[0] == 0 && InfraredLevel_Read[1] == 0 && InfraredLevel_Read[2] == 0 && InfraredLevel_Read[3] == 0){
	//		  sprintf(uart_buf, "Moving Forward\r\n");
	//		  HAL_UART_Transmit(&huart1, (uint8_t *)uart_buf, strlen(uart_buf), 100); // Increase efficiency
			  CurrentState = 0; // move forward
		  }
		  else if(InfraredLevel_Read[0] == 0 && InfraredLevel_Read[1] == 1 && InfraredLevel_Read[2] == 0 && InfraredLevel_Read[3] == 0){
	//		  sprintf(uart_buf, "Turn Right\r\n");
	//		  HAL_UART_Transmit(&huart1, (uint8_t *)uart_buf, strlen(uart_buf), 100); // Increase efficiency
			  CurrentState = -1; // turn right
		  }
		  else if(InfraredLevel_Read[0] == 1 && InfraredLevel_Read[1] == 0 && InfraredLevel_Read[2] == 0 && InfraredLevel_Read[3] == 0){
	//		  sprintf(uart_buf, "Instantly Turn Right\r\n");
	//		  HAL_UART_Transmit(&huart1, (uint8_t *)uart_buf, strlen(uart_buf), 100); // Increase efficiency
			  CurrentState = -2; // instantly turn right
		  }
		  else if(InfraredLevel_Read[0] == 1 && InfraredLevel_Read[1] == 1 && InfraredLevel_Read[2] == 0 && InfraredLevel_Read[3] == 0){
	//		  sprintf(uart_buf, "Instantly Turn Right\r\n");
	//		  HAL_UART_Transmit(&huart1, (uint8_t *)uart_buf, strlen(uart_buf), 100); // Increase efficiency
			  CurrentState = -3; // instantly turn right
		  }
		  else if(InfraredLevel_Read[0] == 0 && InfraredLevel_Read[1] == 0 && InfraredLevel_Read[2] == 1 && InfraredLevel_Read[3] == 0){
	//		  sprintf(uart_buf, "Turn Left\r\n");
	//		  HAL_UART_Transmit(&huart1, (uint8_t *)uart_buf, strlen(uart_buf), 100); // Increase efficiency
			  CurrentState = 1; // turn left
		  }
		  else if(InfraredLevel_Read[0] == 0 && InfraredLevel_Read[1] == 0 && InfraredLevel_Read[2] == 0 && InfraredLevel_Read[3] == 1){
	//		  sprintf(uart_buf, "Instantly Turn Left\r\n");
	//		  HAL_UART_Transmit(&huart1, (uint8_t *)uart_buf, strlen(uart_buf), 100); // Increase efficiency
			  CurrentState = 2; // instantly turn left
		  }
		  else if(InfraredLevel_Read[0] == 0 && InfraredLevel_Read[1] == 0 && InfraredLevel_Read[2] == 1 && InfraredLevel_Read[3] == 1){
	//		  sprintf(uart_buf, "Instantly Turn Left\r\n");
	//		  HAL_UART_Transmit(&huart1, (uint8_t *)uart_buf, strlen(uart_buf), 100); // Increase efficiency
			  CurrentState = 3; // instantly turn left
		  }

		  Infraredpid_Speed = PID_realize(&pidInfrared_Tracking, CurrentState); // PID calculates the target speed
		  Infraredpid_Speed1 = 3 + Infraredpid_Speed; // Motor1 Speed = Initial Speed + PID Output Speed
		  Infraredpid_Speed2 = 3 - Infraredpid_Speed; // Motor2 Speed = Initial Speed - PID Output Speed
		  if(Infraredpid_Speed1 > 5) Infraredpid_Speed1 = 5;
		  if(Infraredpid_Speed1 < 0) Infraredpid_Speed1 = 0;
		  if(Infraredpid_Speed2 > 5) Infraredpid_Speed2 = 5;
		  if(Infraredpid_Speed2 < 0) Infraredpid_Speed2 = 0;
		  if(CurrentState != PreviousState){
			  motorPIDSetSpeed(Infraredpid_Speed1, Infraredpid_Speed2);
		  }
		  PreviousState = CurrentState;
	  }

	  if(Key_Mode == 2){
		  /* Using Computer USART to Control Car */
		  // This is completed in stm32f1xx_it.c
	  }

	  if(Key_Mode == 3){
		  /* Obstacle Avoidance */
		  	  if(HC_SR04_Read() > 25){ // the next 25cm are clear, no obstacle
		  		  motorPIDSetSpeed(1, 1); // move forward
		  		  osDelay(100);
		  	  }
		  	  else{ // obstacle in front
		  		  motorPIDSetSpeed(-1, 1); // turn around (to the right)
		  	   	  osDelay(500);
		  		  if(HC_SR04_Read() > 25){ // no obstacle at the right
		  			  motorPIDSetSpeed(1, 1); // move forward
		  			  osDelay(100);
		  		  }
		  		  else{ // obstacle at the right
		  			  motorPIDSetSpeed(1, -1); // turn around (to the left)
		  			  osDelay(1000);
		  			  if(HC_SR04_Read() > 25){ // no obstacle at the left
		  				  motorPIDSetSpeed(1, 1); // move forward
		  				  osDelay(100);
		  			  }
		  			  else{ // obstacle at the left
		  				  motorPIDSetSpeed(-1, -1); // move backward
		  				  osDelay(1000);
		  				  motorPIDSetSpeed(-1, 1); // turn around (to the right)
		  				  osDelay(50);
		  			  }
		  		  }
		  	  }
	  }

	  if(Key_Mode == 4){
		  /* Following with PID */
		  SR04_Read = HC_SR04_Read(); // read the distance to the obstacle
		  if(SR04_Read < 60){ // start following when the obstacle is 60cm ahead
			  PID_Follow = PID_realize(&pidFollow, SR04_Read); // PID calculates target speed
			  if(PID_Follow > 6) PID_Follow = 6;
			  if(PID_Follow < -6) PID_Follow = -6;
			  motorPIDSetSpeed(PID_Follow, PID_Follow); // speed applies to motors
		  }
		  else motorPIDSetSpeed(0, 0); // stop
		  osDelay(10);
	  }

	  if(Key_Mode == 5){
		  sprintf((char * )Usart3Array, "pitch:%.2f roll:%.2f yaw:%.2f\r\n", pitch, roll, yaw);
		  HAL_UART_Transmit(&huart3, (uint8_t *)Usart3Array, strlen((const char * )Usart3Array), 0XFFFF);


		 	  /* MPU_6050 PID Control */
		 	  // mpu_dmp_get_data(&pitch, &roll, &yaw); // DMP returns angle value
//		 	  while(mpu_dmp_get_data(&pitch, &roll, &yaw) != 0){}

		 	  MPU6050pid_YawMove = PID_realize(&pidMPU6050_YawMovement, yaw); // PID calculates target speed
		 	  MPU6050pid_YawMove1 = 1.5 + MPU6050pid_YawMove;
		 	  MPU6050pid_YawMove2 = 1.5 - MPU6050pid_YawMove;
		 	  if(MPU6050pid_YawMove1 > 3.5) MPU6050pid_YawMove1 = 3.5;
		 	  if(MPU6050pid_YawMove1 < 0) MPU6050pid_YawMove1 = 0;
		 	  if(MPU6050pid_YawMove2 > 3.5) MPU6050pid_YawMove2 = 3.5;
		 	  if(MPU6050pid_YawMove2 < 0) MPU6050pid_YawMove2 = 0;
		 	  motorPIDSetSpeed(MPU6050pid_YawMove1, MPU6050pid_YawMove2);
	  }
    osDelay(10);
  }
  /* USER CODE END StartMulti_ModeTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

