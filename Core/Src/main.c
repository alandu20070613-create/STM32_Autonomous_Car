/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "cmsis_os.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
//extern float Motor1Speed;
//extern float Motor2Speed;
char uart_buf[64];
//extern uint16_t Encode1Count;
//extern uint16_t Encode2Count;
extern tPid pid_Motor1Speed;

int Motor1PWM;
int Motor2PWM;

extern tPid pid_Motor1Speed;
extern tPid pid_Motor2Speed;
extern uint8_t Usart1_ReadBuf[256];
extern uint8_t Usart1_ReadCount;
float p, i, d, a, b;
uint8_t K1_1[64];
extern float distance;

extern tPid pidInfrared_Tracking; 
uint8_t InfraredLevel_Read[4] = {0};
int8_t CurrentState = 0;
int8_t PreviousState = 0;
float Infraredpid_Speed;  // Infrared PID Output Speed
float Infraredpid_Speed1; // Infrared Final PID Output Speed - Motor 1
float Infraredpid_Speed2; // Infrared Final PID Output Speed - Motor 2

uint8_t Usart3ReceiveData;
uint8_t Usart3Array[80]; 
float SR04_Read;
float PID_Follow;
extern tPid pidFollow;

float pitch, roll, yaw;

float MPU6050pid_YawMove = 0.00;
float MPU6050pid_YawMove1 = 0.00;
float MPU6050pid_YawMove2 = 0.00;
extern tPid pidMPU6050_YawMovement;

uint16_t timerCount = 0;
short Encode1Count = 0;
short Encode2Count = 0;
float Motor1Speed = 0.00;
float Motor2Speed = 0.00;

uint8_t Key_Mode = 0;

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* Input Capture */
//uint8_t rising_flag = 0;
//uint8_t falling_flag = 0;
//uint8_t overflow_val = 0;
//uint16_t cnt_val = 0;

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Input Capture */
//void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
//	if(htim->Instance == TIM5){
//		if(falling_flag == 0){
//
//			if(rising_flag == 1){
//				/* This will capture falling edge */
//				falling_flag = 1;
//				cnt_val = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
//				__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
//			}
//
//			else{
//				rising_flag = 0;
//				falling_flag = 0;
//				overflow_val = 0;
//				cnt_val = 0;
//
//				/* This will capture rising edge */
//				rising_flag = 1;
//				__HAL_TIM_DISABLE(htim);
//				__HAL_TIM_SET_COUNTER(htim, 0);
//				__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
//				__HAL_TIM_ENABLE(htim);
//			}
//		}
//	}
//}
//
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
//	if(htim->Instance == TIM5){
//		if(falling_flag == 0){
//			if(rising_flag == 1){
//				if((overflow_val & 0XFF) == 0XFF){ // Staying HIGH for too long, capture in mandatory
//					falling_flag = 1;
//					__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
//					cnt_val = 0XFFFF;
//				}
//				else{
//					overflow_val++;
//				}
//			}
//		}
//	}
//}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

	/* PWM Output */
//	uint8_t dir = 1;
//	uint16_t val = 250;

	/* Input Capture */
//	uint32_t temp = 0;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

	/* PWM Output */
//  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

   /* Input Capture */
//  HAL_TIM_Base_Start_IT(&htim5);
//  HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_1);

  /* PWM Dead Time */
//  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
//  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim4);
  HAL_TIM_Base_Start_IT(&htim1);
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);

  PID_init();
//  OLED_Init();
//  OLED_Clear();
  HAL_UART_Receive_IT(&huart3, &Usart3ReceiveData, 1);

  HAL_Delay(500);
  MPU_Init();
  while(MPU_Init() != 0);
  while(mpu_dmp_init() != 0);
//  cJSON *cJsonData, *cJsonValue;

//  delay_init();
  /* USER CODE END 2 */

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /* Testing Serial Monitor */
//	  uint8_t str[] = "Hello UART\r\n";
//	  HAL_UART_Transmit(&huart1, str, sizeof(str)-1, 100);
//	  HAL_Delay(500);


	  /* PWM Output */
//	  if(dir == 1) val++;
//	  else val--;
//
//	  if(val >= 300) dir = 0;
//	  if(val == 0) dir = 1;
//	  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, val);
//	  HAL_Delay(10);

	  /* Input Capture */
//	  if(falling_flag == 1){
//		  temp = overflow_val * 65536;
//		  temp += cnt_val;
//		  printf("HIGH pulse width: %lu us\r\n", temp);
//		  falling_flag = 0;
//		  rising_flag = 0;
//	  }

//	  printf("HIGH pulse width: hello!!\r\n");
//	  fflush(stdout);
//	  HAL_Delay(100);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  sprintf((char * )K1_1, "Key_Mode:%d", Key_Mode);
	  OLED_ShowString(0, 0, K1_1, 16, 1); // show current key mode

	  if(Key_Mode == 0){

		  /* OLED Display Info (Speed, Distance, Voltage) */
		  sprintf((char * )K1_1, "Motor1Speed:%f Motor2Speed:%f", Motor1Speed, Motor2Speed);
		  OLED_ShowString(0, 0, K1_1, 16, 1); // From oled.c, a function that shows position

		  sprintf((char * )K1_1, "Distance:%f", distance);
		  OLED_ShowString(0, 1, K1_1, 16, 1); // a function that shows distance traveled

//		  sprintf((char * )K1_1, "Battery_Voltage:%.2d", adcGetBatteryVoltage());
//		  OLED_ShowString(0, 2, K1_1, 16, 1); // a function that shows battery voltage

		  sprintf((char * )Usart3Array, "HC_SR04:%.2f\r\n", HC_SR04_Read());
		  OLED_ShowString(0, 2, K1_1, 16, 1); // a function that shows HC_SR04(supersonic wave) value

		  sprintf((char * )Usart3Array, "pitch:%.2f roll:%.2f yaw:%.2f\r\n", pitch, roll, yaw);
		  OLED_ShowString(0, 2, K1_1, 16, 1); // a function that shows 6050 position values

		  /* Receive 6050 data */
//		  while(mpu_dmp_get_data(&pitch, &roll, &yaw) != 0) {};

		  /* Show Motors Stop */
		  motorPIDSetSpeed(0, 0);
	  }

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
		  		  HAL_Delay(100);
		  	  }
		  	  else{ // obstacle in front
		  		  motorPIDSetSpeed(-1, 1); // turn around (to the right)
		  		  HAL_Delay(500);
		  		  if(HC_SR04_Read() > 25){ // no obstacle at the right
		  			  motorPIDSetSpeed(1, 1); // move forward
		  			  HAL_Delay(100);
		  		  }
		  		  else{ // obstacle at the right
		  			  motorPIDSetSpeed(1, -1); // turn around (to the left)
		  			  HAL_Delay(1000);
		  			  if(HC_SR04_Read() > 25){ // no obstacle at the left
		  				  motorPIDSetSpeed(1, 1); // move forward
		  				  HAL_Delay(100);
		  			  }
		  			  else{ // obstacle at the left
		  				  motorPIDSetSpeed(-1, -1); // move backward
		  				  HAL_Delay(1000);
		  				  motorPIDSetSpeed(-1, 1); // turn around (to the right)
		  				  HAL_Delay(50);
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
		  HAL_Delay(10);
	  }

	  if(Key_Mode == 5){
		  sprintf((char * )Usart3Array, "pitch:%.2f roll:%.2f yaw:%.2f\r\n", pitch, roll, yaw);
		 	  HAL_UART_Transmit(&huart3, (uint8_t *)Usart3Array, strlen((const char * )Usart3Array), 0XFFFF);
		 	  HAL_Delay(1000);

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

	  /* PWM control */
//	  if(Motor1Speed < 2.9) Motor1PWM++;
//	  if(Motor1Speed > 3.1) Motor1PWM--;
//	  if(Motor2Speed < 2.9) Motor2PWM++;
//	  if(Motor2Speed > 3.1) Motor2PWM--;


//	  sprintf(uart_buf, "Encode1Count: %d\r\n", Encode1Count);
//	  HAL_UART_Transmit(&huart1, (uint8_t *)uart_buf, strlen(uart_buf), 100);
//
//	  sprintf(uart_buf, "Encode2Count: %d\r\n", Encode2Count);
//	  HAL_UART_Transmit(&huart1, (uint8_t *)uart_buf, strlen(uart_buf), 100);

//	  sprintf(uart_buf, "Motor1Speed: %d\r\n", Motor1Speed);
//	  HAL_UART_Transmit(&huart1, (uint8_t *)uart_buf, strlen(uart_buf), 100);
//
//	  sprintf(uart_buf, "Motor2Speed: %d\r\n", Motor2Speed);
//	  HAL_UART_Transmit(&huart1, (uint8_t *)uart_buf, strlen(uart_buf), 100);
//
//	  sprintf(uart_buf, "Motor1PWM: %d\r\n", Motor1PWM);
//	  HAL_UART_Transmit(&huart1, (uint8_t *)uart_buf, strlen(uart_buf), 100);
//
//	  sprintf(uart_buf, "Motor2PWM: %d\r\n", Motor2PWM);
//	  HAL_UART_Transmit(&huart1, (uint8_t *)uart_buf, strlen(uart_buf), 100);
//	  HAL_Delay(10);


	  /* Control Car Movement */
//	  motorPIDSetSpeed(1, 2); // turn right (Motor2 runs faster than Motor1, Motor2 on the left)
//	  motorPIDSetSpeed(2, 1); // turn left (Motor1 runs faster than Motor2)
//	  motorPIDSetSpeed(1, 1); // move forward
//	  motorPIDSetSpeed(-1, -1); // move backward
//	  motorPIDSetSpeed(-1, 1); // turn around (to the right)
//	  motorPIDSetSpeed(1, -1); // turn around (to the left)
//    motorPidAccelerate();
//	  motorPidDecelerate();

	  /* OLED Display Info (Speed, Distance, Voltage) */
//	  sprintf((char * )K1_1, "Motor1Speed:%d Motor2Speed:%d", Motor1Speed, Motor2Speed);
//	  OLED_ShowString(0, 0, K1_1, 16, 1); // From oled.c, a function that shows position
//
//	  sprintf((char * )K1_1, "Distance:%d", distance);
//	  OLED_ShowString(0, 1, K1_1, 16, 1); // a function that shows distance traveled
//
//	  sprintf((char * )K1_1, "Battery_Voltage:%.2f", adcGetBatteryVoltage());
//	  OLED_ShowString(0, 2, K1_1, 16, 1); // a function that shows battery voltage
//
//	  sprintf((char * )Usart3Array, "HC_SR04:%.2f\r\n", HC_SR04_Read());
//	  OLED_ShowString(0, 2, K1_1, 16, 1); // a function that shows HC_SR04(supersonic wave) value
//
//	  sprintf((char * )Usart3Array, "pitch:%.2f roll:%.2f yaw:%.2f\r\n", pitch, roll, yaw);
//	  OLED_ShowString(0, 2, K1_1, 16, 1); // a function that shows 6050 position values
//
//	  OLED_Refresh();

	  /* Infrared Movement With PID */
//	  InfraredLevel_Read[0] = Read_Infrared_1;
//	  InfraredLevel_Read[1] = Read_Infrared_2;
//	  InfraredLevel_Read[2] = Read_Infrared_3;
//	  InfraredLevel_Read[3] = Read_Infrared_4;
//
//	  if(InfraredLevel_Read[0] == 0 && InfraredLevel_Read[1] == 0 && InfraredLevel_Read[2] == 0 && InfraredLevel_Read[3] == 0){
////		  sprintf(uart_buf, "Moving Forward\r\n");
////		  HAL_UART_Transmit(&huart1, (uint8_t *)uart_buf, strlen(uart_buf), 100); // Increase efficiency
//		  CurrentState = 0; // move forward
//	  }
//	  else if(InfraredLevel_Read[0] == 0 && InfraredLevel_Read[1] == 1 && InfraredLevel_Read[2] == 0 && InfraredLevel_Read[3] == 0){
////		  sprintf(uart_buf, "Turn Right\r\n");
////		  HAL_UART_Transmit(&huart1, (uint8_t *)uart_buf, strlen(uart_buf), 100); // Increase efficiency
//		  CurrentState = -1; // turn right
//	  }
//	  else if(InfraredLevel_Read[0] == 1 && InfraredLevel_Read[1] == 0 && InfraredLevel_Read[2] == 0 && InfraredLevel_Read[3] == 0){
////		  sprintf(uart_buf, "Instantly Turn Right\r\n");
////		  HAL_UART_Transmit(&huart1, (uint8_t *)uart_buf, strlen(uart_buf), 100); // Increase efficiency
//		  CurrentState = -2; // instantly turn right
//	  }
//	  else if(InfraredLevel_Read[0] == 1 && InfraredLevel_Read[1] == 1 && InfraredLevel_Read[2] == 0 && InfraredLevel_Read[3] == 0){
////		  sprintf(uart_buf, "Instantly Turn Right\r\n");
////		  HAL_UART_Transmit(&huart1, (uint8_t *)uart_buf, strlen(uart_buf), 100); // Increase efficiency
//		  CurrentState = -3; // instantly turn right
//	  }
//	  else if(InfraredLevel_Read[0] == 0 && InfraredLevel_Read[1] == 0 && InfraredLevel_Read[2] == 1 && InfraredLevel_Read[3] == 0){
////		  sprintf(uart_buf, "Turn Left\r\n");
////		  HAL_UART_Transmit(&huart1, (uint8_t *)uart_buf, strlen(uart_buf), 100); // Increase efficiency
//		  CurrentState = 1; // turn left
//	  }
//	  else if(InfraredLevel_Read[0] == 0 && InfraredLevel_Read[1] == 0 && InfraredLevel_Read[2] == 0 && InfraredLevel_Read[3] == 1){
////		  sprintf(uart_buf, "Instantly Turn Left\r\n");
////		  HAL_UART_Transmit(&huart1, (uint8_t *)uart_buf, strlen(uart_buf), 100); // Increase efficiency
//		  CurrentState = 2; // instantly turn left
//	  }
//	  else if(InfraredLevel_Read[0] == 0 && InfraredLevel_Read[1] == 0 && InfraredLevel_Read[2] == 1 && InfraredLevel_Read[3] == 1){
////		  sprintf(uart_buf, "Instantly Turn Left\r\n");
////		  HAL_UART_Transmit(&huart1, (uint8_t *)uart_buf, strlen(uart_buf), 100); // Increase efficiency
//		  CurrentState = 3; // instantly turn left
//	  }
//
//	  Infraredpid_Speed = PID_realize(&pidInfrared_Tracking, CurrentState); // PID calculates the target speed
//	  Infraredpid_Speed1 = 3 + Infraredpid_Speed; // Motor1 Speed = Initial Speed + PID Output Speed
//	  Infraredpid_Speed2 = 3 - Infraredpid_Speed; // Motor2 Speed = Initial Speed - PID Output Speed
//	  if(Infraredpid_Speed1 > 5) Infraredpid_Speed1 = 5;
//	  if(Infraredpid_Speed1 < 0) Infraredpid_Speed1 = 0;
//	  if(Infraredpid_Speed2 > 5) Infraredpid_Speed2 = 5;
//	  if(Infraredpid_Speed2 < 0) Infraredpid_Speed2 = 0;
//	  if(CurrentState != PreviousState){
//		  motorPIDSetSpeed(Infraredpid_Speed1, Infraredpid_Speed2);
//	  }
//	  PreviousState = CurrentState;
	  
	  /* Infrared Movement With No PID */
//	  if(Read_Infrared_1 == 0 && Read_Infrared_2 == 0 && Read_Infrared_3 == 0 && Read_Infrared_4 == 0){
//		  sprintf(uart_buf, "Moving Forward\r\n");
//		  HAL_UART_Transmit(&huart1, (uint8_t *)uart_buf, strlen(uart_buf), 100);
//		  motorPIDSetSpeed(1, 1); // move forward
//	  }
//	  if(Read_Infrared_1 == 0 && Read_Infrared_2 == 1 && Read_Infrared_3 == 0 && Read_Infrared_4 == 0){
//		  sprintf(uart_buf, "Turning Right\r\n");
//		  HAL_UART_Transmit(&huart1, (uint8_t *)uart_buf, strlen(uart_buf), 100);
//		  motorPIDSetSpeed(0.5, 2); // turn right
//	  }
//	  if(Read_Infrared_1 == 1 && Read_Infrared_2 == 0 && Read_Infrared_3 == 0 && Read_Infrared_4 == 0){
//		  sprintf(uart_buf, "Instantly Turning Right\r\n");
//		  HAL_UART_Transmit(&huart1, (uint8_t *)uart_buf, strlen(uart_buf), 100);
//		  motorPIDSetSpeed(0.5, 2.5); // instantly turn right
//	  }
//
//	  if(Read_Infrared_1 == 0 && Read_Infrared_2 == 0 && Read_Infrared_3 == 1 && Read_Infrared_4 == 0){
//		  sprintf(uart_buf, "Turning Left\r\n");
//		  HAL_UART_Transmit(&huart1, (uint8_t *)uart_buf, strlen(uart_buf), 100);
//		  motorPIDSetSpeed(2, 0.5); // turn left
//	  }
//
//	  if(Read_Infrared_1 == 0 && Read_Infrared_2 == 0 && Read_Infrared_3 == 0 && Read_Infrared_4 == 1){
//		  sprintf(uart_buf, "Instantly Turning Left\r\n");
//		  HAL_UART_Transmit(&huart1, (uint8_t *)uart_buf, strlen(uart_buf), 100);
//		  motorPIDSetSpeed(2.5, 0.5); // instantly turn left
//	  }
	  /* Using Computer USART to Control Car */

//	  sprintf((char * )Usart3Array, "Motor1Speed:%.2f Motor2Speed:%.2f\r\n", Motor1Speed, Motor2Speed);
//	  HAL_UART_Transmit(&huart3, (uint8_t *)Usart3Array, strlen((const char * )Usart3Array), 50);
//
//	  sprintf((char * )Usart3Array, "Distance:%.2f\r\n", distance);
//	  HAL_UART_Transmit(&huart3, (uint8_t *)Usart3Array, strlen((const char * )Usart3Array), 50);
//
//	  sprintf((char * )Usart3Array, "Battery_Voltage:%.2f\r\n", adcGetBatteryVoltage());
//	  HAL_UART_Transmit(&huart3, (uint8_t *)Usart3Array, strlen((const char * )Usart3Array), 50);
//
	  sprintf((char * )Usart3Array, "HC_SR04:%.2f\r\n", HC_SR04_Read());
	  HAL_UART_Transmit(&huart3, (uint8_t *)Usart3Array, strlen((const char * )Usart3Array), 0XFFFF);
	  HAL_Delay(1000);

//	  sprintf((char * )Usart3Array, "pitch:%.2f roll:%.2f yaw:%.2f\r\n", pitch, roll, yaw);
//	  HAL_UART_Transmit(&huart3, (uint8_t *)Usart3Array, strlen((const char * )Usart3Array), 0XFFFF);
//	  HAL_Delay(1000);
//
//	  /* MPU_6050 PID Control */
//	  // mpu_dmp_get_data(&pitch, &roll, &yaw); // DMP returns angle value
//	  while(mpu_dmp_get_data(&pitch, &roll, &yaw) != 0){}
//
//	  MPU6050pid_YawMove = PID_realize(&pidMPU6050_YawMovement, yaw); // PID calculates target speed
//	  MPU6050pid_YawMove1 = 1.5 + MPU6050pid_YawMove;
//	  MPU6050pid_YawMove2 = 1.5 - MPU6050pid_YawMove;
//	  if(MPU6050pid_YawMove1 > 3.5) MPU6050pid_YawMove1 = 3.5;
//	  if(MPU6050pid_YawMove1 < 0) MPU6050pid_YawMove1 = 0;
//	  if(MPU6050pid_YawMove2 > 3.5) MPU6050pid_YawMove2 = 3.5;
//	  if(MPU6050pid_YawMove2 < 0) MPU6050pid_YawMove2 = 0;
//	  motorPIDSetSpeed(MPU6050pid_YawMove1, MPU6050pid_YawMove2);

      /* cJson Usart Edit */
//	  if(Usart_WaitReasFinish() == 0){ // check if data finished receiving
//		  cJsonData = cJSON_Parse((const char *)Usart1_ReadBuf);
//
//		  if(cJSON_GetObjectItem(cJsonData, "p") != NULL){
//			  cJsonValue = cJSON_GetObjectItem(cJsonData, "p");
//			  p = cJsonValue->valuedouble;
//			  pid_Motor1Speed.Kp = p;
//		  }
//		  if(cJSON_GetObjectItem(cJsonData, "i") != NULL){
//			  cJsonValue = cJSON_GetObjectItem(cJsonData, "i");
//			  i = cJsonValue->valuedouble;
//			  pid_Motor1Speed.Ki = i;
//		  }
//		  if(cJSON_GetObjectItem(cJsonData, "d") != NULL){
//			  cJsonValue = cJSON_GetObjectItem(cJsonData, "d");
//			  d = cJsonValue->valuedouble;
//			  pid_Motor1Speed.Kd = d;
//		  }
//		  if(cJSON_GetObjectItem(cJsonData, "a") != NULL){
//			  cJsonValue = cJSON_GetObjectItem(cJsonData, "a");
//			  a = cJsonValue->valuedouble;
//			  pid_Motor1Speed.target_val = a;
//		  }
//		  if(cJSON_GetObjectItem(cJsonData, "b") != NULL){
//			  cJsonValue = cJSON_GetObjectItem(cJsonData, "b");
//			  b = cJsonValue->valuedouble;
//			  pid_Motor2Speed.target_val = b;
//		  }
//		  if(cJsonData != NULL){
//			  cJSON_Delete(cJsonData); // release RAM space, but keeping cJSONValue
//		  }
//		  memset(Usart1_ReadBuf, 0, sizeof(Usart1_ReadBuf)); //clean buf, do not use strlen here
//		  sprintf(uart_buf, "P: %.2f  I: %.2f  D: %.2f  A: %.2f\r\n", p, i, d, a);
//		  HAL_UART_Transmit(&huart1, (uint8_t *)uart_buf, strlen(uart_buf), 100);
//	  }

	  /* Obstacle Avoidance */
//	  if(HC_SR04_Read() > 25){ // the next 25cm are clear, no obstacle
//		  motorPIDSetSpeed(1, 1); // move forward
//		  HAL_Delay(100);
//	  }
//	  else{ // obstacle in front
//		  motorPIDSetSpeed(-1, 1); // turn around (to the right)
//		  HAL_Delay(500);
//		  if(HC_SR04_Read() > 25){ // no obstacle at the right
//			  motorPIDSetSpeed(1, 1); // move forward
//			  HAL_Delay(100);
//		  }
//		  else{ // obstacle at the right
//			  motorPIDSetSpeed(1, -1); // turn around (to the left)
//			  HAL_Delay(1000);
//			  if(HC_SR04_Read() > 25){ // no obstacle at the left
//				  motorPIDSetSpeed(1, 1); // move forward
//				  HAL_Delay(100);
//			  }
//			  else{ // obstacle at the left
//				  motorPIDSetSpeed(-1, -1); // move backward
//				  HAL_Delay(1000);
//				  motorPIDSetSpeed(-1, 1); // turn around (to the right)
//				  HAL_Delay(50);
//			  }
//		  }
//	  }
//  }

  /* Following without PID */
//  if(HC_SR04_Read() > 25){
//	  motorPIDSetSpeed(1, 1); // move forward
//	  HAL_Delay(100);
//  }
//  if(HC_SR04_Read() < 20){
//	  motorPIDSetSpeed(-1, -1); // move backward
//	  HAL_Delay(100);
//  }

  /* Following with PID */
//  SR04_Read = HC_SR04_Read(); // read the distance to the obstacle
//  if(SR04_Read < 60){ // start following when the obstacle is 60cm ahead
//	  PID_Follow = PID_realize(&pidFollow, SR04_Read); // PID calculates target speed
//	  if(PID_Follow > 6) PID_Follow = 6;
//	  if(PID_Follow < -6) PID_Follow = -6;
//	  motorPIDSetSpeed(PID_Follow, PID_Follow); // speed applies to motors
//  }
//  else motorPIDSetSpeed(0, 0); // stop
//  HAL_Delay(10);
  }


  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* Printf Redirection */
//int fputc(int ch, FILE *f)
//{
//    uint8_t temp = (uint8_t)ch;
//    // 非阻塞发送，不等待，避免卡死printf
//    HAL_UART_Transmit_IT(&huart1, &temp, 1);
//    // 等待发送完成标志
//    while(HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_TX);
//    return ch;
//}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM3 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
	if(htim == &htim1){ // htim1 500HZ, 2ms interruption
		timerCount++;
		if(timerCount % 5 == 0){ // runs every 10ms

			Encode1Count = (short)__HAL_TIM_GET_COUNTER(&htim4);
			Encode2Count = (short)__HAL_TIM_GET_COUNTER(&htim2);
			__HAL_TIM_SET_COUNTER(&htim4, 0);
			__HAL_TIM_SET_COUNTER(&htim2, 0);
			Motor1Speed = (float)Encode1Count * 100 / 9.6 / 11 / 4;
			Motor2Speed = (float)Encode2Count * 100 / 9.6 / 11 / 4;


		}
		if(timerCount % 10 == 0){ // runs every 20ms

			distance += 0.02 * Motor1Speed * 22;
			Motor1PWM = PID_realize(&pid_Motor1Speed, Motor1Speed);
			Motor2PWM = PID_realize(&pid_Motor2Speed, Motor2Speed);
			Motor_Set(Motor1PWM, Motor2PWM);

			timerCount = 0;
		}
	}
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM3)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
