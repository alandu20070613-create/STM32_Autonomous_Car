#ifndef MOTOR_H__
#define MOTOR_H__

#include "main.h"


#define MotorA_SET   HAL_GPIO_WritePin(MotorA_GPIO_Port, MotorA_Pin, GPIO_PIN_SET)
#define MotorA_RESET HAL_GPIO_WritePin(MotorA_GPIO_Port, MotorA_Pin, GPIO_PIN_RESET)

#define MotorB_SET   HAL_GPIO_WritePin(MotorB_GPIO_Port, MotorB_Pin, GPIO_PIN_SET)
#define MotorB_RESET HAL_GPIO_WritePin(MotorB_GPIO_Port, MotorB_Pin, GPIO_PIN_RESET)

void Motor_Set(int Motor1, int Motor2);
void motorPIDSetSpeed(float Motor1SetSpeed, float Motor2SetSpeed);
void motorPidAccelerate(void);
void motorPidDecelerate(void);

#endif
