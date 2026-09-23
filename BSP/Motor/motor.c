/*
 * motor.c
 *
 *  Created on: Jul 23, 2026
 *      Author: zidu
 */

#include "motor.h"
#include "tim.h"
#include "pid.h"

#define MAX_ACCELERATION  3
#define MIN_ACCELERATION  0.5

extern tPid pid_Motor1Speed;
extern tPid pid_Motor2Speed;
extern uint16_t Motor1Speed;
extern uint16_t Motor2Speed;
extern int Motor1PWM;
extern int Motor2PWM;

float Motor_Acceleration = 0.5; // acceleration/deceleration variable

void Motor_Set(int Motor1, int Motor2){

	// Determine HIGH/LOW according to direction of motors
	if(Motor1 < 0) MotorB_SET;
	else MotorB_RESET;

	if(Motor2 < 0) MotorA_SET;
	else MotorA_RESET;

	// Set PWM Duty Cycle
	if(Motor1 < 0){
		if(Motor1 < -99) Motor1 = -99;
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (100 + Motor1));
	}
	else{
		if(Motor1 > 99) Motor1 = 99;
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, Motor1);
	}

	if(Motor2 < 0){
		if(Motor2 < -99) Motor2 = -99;
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, (100 + Motor2));
	}
	else{
		if(Motor2 > 99) Motor2 = 99;
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, Motor2);
	}
}

//	  motorPIDSetSpeed(1, 2); // turn right (Motor2 runs faster than Motor1, Motor2 on the left)
//	  motorPIDSetSpeed(2, 1); // turn left (Motor1 runs faster than Motor2)
//	  motorPIDSetSpeed(1, 1); // move forward
//	  motorPIDSetSpeed(-1, -1); // move backward
//	  motorPIDSetSpeed(-1, 1); // turn around (to the right)
//	  motorPIDSetSpeed(1, -1); // turn around (to the left)
//    motorPidAccelerate();
//	  motorPidDecelerate();
void motorPIDSetSpeed(float Motor1SetSpeed, float Motor2SetSpeed){

	// Set PID target speed
	pid_Motor1Speed.target_val = Motor1SetSpeed;
	pid_Motor2Speed.target_val = Motor2SetSpeed;

	// PID Motors Control
	Motor1PWM = PID_realize(&pid_Motor1Speed, Motor1Speed);
	Motor2PWM = PID_realize(&pid_Motor2Speed, Motor2Speed);
	Motor_Set(Motor1PWM, Motor2PWM);
}

void motorPidAccelerate(void){
	if(Motor_Acceleration <= MAX_ACCELERATION) Motor_Acceleration += 0.5;
	motorPIDSetSpeed(Motor_Acceleration, Motor_Acceleration);
}

void motorPidDecelerate(void){
	if(Motor_Acceleration >= MIN_ACCELERATION) Motor_Acceleration -= 0.5;
	motorPIDSetSpeed(Motor_Acceleration, Motor_Acceleration);
}



