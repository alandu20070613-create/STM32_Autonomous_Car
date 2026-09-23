/*
 * pid.c
 *
 *  Created on: Jul 20, 2026
 *      Author: zidu
 */

#include "pid.h"


tPid pid_Motor1Speed;
tPid pid_Motor2Speed;
tPid pidInfrared_Tracking;
tPid pidFollow;
tPid pidMPU6050_YawMovement;
void PID_init() {
	pid_Motor1Speed.actual_val = 0.0;
	pid_Motor1Speed.target_val = 0.00;
	pid_Motor1Speed.err = 0.0;
	pid_Motor1Speed.err_last = 0.0;
	pid_Motor1Speed.err_sum = 0.0;
	pid_Motor1Speed.Kp = 15.0;
	pid_Motor1Speed.Ki = 5.0;
	pid_Motor1Speed.Kd = 0.0;

	pid_Motor2Speed.actual_val = 0.0;
	pid_Motor2Speed.target_val = 0.00;
	pid_Motor2Speed.err = 0.0;
	pid_Motor2Speed.err_last = 0.0;
	pid_Motor2Speed.err_sum = 0.0;
	pid_Motor2Speed.Kp = 15.0;
	pid_Motor2Speed.Ki = 5.0;
	pid_Motor2Speed.Kd = 0.0;
	
	pidInfrared_Tracking.actual_val = 0.0;
	pidInfrared_Tracking.target_val = 0.00;
	pidInfrared_Tracking.err = 0.0;
	pidInfrared_Tracking.err_last = 0.0;
	pidInfrared_Tracking.err_sum = 0.0;
	pidInfrared_Tracking.Kp = -1.5;
	pidInfrared_Tracking.Ki = 5.0;
	pidInfrared_Tracking.Kd = 0.8;

	pidFollow.actual_val = 0.0;
	pidFollow.target_val = 22.5;
	pidFollow.err = 0.0;
	pidFollow.err_last = 0.0;
	pidFollow.err_sum = 0.0;
	pidFollow.Kp = -0.5;
	pidFollow.Ki = -0.001;
	pidFollow.Kd = 0;

	pidMPU6050_YawMovement.actual_val = 0.0;
	pidMPU6050_YawMovement.target_val = 0.0;
	pidMPU6050_YawMovement.err = 0.0;
	pidMPU6050_YawMovement.err_last = 0.0;
	pidMPU6050_YawMovement.err_sum = 0.0;
	pidMPU6050_YawMovement.Kp = 2;
	pidMPU6050_YawMovement.Ki = 0;
	pidMPU6050_YawMovement.Kd = 0;
}

/* P function */
float P_realize(tPid * pid, float actual_val){
	pid->actual_val = actual_val; // transmit actual value
	pid->err = pid->target_val - pid->actual_val; // current error = target value - actual value
	pid->actual_val = pid->Kp * pid->err;
	return pid->actual_val; // Output Kp * current error
}

/* PI function */
float PI_realize(tPid * pid, float actual_val){
	pid->actual_val = actual_val; // transmit actual value
	pid->err = pid->target_val - pid->actual_val; // current error = target value - actual value
	pid->err_sum += pid->err; // error sum = current accumulation of error values
	pid->actual_val = pid->Kp * pid->err + pid->Ki * pid->err_sum;
	return pid->actual_val; // Output Kp * current error + Ki * error sum
}

/* PID function */
float PID_realize(tPid * pid, float actual_val){
	pid->actual_val = actual_val; // transmit actual value
	pid->err = pid->target_val - pid->actual_val; // current error = target value - actual value
	pid->err_sum += pid->err; // error sum = current accumulation of error values
	pid->actual_val = pid->Kp * pid->err + pid->Ki * pid->err_sum + pid->Kd * (pid->err - pid->err_last);
	pid->err_last = pid->err;
	return pid->actual_val;
}
