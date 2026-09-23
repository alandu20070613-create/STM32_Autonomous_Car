/*
 * pid.h
 *
 *  Created on: Jul 20, 2026
 *      Author: zidu
 */

#ifndef BSP_PID_PID_H_
#define BSP_PID_PID_H_

typedef struct {
	float target_val;
	float actual_val;
	float err;        // current error
	float err_last;   // previous error
	float err_sum;    // sum of error
	float Kp, Ki, Kd; // proportion, integration, differentiation coefficient;
} tPid;


float P_realize(tPid * pid, float actual_val);
void PID_init(void);
float PI_realize(tPid * pid, float actual_value);
float PID_realize(tPid * pid, float actual_value);

#endif /* BSP_PID_PID_H_ */
