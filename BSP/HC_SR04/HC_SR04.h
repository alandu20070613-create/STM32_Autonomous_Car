/*
 * HC_SR04.h
 *
 *  Created on: Jul 27, 2026
 *      Author: zidu
 */

#ifndef BSP_HC_SR04_HC_SR04_H_
#define BSP_HC_SR04_HC_SR04_H_

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include "gpio.h"

void HC_SR04_Delayus(uint32_t usdelay);
float HC_SR04_Read(void);


#endif /* BSP_HC_SR04_HC_SR04_H_ */
