/*
 * led.h
 *
 *  Created on: Jul 20, 2026
 *      Author: zidu
 */

#ifndef BSP_LED_LED_H_
#define BSP_LED_LED_H_

#define LEDPC5(x)  x ? HAL_GPIO_WritePin(LEDPC5_GPIO_Port, LEDPC5_Pin, GPIO_PIN_SET) : HAL_GPIO_WritePin(LEDPC5_GPIO_Port, LEDPC5_Pin, GPIO_PIN_RESET)


#endif /* BSP_LED_LED_H_ */
