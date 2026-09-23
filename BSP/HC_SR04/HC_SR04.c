#include "HC_SR04.h"


void HC_SR04_Delayus(uint32_t usdelay){
	__IO uint32_t Delay = usdelay * (SystemCoreClock / 8U / 1000U / 1000); // systemcoreclcok frequency
	do{
		__NOP();
	}
	while (Delay--);
}

float HC_SR04_Read(void){
	uint32_t i = 0;
	float Distance;
	HAL_GPIO_WritePin(PB5_Trig_GPIO_Port, PB5_Trig_Pin, GPIO_PIN_SET); // Output HIGH level for 15us
	HC_SR04_Delayus(15);
	HAL_GPIO_WritePin(PB5_Trig_GPIO_Port, PB5_Trig_Pin, GPIO_PIN_RESET); // Turn back to LOW level

	while(HAL_GPIO_ReadPin(PA6_Echo_GPIO_Port, PA6_Echo_Pin) == GPIO_PIN_RESET){ // waiting for it to turn HIGH
		i++;
		HC_SR04_Delayus(1);
		if(i > 100000) return -1; // prevent timeout and the code stuck here
	}
	i = 0;
	while(HAL_GPIO_ReadPin(PA6_Echo_GPIO_Port, PA6_Echo_Pin) == GPIO_PIN_SET){ // the while loop goes for 2us
		i++;
		HC_SR04_Delayus(1); // 1us delay
		if(i > 100000) return -2;
	}
	Distance = i * 2 * 0.033 / 2; //
	return Distance;
}

