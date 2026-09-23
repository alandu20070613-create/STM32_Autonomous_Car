#ifndef __DELAY_H
#define __DELAY_H

#include <stdint.h>
#include "stm32f1xx_hal.h"

// Set 1 to enable uC/OS timing logic, set 0 for bare-metal only
#define SYSTEM_SUPPORT_UCOS  0

void delay_init(void);
void delay_us(uint32_t nus);
void delay_ms(uint16_t nms);

#endif





























