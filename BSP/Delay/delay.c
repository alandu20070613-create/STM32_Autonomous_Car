#include "delay.h"
#include <stdint.h>
#include "stm32f1xx_hal.h"

//////////////////////////////////////////////////////////////////////////////////
// Include uC/OS header if uC/OS is enabled
#if SYSTEM_SUPPORT_UCOS
#include "includes.h"					// Header for uC/OS usage
#endif

static uint8_t  fac_us = 0; // Multiplier for microsecond delay
static uint16_t fac_ms = 0; // Multiplier for millisecond delay

#ifdef OS_CRITICAL_METHOD  // OS_CRITICAL_METHOD defined, uC/OS-II is used
// SysTick interrupt handler, required for uC/OS operation
void SysTick_Handler(void)
{
	OSIntEnter();		// Enter interrupt service routine
    OSTimeTick();       // Call uC/OS system tick timer service
    OSIntExit();        // Exit interrupt service routine
}
#endif

// Initialize delay function
// If uC/OS is enabled, initialize uC/OS system tick timing
// HAL uses HCLK as SysTick clock source by hardware default.
// We manually divide HCLK by 8 to fully match original standard-library timing
void delay_init(void)
{
#ifdef OS_CRITICAL_METHOD  // OS_CRITICAL_METHOD defined, uC/OS-II enabled
	uint32_t reload;
#endif

    // HAL obtains main system clock frequency, replaces standard library global SystemCoreClock
    uint32_t SystemCoreClock = HAL_RCC_GetHCLKFreq();
    // Simulate original HCLK/8 clock division to keep delay timing identical
    uint32_t tick_clk = SystemCoreClock / 8;
	fac_us = tick_clk / 1000000;

#ifdef OS_CRITICAL_METHOD  // OS_CRITICAL_METHOD defined, uC/OS-II enabled
	reload = tick_clk;		// Total SysTick counts per second (kHz unit)
	reload *= 1000000 / OS_TICKS_PER_SEC; // Reload value calculated by uC/OS tick frequency
							// SysTick reload register is 24-bit, max value = 16777216
							// Max delay ~1.86s when HCLK=72MHz
	fac_ms = 1000 / OS_TICKS_PER_SEC; // Millisecond base tick for uC/OS
	SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;   	// Enable SysTick interrupt
	SysTick->LOAD = reload; 	// Trigger interrupt every 1/OS_TICKS_PER_SEC second
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;   	// Turn on SysTick counter
#else
	fac_ms = (uint16_t)fac_us * 1000; // Calculate SysTick ticks per millisecond without OS
#endif
}

#ifdef OS_CRITICAL_METHOD	// uC/OS enabled
// Microsecond blocking delay
// nus: required delay time in microseconds
void delay_us(uint32_t nus)
{
	uint32_t ticks;
	uint32_t told, tnow, tcnt = 0;
	uint32_t reload = SysTick->LOAD;	// Read SysTick reload value
	ticks = nus * fac_us; 			// Total SysTick ticks needed
	tcnt = 0;
	told = SysTick->VAL;        	// Record current SysTick counter value
	while(1)
	{
		tnow = SysTick->VAL;
		if(tnow != told)
		{
			if(tnow < told)
				tcnt += told - tnow; // Counter counts down normally
			else
				tcnt += reload - tnow + told;	// Counter underflow wraps around
			told = tnow;
			if(tcnt >= ticks)break;// Exit once required ticks are accumulated
		}
	};
}

// Millisecond blocking delay
// nms: required delay time in milliseconds
void delay_ms(uint16_t nms)
{
	if(OSRunning == TRUE) // uC/OS scheduler is running
	{
		if(nms >= fac_ms) // Long delay, use native uC/OS task delay
		{
   			OSTimeDly(nms / fac_ms); // uC/OS task blocking delay
		}
		nms %= fac_ms;				// Remaining short delay handled by hardware
	}
	delay_us((uint32_t)(nms * 1000));	// Short delay implemented by microsecond loop
}

#else // uC/OS NOT enabled
// Microsecond blocking delay
// nus: target delay length in microseconds
void delay_us(uint32_t nus)
{
	uint32_t temp;
	SysTick->LOAD = nus * fac_us; // Load target tick count
	SysTick->VAL = 0x00;        // Clear current counter register
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk ;          // Start SysTick counting
	do
	{
		temp = SysTick->CTRL;
	}
	while(temp & 0x01 && !(temp & (1 << 16))); // Poll until count complete (COUNTFLAG=1)
	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;       // Disable SysTick counter
	SysTick->VAL = 0X00;       // Reset counter value
}

// Millisecond blocking delay
// Note maximum valid nms range:
// SysTick LOAD register is 24-bit unsigned
// Max delay formula: nms <= 0xFFFFFF * 8 * 1000 / SYSCLK
// For 72MHz system clock, maximum delay = 1864ms
void delay_ms(uint16_t nms)
{
	uint32_t temp;
	SysTick->LOAD = (uint32_t)nms * fac_ms; // Load millisecond tick count
	SysTick->VAL = 0x00;           // Clear counter register
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk ;          // Start counter
	do
	{
		temp = SysTick->CTRL;
	}
	while(temp & 0x01 && !(temp & (1 << 16))); // Wait for timer completion
	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;       // Close SysTick counter
	SysTick->VAL = 0X00;       // Clear counter register
}
#endif
