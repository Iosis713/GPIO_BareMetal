/*
 * Timer.cpp
 *
 *  Created on: May 25, 2025
 *      Author: bartoszlozinski
 */

#include "../Inc/Timer.hpp"

SystemTimer::SystemTimer(const uint32_t ticksPerInterrupt)
{
	SysTick_Config(ticksPerInterrupt);
}


extern "C" void SysTick_Handler(void)
{
	SystemTimer::Inc();
}
