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

Timer::Timer(const uint32_t delay)
	:delay_(delay)
	, lastEnabled_(SystemTimer::Now())
{};

bool Timer::IsExpired()
{
	if ((SystemTimer::Now() -  lastEnabled_) > delay_)
	{
		lastEnabled_ = SystemTimer::Now();
		return true;
	}
	return false;
}

extern "C" void SysTick_Handler(void)
{
	SystemTimer::Inc();
}
