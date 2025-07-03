/*
 * Timer.cpp
 *
 *  Created on: May 25, 2025
 *      Author: bartoszlozinski
 */

#include "../Inc/Timer.hpp"

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

extern "C" [[gnu::used]] void SysTick_Handler(void)
{
	SystemTimer::Inc();
}

void Delay(const uint32_t delay)
{
	const uint32_t startTime = SystemTimer::Now();
	while(SystemTimer::Now() < startTime + delay)
	{
		//just wait
	}
}
