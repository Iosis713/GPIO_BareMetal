/*
 * Timer.hpp
 *
 *  Created on: May 25, 2025
 *      Author: bartoszlozinski
 */

#pragma once
#ifndef TIMER_HPP_
#define TIMER_HPP_

#include "Config.hpp"
#include <atomic>

class SystemTimer
{
private:
	inline static std::atomic<uint32_t> tick_ = 0;
public:
	SystemTimer(const SystemTimer& source) = delete;
	SystemTimer(SystemTimer&& source) = delete;
	SystemTimer& operator=(const SystemTimer& source) = delete;
	SystemTimer& operator=(SystemTimer&& source) = delete;
	SystemTimer() = default;

	static void Init(const uint32_t ticksPerInterrupt) { SysTick_Config(ticksPerInterrupt); }
	static uint32_t Now() { return tick_.load(std::memory_order_relaxed); }
	static void Inc() { tick_.fetch_add(1, std::memory_order_relaxed); }
};

class Timer
{
private:
	uint32_t delay_ = 0;
	uint32_t lastEnabled_ = 0;
public:
	Timer(const Timer& source) = delete;
	Timer(Timer&& source) = delete;
	Timer& operator=(const Timer& source) = delete;
	Timer& operator=(Timer&& source) = delete;
	Timer() = delete;
	Timer(const uint32_t delay);

	bool IsExpired();
};

#endif /* TIMER_HPP_ */
