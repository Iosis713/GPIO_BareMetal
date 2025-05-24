/*
 * gpio.hpp
 *
 *  Created on: May 21, 2025
 *      Author: bartoszlozinski
 */

#pragma once
#ifndef INC_GPIO_HPP_
#define INC_GPIO_HPP_

#include "main.h"
#include <cstdint>

template<typename Derived>
class IGpio
{
protected:
	GPIO_TypeDef* Port() const { return reinterpret_cast<GPIO_TypeDef*>(Derived::portAddr); }

public:
	void EnableClock()
	{
		if (Derived::portAddr == GPIOA_BASE)
			RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
		else if (Derived::portAddr == GPIOB_BASE)
			RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
		else if (Derived::portAddr == GPIOC_BASE)
			RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;
		else if (Derived::portAddr == GPIOD_BASE)
			RCC->AHB2ENR |= RCC_AHB2ENR_GPIODEN;
	}
};

template<std::uintptr_t portAddr_, uint8_t pin_>
class GpioOutput : public IGpio<GpioOutput<portAddr_, pin_>>
{
private:
	void ConfigureAsOutput()
	{
		auto port = this->Port();
		static_assert(pin >= 0 && pin < 16, "Invalid pin number: needs to be in range of 0 - 15!");

		// STM32L476RGT6 Reference Manual:
		// GPIOx_MODER (GPIO port mode register) controls the mode of each pin.
		// 00 = Input mode
		// 01 = General purpose output mode
		// 10 = Alternate function mode
		// 11 = Analog mode (reset state)
		port->MODER &= ~MODER_MASKS[pin];
		port->MODER |= MODER_OUTPUT_BITS[pin];

		// GPIOx_OTYPER (GPIO port output type register):
		// 0 = Output push-pull (reset state)
		// 1 = Output open-drain
		port->OTYPER &= ~OTYPER_BITS[pin];

		// GPIOx_OSPEEDR (GPIO port output speed register):
		// 00 = Low speed (reset value)
		// 01 = Medium speed
		// 10 = High speed
		// 11 = Very high speed
		port->OSPEEDR &= ~OSPEEDR_MASKS[pin];

	    // GPIOx_PUPDR (GPIO port pull-up/pull-down register):
		// Reset value: 0x6400 0000 (for port A) - pin15 [pull-up], pin14 [pull-down], others [No pull-up/pull-down]
		// Reset value: 0x0000 0100 (for port B) - pin4 [pull-up]
		// Reset value: 0x0000 0000 (for other ports)

	    // 00 = No pull-up, no pull-down
	    // 01 = Pull-up
	    // 10 = Pull-down
	    // 11 = Reserved
		port->PUPDR &= ~PUPDR_MASKS[pin];
	}

public:
	static constexpr std::uintptr_t portAddr = portAddr_;
	static constexpr uint8_t pin = pin_;

	GpioOutput(const GpioOutput& source) = delete;
	GpioOutput(GpioOutput&& source) = delete;
	GpioOutput& operator=(const GpioOutput& source) = delete;
	GpioOutput& operator=(GpioOutput&& source) = delete;
	GpioOutput()
	{
		this->EnableClock();
		ConfigureAsOutput();
	}
	~GpioOutput() = default;


	bool IsPinSet() const { return this->Port()->ODR & PinMask<pin>(); }

	void Set() { this->Port()->BSRR |= BSRR_BS_MASKS[pin]; }
	void Clear() { this->Port()->BSRR |= BSRR_BR_MASKS[pin]; }
	void Toggle() { this->Port()->ODR ^= ODR_OD_MASKS[pin]; /*Bitwise XOR*/}
};

enum class OptionsPUPDR
{
    None, // 00 = No pull-up, no pull-down
    PullUp, // 01 = Pull-up
    PullDown,// 10 = Pull-down
    /*Reserved,// 11 = Reserved - but you shouldn't use reserved pin*/
};

template<std::uintptr_t portAddr_, uint8_t pin_, OptionsPUPDR pupdrOption>
class GpioInput : public IGpio<GpioInput<portAddr_, pin_, pupdrOption>>
{
protected:
	void ConfigureAsInput()
	{
		auto port = this->Port();
		static_assert(pin >= 0 && pin < 16, "Invalid pin number: needs to be in range of 0 - 15!");
		port->MODER &= ~(MODER_MASKS[pin]); //00 - input
		port->PUPDR &= ~(PUPDR_MASKS[pin]); //00- no pull-up, no pull-down
		{
			using enum OptionsPUPDR;
			if constexpr (pupdrOption == PullUp)
				port->PUPDR |= PUPDR_MASKS_0[pin];
			else if constexpr (pupdrOption == PullDown)
				port->PUPDR |= PUPDR_MASKS_1[pin];
		}
	}
public:
	static constexpr std::uintptr_t portAddr = portAddr_;
	static constexpr uint8_t pin = pin_;

	GpioInput(const GpioInput& source) = delete;
	GpioInput(GpioInput&& source) = delete;
	GpioInput& operator=(const GpioInput& source) = delete;
	GpioInput& operator=(GpioInput&& source) = delete;
	GpioInput()
	{
		this->EnableClock();
		ConfigureAsInput();
	}

	bool ReadPin() const
	{
		//true - high state, false - low state
		return this->Port()->IDR & PinMask<pin>() ;
	};
};

#endif /* INC_GPIO_HPP_ */
