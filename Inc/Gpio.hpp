/*
 * gpio.hpp
 *
 *  Created on: May 21, 2025
 *      Author: bartoszlozinski
 */

#pragma once
#ifndef INC_GPIO_HPP_
#define INC_GPIO_HPP_

#include "Config.hpp"
#include <cstdint>

enum class OptionsPUPDR
{
	// GPIOx_PUPDR (GPIO port pull-up/pull-down register):
    // Reset value: 0x6400 0000 (for port A) - pin15 [pull-up], pin14 [pull-down], others [No pull-up/pull-down]
	// Reset value: 0x0000 0100 (for port B) - pin4 [pull-up]
	// Reset value: 0x0000 0000 (for other ports)
    None, // 00 = No pull-up, no pull-down
    PullUp, // 01 = Pull-up
    PullDown,// 10 = Pull-down
    /*Reserved,// 11 = Reserved - but you shouldn't use reserved pin*/
};

enum class OptionsOTYPER
{
	// GPIOx_OTYPER (GPIO port output type register):
	//reset value 0x0000 0000
	PushPull, // 0 = Output push-pull (reset state)
	OpenDrain,// 1 = Output open-drain
};

enum class OptionsMODER
{
	// STM32L476RGT6 Reference Manual:
	// GPIOx_MODER (GPIO port mode register) controls the mode of each pin
	//Reset value: 0xABFF FFFF (port A) - 1010 1011 1111 1111 1111 1111 1111 1111
	//Reset value: 0xFFFF FEBF (port B) - 1111 1111 1111 1111 1111 1110 1011 1111
	Input, // 00 = Input mode
	Output, // 01 = General purpose output mode
	Alternate, // 10 = Alternate function mode
	Analog, // 11 = Analog mode (reset state for most cases)
};

enum class OptionsOSPEEDR
{
	// STM32L476RGT6 RM: 8.5.3 GPIO port output speed register
	// Reset value: 0x0C00 0000 for port A
	// Reset value: 0x0000 0000 for other ports
	LowSpeed, // 00
	MediumSpeed, // 01
	HighSpeed, // 10
	VeryHighSpeed, //11
	//Refer to the device datasheet for the frequency specifications
	//and the power supply and load conditions for each speed...
};

//Datasheet Pinouts and pin description
//AF2 Datasheet Alternate functino 0 - 7
enum class AlternateFunction
{
	AF0,
	AF1,
	AF2,
	AF3,
	AF4,
	AF5,
	AF6,
	AF7,
	AF8,
	AF9,
	AF10,
	AF11,
	AF12,
	AF13,
	AF14,
	AF15
};

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

	template<OptionsPUPDR pupdrOption>
	void ConfigurePUPDR()
	{
		using enum OptionsPUPDR;
		auto port = this->Port();
		port->PUPDR &= ~(PUPDR_MASKS[Derived::pin]); //00- no pull-up, no pull-down
		if constexpr (pupdrOption == PullUp)
			port->PUPDR |= PUPDR_MASKS_0[Derived::pin];
		else if constexpr (pupdrOption == PullDown)
			port->PUPDR |= PUPDR_MASKS_1[Derived::pin];
	}

	template<OptionsOTYPER otyperOption>
	void ConfigureOTYPER()
	{
		using enum OptionsOTYPER;
		auto port = this->Port();
		if constexpr (otyperOption == PushPull)
			port->OTYPER &= ~OTYPER_BITS[Derived::pin];
		else if constexpr (otyperOption == OpenDrain)
			port->OTYPER |= OTYPER_BITS[Derived::pin];
	}

	template<OptionsMODER moderOption>
	void ConfigureMODER()
	{
		using enum OptionsMODER;
		auto port = this->Port();
		port->MODER &= ~(MODER_MASKS[Derived::pin]); //reset to 00
		if constexpr (moderOption == Input)
			return;
		else if constexpr (moderOption == Output)
			port->MODER |= MODER_MASKS_0[Derived::pin];
		else if constexpr (moderOption == Alternate)
			port->MODER |= MODER_MASKS_1[Derived::pin];
		else if constexpr (moderOption == Analog)
			port->MODER |= MODER_MASKS[Derived::pin];
	}

	template<OptionsOSPEEDR ospeedrOption>
	void ConfigureOSPEEDR()
	{
		using enum OptionsOSPEEDR;
		auto port = this->Port();
		port->OSPEEDR &= ~OSPEEDR_MASKS[Derived::pin];
		if constexpr (ospeedrOption == LowSpeed)
			return;
		else if constexpr (ospeedrOption == MediumSpeed)
			port->OSPEEDR |= OSPEEDR_MASKS_0[Derived::pin];
		else if constexpr (ospeedrOption == HighSpeed)
			port->OSPEEDR |= OSPEEDR_MASKS_1[Derived::pin];
		else if constexpr (ospeedrOption == VeryHighSpeed)
			port->OSPEEDR |= OSPEEDR_MASKS[Derived::pin];
	}

	//Datasheet Pinouts and pin description
	//AF2 Datasheet Alternate functino 0 - 7 / 8 -15
	//RM 8.5.10 GPIO alternate function low/high register
	void ConfigureAlternateFunction(const AlternateFunction af)
	{
		uint8_t LowOrHigh = Derived::pin <= 7 ? 0 : 1;
		Port()->AFR[LowOrHigh] &= ~GPIO_AFR_AFSEL_MASKS[Derived::pin][4]; //clearing bits

		using enum AlternateFunction;
		switch (af)
		{
		case AF0:
			break; //0000 (reset value)
		case AF1:
			Port()->AFR[LowOrHigh] |= GPIO_AFR_AFSEL_MASKS[Derived::pin][0]; //0001
			break;
		case AF2:
			Port()->AFR[LowOrHigh] |= GPIO_AFR_AFSEL_MASKS[Derived::pin][1]; //0010
			break;
		case AF3:
			Port()->AFR[LowOrHigh] |= GPIO_AFR_AFSEL_MASKS[Derived::pin][0]; //0011
			Port()->AFR[LowOrHigh] |= GPIO_AFR_AFSEL_MASKS[Derived::pin][1];
			break;
		case AF4:
			Port()->AFR[LowOrHigh] |= GPIO_AFR_AFSEL_MASKS[Derived::pin][2]; //0100
			break;
		case AF5:
			Port()->AFR[LowOrHigh] |= GPIO_AFR_AFSEL_MASKS[Derived::pin][0]; //0101
			Port()->AFR[LowOrHigh] |= GPIO_AFR_AFSEL_MASKS[Derived::pin][2];
			break;
		case AF6:
			Port()->AFR[LowOrHigh] |= GPIO_AFR_AFSEL_MASKS[Derived::pin][1]; //0110
			Port()->AFR[LowOrHigh] |= GPIO_AFR_AFSEL_MASKS[Derived::pin][2];
			break;
		case AF7:
			Port()->AFR[LowOrHigh] |= GPIO_AFR_AFSEL_MASKS[Derived::pin][4]; //0111
			Port()->AFR[LowOrHigh] &= ~GPIO_AFR_AFSEL_MASKS[Derived::pin][3];
			break;
		case AF8:
			Port()->AFR[LowOrHigh] |= GPIO_AFR_AFSEL_MASKS[Derived::pin][3]; //1000
			break;
		case AF9:
			Port()->AFR[LowOrHigh] |= GPIO_AFR_AFSEL_MASKS[Derived::pin][0]; //1001
			Port()->AFR[LowOrHigh] |= GPIO_AFR_AFSEL_MASKS[Derived::pin][3];
			break;
		case AF10:
			Port()->AFR[LowOrHigh] |= GPIO_AFR_AFSEL_MASKS[Derived::pin][1]; //1010
			Port()->AFR[LowOrHigh] |= GPIO_AFR_AFSEL_MASKS[Derived::pin][3];
			break;
		case AF11:
			Port()->AFR[LowOrHigh] |= GPIO_AFR_AFSEL_MASKS[Derived::pin][4]; //1011
			Port()->AFR[LowOrHigh] &= ~GPIO_AFR_AFSEL_MASKS[Derived::pin][2];
			break;
		case AF12:
			Port()->AFR[LowOrHigh] |= GPIO_AFR_AFSEL_MASKS[Derived::pin][2]; //1100
			Port()->AFR[LowOrHigh] |= GPIO_AFR_AFSEL_MASKS[Derived::pin][3];
			break;
		case AF13:
			Port()->AFR[LowOrHigh] |= GPIO_AFR_AFSEL_MASKS[Derived::pin][4]; //1101
			Port()->AFR[LowOrHigh] &= ~GPIO_AFR_AFSEL_MASKS[Derived::pin][1];
			break;
		case AF14:
			Port()->AFR[LowOrHigh] |= GPIO_AFR_AFSEL_MASKS[Derived::pin][4]; //1110
			Port()->AFR[LowOrHigh] &= ~GPIO_AFR_AFSEL_MASKS[Derived::pin][0];
			break;
		case AF15:
			Port()->AFR[LowOrHigh] |= GPIO_AFR_AFSEL_MASKS[Derived::pin][4]; //1111
			break;
		default:
			break;
		}
	}
	//GPIOA->AFR[0] |= GPIO_AFRL_AFSEL6_1; //to be refactored


};

template<std::uintptr_t portAddr_
        , uint8_t pin_
		, OptionsOTYPER otyperOption = OptionsOTYPER::PushPull
		, OptionsOSPEEDR ospeedrOption = OptionsOSPEEDR::LowSpeed
		, OptionsPUPDR pupdrOption = OptionsPUPDR::None>
class GpioOutput : public IGpio<GpioOutput<portAddr_, pin_, otyperOption, ospeedrOption, pupdrOption>>
{
private:
	void ConfigureAsOutput()
	{
		static_assert(pin >= 0 && pin < 16, "Invalid pin number: needs to be in range of 0 - 15!");
		this->template ConfigureMODER<OptionsMODER::Output>();
		this->template ConfigureOTYPER<otyperOption>();
		this->template ConfigureOSPEEDR<ospeedrOption>();
		this->template ConfigurePUPDR<pupdrOption>();
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

template<std::uintptr_t portAddr_
        , uint8_t pin_
		, OptionsOTYPER otyperOption = OptionsOTYPER::PushPull
		, OptionsOSPEEDR ospeedrOption = OptionsOSPEEDR::LowSpeed
		, OptionsPUPDR pupdrOption = OptionsPUPDR::None>
class GpioInput : public IGpio<GpioInput<portAddr_, pin_, otyperOption, ospeedrOption, pupdrOption>>
{
protected:
	void ConfigureAsInput()
	{
		static_assert(pin >= 0 && pin < 16, "Invalid pin number: needs to be in range of 0 - 15!");
		this->template ConfigureMODER<OptionsMODER::Input>();
		this->template ConfigureOTYPER<otyperOption>();
		this->template ConfigureOSPEEDR<ospeedrOption>();
		this->template ConfigurePUPDR<pupdrOption>();
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
