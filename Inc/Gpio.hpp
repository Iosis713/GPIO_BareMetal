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
#include <concepts>

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

enum class Trigger
{
	Rising,
	Falling
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
};

template<std::uintptr_t portAddr_
        , uint8_t pin_
		, OptionsOTYPER otyperOption = OptionsOTYPER::PushPull
		, OptionsOSPEEDR ospeedrOption = OptionsOSPEEDR::HighSpeed
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

//_________________________INPUT___________________________________//


template<std::uintptr_t portAddr_
        , uint8_t pin_
		, OptionsOTYPER otyperOption = OptionsOTYPER::PushPull
		, OptionsOSPEEDR ospeedrOption = OptionsOSPEEDR::HighSpeed
		, OptionsPUPDR pupdrOption = OptionsPUPDR::None>
class GpioInput : public IGpio<GpioInput<portAddr_, pin_, otyperOption, ospeedrOption, pupdrOption>>
{
protected:
	volatile bool interruptOccured = false;

	void ConfigureAsInput()
	{
		static_assert(pin >= 0 && pin < 16, "Invalid pin number: needs to be in range of 0 - 15!");
		this->template ConfigureMODER<OptionsMODER::Input>();
		this->template ConfigureOTYPER<otyperOption>();
		this->template ConfigureOSPEEDR<ospeedrOption>();
		this->template ConfigurePUPDR<pupdrOption>();
	}
	void ConfigureEXTICR()
	{
		//RM 9.2.3 - 9.2.6 - SYSCFG External interrupt configuration register X
		//EXTI enable SYSCFG_EXTICR* bit)
		//EXTICR1 (index 0): pins 0 - 3, EXTICR2 (1): pins 4 - 7; EXTICR3(2): pins 8- 11; EXTICR4(3): pins 12 - 15
		constexpr uint8_t extiCrIndex = pin / 4;
		SYSCFG->EXTICR[extiCrIndex] &= ~SYSCFG_EXTI[pin]; //reset to 0000 (defualt value);
		if constexpr (this->portAddr == GPIOA_BASE)
			SYSCFG->EXTICR[extiCrIndex] |= SYSCFG_EXTI_PA[pin]; //example for pin PC13 = GPIOC_BASE, pin 13 (index is proper, as pins are 0 - 15)
		else if constexpr (this->portAddr == GPIOB_BASE)
			SYSCFG->EXTICR[extiCrIndex] |= SYSCFG_EXTI_PB[pin];
		else if constexpr (this->portAddr == GPIOC_BASE)
			SYSCFG->EXTICR[extiCrIndex] |= SYSCFG_EXTI_PC[pin];
		else if constexpr (this->portAddr == GPIOD_BASE)
			SYSCFG->EXTICR[extiCrIndex] |= SYSCFG_EXTI_PD[pin];
		else if constexpr (this->portAddr == GPIOE_BASE)
			SYSCFG->EXTICR[extiCrIndex] |= SYSCFG_EXTI_PE[pin];
		else if constexpr (this->portAddr == GPIOF_BASE)
			SYSCFG->EXTICR[extiCrIndex] |= SYSCFG_EXTI_PF[pin];
		else if constexpr (this->portAddr == GPIOG_BASE)
			SYSCFG->EXTICR[extiCrIndex] |= SYSCFG_EXTI_PG[pin];
	}

	//interrupt priority; enum from stm32l476xx.h (CMSIS file) - Interrupt number definition
	template <uint8_t priority>
	void NvicExtiPriorityConfigurator(const IRQn_Type irqnType)
	{
		static_assert(priority <= 15, "EXTI priority should be in range: 0 - 15!");
		NVIC_SetPriority(irqnType, priority); //set priority 0, priotity (0 - 15)
		NVIC_EnableIRQ(irqnType); //enable interrupt
	}

	template<uint8_t priority>
	void ConfigureExtiPriority()
	{
		if constexpr (pin == 0)
			NvicExtiPriorityConfigurator<priority>(EXTI0_IRQn);
		else if constexpr (pin == 1)
			NvicExtiPriorityConfigurator<priority>(EXTI1_IRQn);
		else if constexpr (pin == 2)
			NvicExtiPriorityConfigurator<priority>(EXTI2_IRQn);
		else if constexpr (pin == 3)
			NvicExtiPriorityConfigurator<priority>(EXTI3_IRQn);
		else if constexpr (pin == 4)
			NvicExtiPriorityConfigurator<priority>(EXTI4_IRQn);
		else if constexpr (pin >= 5 && pin <= 9)
			NvicExtiPriorityConfigurator<priority>(EXTI9_5_IRQn);
		else if constexpr (pin >= 10 && pin <= 15)
			NvicExtiPriorityConfigurator<priority>(EXTI15_10_IRQn);
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

	//RM External iterrupt/event (EXTI) GPIO Mapping (multiplexer)
	//For single exti only single port
	//I.e interrupt for PC13 cannot be set for PA13 at the same time
	template<uint8_t priority>
	void ConfigureEXTI(const Trigger trigger)
	{
		//RM 9 System configuration controller SYSCFG:
		//Manages among the others external interrupt line connection to GPIOs (EXTI)
		//RM 6.4.21 - (APB2 peripheral clock enable register [lookup for SYSCFG bit]
		RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
		ConfigureEXTICR();

		//reference manual 14.5.3 Rising trigger selection register
		//from high to low -- falling; from low to high -- rising
		if (trigger == Trigger::Rising)
			EXTI->RTSR1 |= EXTI_RTSR1_RT[pin];
		else
			EXTI->FTSR1 |= EXTI_FTSR1_FT[pin];

		//Interrupt mask register IMR - unmasked
		EXTI->IMR1 |= EXTI_IMR1[pin];

		ConfigureExtiPriority<priority>();
	}

	bool ReadPin() const {return this->Port()->IDR & PinMask<pin>();} //true - high state, false - low state
	void ClearInterruptFlag() { this->interruptOccured = false; };
	void IrqHandler()
	{
		if (EXTI->PR1 & EXTI_PR1_PIF[pin])
		{
			EXTI->PR1 |= EXTI_PR1_PIF[pin]; //PIFx --> rc_w1 - read/clear with 1 - used 1 to read and clear interrupt state

			interruptOccured = true;
		}
	}
	bool InterruptOccured() const { return this->interruptOccured; }

};

template<std::uintptr_t portAddr_
        , uint8_t pin_
		, AlternateFunction alternateFunction
		, OptionsOTYPER otyperOption = OptionsOTYPER::PushPull
		, OptionsOSPEEDR ospeedrOption = OptionsOSPEEDR::HighSpeed
		, OptionsPUPDR pupdrOption = OptionsPUPDR::None>
class GpioAlternate : public IGpio<GpioAlternate<portAddr_, pin_, alternateFunction, otyperOption, ospeedrOption, pupdrOption>>
{
protected:


public:
	static constexpr std::uintptr_t portAddr = portAddr_;
	static constexpr uint8_t pin = pin_;
	GpioAlternate(const GpioAlternate& source) = delete;
	GpioAlternate(GpioAlternate&& source) = delete;
	GpioAlternate& operator=(const GpioAlternate& source) = delete;
	GpioAlternate& operator=(GpioAlternate&& source) = delete;
	GpioAlternate()
	{
		this->EnableClock();
		this->template ConfigureMODER<OptionsMODER::Alternate>();
		this->template ConfigureOTYPER<otyperOption>();
		this->template ConfigureOSPEEDR<ospeedrOption>();
		this->template ConfigurePUPDR<pupdrOption>();
		this->template ConfigureAlternateFunction(alternateFunction);
	}

};

#endif /* INC_GPIO_HPP_ */
