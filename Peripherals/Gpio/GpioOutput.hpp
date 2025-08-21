#pragma once
#include "IGpio.hpp"

template<GpioPort Port
        , uint8_t pin_
		, OptionsOTYPER otyperOption = GpioDefaults::otyperOption
		, OptionsOSPEEDR ospeedrOption = GpioDefaults::ospeedrOption
		, OptionsPUPDR pupdrOption = GpioDefaults::pupdrOption>
class GpioOutput : public IGpio<GpioOutput<GPIO_TypeDef, pin_, otyperOption, ospeedrOption, pupdrOption>>
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
	volatile Port* const port = nullptr;
	static constexpr uint8_t pin = pin_;

	GpioOutput(const GpioOutput& source) = delete;
	GpioOutput(GpioOutput&& source) = delete;
	GpioOutput& operator=(const GpioOutput& source) = delete;
	GpioOutput& operator=(GpioOutput&& source) = delete;
	GpioOutput(Port* const port_)
		: port(port_)
	{
		this->EnableClock();
		ConfigureAsOutput();
	}
	~GpioOutput() = default;

	//Output data register ODR - reference manual 8.5.6
	bool IsPinSet() const { return port->ODR & PinMask<pin>(); }
	void Set() { port->BSRR |= (0b1 << pin); }
	void Clear()
	{
		static constexpr uint8_t bitShift = 16 + pin; //bit resets are 16-32
		port->BSRR |= (0b1 << bitShift);
	}
	void Toggle() { port->ODR ^= (0b1 << pin); /*Bitwise XOR*/} //ODR has 16 bits
};