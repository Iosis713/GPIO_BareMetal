#pragma once
#include "IGpio.hpp"

template<std::uintptr_t portAddr_
        , uint8_t pin_
		, OptionsOTYPER otyperOption = GpioDefaults::otyperOption
		, OptionsOSPEEDR ospeedrOption = GpioDefaults::ospeedrOption
		, OptionsPUPDR pupdrOption = GpioDefaults::pupdrOption>
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