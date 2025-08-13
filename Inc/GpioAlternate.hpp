#pragma once
#include "IGpio.hpp"

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
	GpioAlternate(const GpioAlternate& source) = default;
	GpioAlternate(GpioAlternate&& source) = default;
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