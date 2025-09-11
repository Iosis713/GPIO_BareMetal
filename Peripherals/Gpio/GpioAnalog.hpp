#pragma once
#include "IGpio.hpp"

template<GpioPort Port
        , uint8_t pin_>
class GpioAnalog : public IGpio<GpioAnalog<GPIO_TypeDef, pin_>>
{
protected:
    void ConfigureAsAnalog()
    {
        static_assert(pin >= 0 && pin <= 15, "Invalid pin number: needs to be in range of 0 - 15!");
		this->EnableClock();
		this-> template ConfigureMODER<OptionsMODER::Analog>();
		this-> template ConfigureOSPEEDR<OptionsOSPEEDR::LowSpeed>();
		this-> template  ConfigurePUPDR<OptionsPUPDR::None>();
		port->ASCR |= GPIO_ASCR_ASC[pin];
    }

public:
	volatile Port* const port = nullptr;
	static constexpr uint8_t pin = pin_;
	GpioAnalog(const GpioAnalog& source) = default;
	GpioAnalog(GpioAnalog&& source) = default;
	GpioAnalog& operator=(const GpioAnalog& source) = delete;
	GpioAnalog& operator=(GpioAnalog&& source) = delete;
	GpioAnalog(Port* const port_)
		: port(port_)
	{
        ConfigureAsAnalog();
	}
};