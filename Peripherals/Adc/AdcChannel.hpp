#pragma once
#include "Adc.hpp"
#include "../Gpio/GpioAnalog.hpp"
#include "InterfaceAdcChannel.hpp"

template<GpioPort Port, AdcConcept ADC, uint8_t pin_, uint8_t channel_>
class AdcChannel : public InterfaceAdcChannel<AdcChannel<Port, ADC, pin_, channel_>>
{
protected:
	friend InterfaceAdcChannel<AdcChannel<Port, ADC, pin_, channel_>>;
	volatile ADC* const adc = nullptr;
	static constexpr uint8_t channel = channel_;
	GpioAnalog<Port, pin_> gpioAnalog;

public:
	AdcChannel(const AdcChannel& source) = delete;
	AdcChannel(AdcChannel&& source) = delete;
	AdcChannel& operator=(const AdcChannel& source) = delete;
	AdcChannel& operator=(AdcChannel&& source) = delete;
	AdcChannel(volatile ADC* const adc_, Port* const port_, const uint8_t sequence, const SamplingTime samplingTime = SamplingTime::Cycles_640_5)
		: adc(adc_)
		, gpioAnalog(port_)
	{
		this->ConfigureSequence(sequence);
		this->ConfigureSamplingTime(samplingTime);
	}

	volatile uint16_t value = 0;
	void Read() { value = adc->DR; }
};
