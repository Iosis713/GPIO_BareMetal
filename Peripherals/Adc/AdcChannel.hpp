#pragma once
#include "Adc.hpp"
#include "../Gpio/GpioAnalog.hpp"

template<GpioPort Port, AdcConcept ADC, uint8_t pin_, uint8_t channel_>
class AdcChannel
{
protected:
	volatile ADC* const adc = nullptr;
	static constexpr uint8_t channel = channel_;
	GpioAnalog<Port, pin_> gpioAnalog;

	void ConfigureSequence(const uint8_t sequence)
	{
		static_assert(channel >= 1 && channel <= 16, "Channel number shall be in range of 1 - 16!");
		//ADC123_IN1 for PC0 sequence 1 for channel 1 (and value 1), but SQ1 with value 3 for channel 3
		if (sequence <= 4 )
			adc->SQR1 |= (channel << ADC_SQR_SQ[sequence - 1]);
		else if (sequence >= 5 && sequence <= 9)
			adc->SQR2 |= (channel << ADC_SQR_SQ[sequence - 1]);
		else if (sequence >= 10 && sequence <= 14)
			adc->SQR3 |= (channel << ADC_SQR_SQ[sequence - 1]);
		else if (sequence >= 15 && sequence <= 16)
			adc->SQR4 |= (channel << ADC_SQR_SQ[sequence - 1]);
	}

	void ConfigureSamplingTime(const SamplingTime samplingTime)
	{
		auto& SMPRx = channel < 10 ? adc->SMPR1 : adc->SMPR2;
		SMPRx |= (static_cast<uint16_t>(samplingTime) << ADC_SMPR_SMP[channel - 1]);
	}

public:
	AdcChannel(const AdcChannel& source) = delete;
	AdcChannel(AdcChannel&& source) = delete;
	AdcChannel& operator=(const AdcChannel& source) = delete;
	AdcChannel& operator=(AdcChannel&& source) = delete;
	AdcChannel(volatile ADC* const adc_, Port* const port_, const uint8_t sequence, const SamplingTime samplingTime = SamplingTime::Cycles_640_5)
		: adc(adc_)
		, gpioAnalog(port_)
	{
		ConfigureSequence(sequence);
		ConfigureSamplingTime(samplingTime);
	}

	volatile uint16_t value = 0;
	void Read() { value = adc->DR; }
};
