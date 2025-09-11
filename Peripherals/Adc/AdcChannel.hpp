
#pragma once
#include "Adc.hpp"

template<GpioPort Port, AdcConcept ADC, uint8_t pin_, uint8_t channel_>
class AdcChannel : public IGpio<AdcChannel<Port, ADC, pin_, channel_>>
{
protected:
	volatile ADC* const adc = nullptr;
	static constexpr uint8_t channel = channel_;
	volatile uint32_t value = 0;

	void ConfigureGPIO()
	{
		static_assert(pin >= 0 && pin <= 15, "Invalid pin number: needs to be in range of 0 - 15!");
		this-> template ConfigureMODER<OptionsMODER::Analog>();
		this-> template ConfigureOSPEEDR<OptionsOSPEEDR::LowSpeed>();
		this-> template  ConfigurePUPDR<OptionsPUPDR::None>();
		port->ASCR |= GPIO_ASCR_ASC[pin];
	}

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
		else if (sequence >= 11 && sequence <= 16)
			adc->SQR4 |= (channel << ADC_SQR_SQ[sequence - 1]);
	}

public:
	volatile Port* const port = nullptr;
	static constexpr uint8_t pin = pin_;

	AdcChannel(const AdcChannel& source) = delete;
	AdcChannel(AdcChannel&& source) = delete;
	AdcChannel& operator=(const AdcChannel& source) = delete;
	AdcChannel& operator=(AdcChannel&& source) = delete;
	AdcChannel(volatile ADC* const adc_, Port* const port_, const uint8_t sequence, const SamplingTime samplingTime = SamplingTime::Cycles_640_5)
		: adc(adc_)
		, port(port_)
	{
		this->EnableClock();
		ConfigureGPIO();
		ConfigureSequence(sequence);
		ConfigureSamplingTime(samplingTime);
	}

	void ConfigureSamplingTime(const SamplingTime samplingTime)
	{
		auto& SMPRx = channel < 10 ? adc->SMPR1 : adc->SMPR2;
		SMPRx |= (static_cast<uint16_t>(samplingTime) << ADC_SMPR_SMP[channel - 1]);
	}

	uint32_t Get() { return this->value; }
	void Read() { value = adc->DR; }
};