#pragma once

template<typename Derived>
class InterfaceAdcChannel
{
protected:
	void ConfigureSequence(const uint8_t sequence)
	{
		static_assert(Derived::channel >= 1 && Derived::channel <= 16, "Channel number shall be in range of 1 - 16!");
		//ADC123_IN1 for PC0 sequence 1 for channel 1 (and value 1), but SQ1 with value 3 for channel 3
		if (sequence <= 4 )
			static_cast<Derived*>(this)->adc->SQR1 |= (Derived::channel << ADC_SQR_SQ[sequence - 1]);
		else if (sequence >= 5 && sequence <= 9)
			static_cast<Derived*>(this)->adc->SQR2 |= (Derived::channel << ADC_SQR_SQ[sequence - 1]);
		else if (sequence >= 10 && sequence <= 14)
			static_cast<Derived*>(this)->adc->SQR3 |= (Derived::channel << ADC_SQR_SQ[sequence - 1]);
		else if (sequence >= 15 && sequence <= 16)
			static_cast<Derived*>(this)->adc->SQR4 |= (Derived::channel << ADC_SQR_SQ[sequence - 1]);
	}

	void ConfigureSamplingTime(const SamplingTime samplingTime)
	{
		auto& SMPRx = Derived::channel < 10 ? static_cast<Derived*>(this)->adc->SMPR1 : static_cast<Derived*>(this)->adc->SMPR2;
		SMPRx |= (static_cast<uint16_t>(samplingTime) << ADC_SMPR_SMP[Derived::channel - 1]);
	}
};
