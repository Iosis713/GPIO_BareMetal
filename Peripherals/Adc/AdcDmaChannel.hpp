#pragma once
#include "InterfaceAdcChannel.hpp"
#include "Adc.hpp"
#include "../Dma/DmaChannel.hpp"
#include "../Gpio/GpioAnalog.hpp"

template<GpioPort Port, AdcConcept ADC, uint8_t pin_, uint8_t channel_>
class AdcDmaChannel : public InterfaceAdcChannel<AdcDmaChannel<Port, ADC, pin_, channel_>>
{
protected:
    friend InterfaceAdcChannel<AdcDmaChannel<Port, ADC, pin_, channel_>>;
    volatile ADC* const adc = nullptr;
	static constexpr uint8_t channel = channel_;
    GpioAnalog<Port, pin_> gpioAnalog;
    
public:
    DmaChannel dmaChannel;
	volatile uint16_t value = 0;

    AdcDmaChannel(const AdcDmaChannel& source) = delete;
	AdcDmaChannel(AdcDmaChannel&& source) = delete;
	AdcDmaChannel& operator=(const AdcDmaChannel& source) = delete;
	AdcDmaChannel& operator=(AdcDmaChannel&& source) = delete;
	AdcDmaChannel(volatile ADC* const adc_, Port* const port_, volatile DMA_Channel_TypeDef* const dmaChannel_, const uint8_t sequence, const SamplingTime samplingTime = SamplingTime::Cycles_640_5)
		: adc(adc_)
		, gpioAnalog(port_)
        , dmaChannel(dmaChannel_)
	{
		this->ConfigureSequence(sequence);
		this->ConfigureSamplingTime(samplingTime);
	}

    //uint16_t Get() { return this->value; }
};
