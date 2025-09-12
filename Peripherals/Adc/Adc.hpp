#pragma once

#include "../../Inc/Config.hpp"
#include "../Dma/DmaChannel.hpp"
#include <cassert>


template<typename T>
concept AdcConcept = requires(T adc)
{
	{ adc.ISR } -> std::convertible_to<volatile uint32_t&>;
	{ adc.IER } -> std::convertible_to<volatile uint32_t&>;
	{ adc.CR } -> std::convertible_to<volatile uint32_t&>;
	{ adc.CFGR }-> std::convertible_to<volatile uint32_t&>;
	{ adc.CFGR2 } -> std::convertible_to<volatile uint32_t&>;
	{ adc.SMPR1 } -> std::convertible_to<volatile uint32_t&>;
	{ adc.SMPR2 } -> std::convertible_to<volatile uint32_t&>;
	{ adc.TR1 }-> std::convertible_to<volatile uint32_t&>;
	{ adc.TR2 } -> std::convertible_to<volatile uint32_t&>;
	{ adc.TR3 } -> std::convertible_to<volatile uint32_t&>;
	{ adc.SQR1 } -> std::convertible_to<volatile uint32_t&>;
	{ adc.SQR2 } -> std::convertible_to<volatile uint32_t&>;
	{ adc.SQR3 } -> std::convertible_to<volatile uint32_t&>;
	{ adc.SQR4 } -> std::convertible_to<volatile uint32_t&>;
	{ adc.DR } -> std::convertible_to<volatile uint32_t&>;
	{ adc.JSQR } -> std::convertible_to<volatile uint32_t&>;
	{ adc.OFR1 } -> std::convertible_to<volatile uint32_t&>;
	{ adc.OFR2 } -> std::convertible_to<volatile uint32_t&>;
	{ adc.OFR3 } -> std::convertible_to<volatile uint32_t&>;
	{ adc.OFR4 } -> std::convertible_to<volatile uint32_t&>;
	{ adc.JDR1 } -> std::convertible_to<volatile uint32_t&>;
	{ adc.JDR2 } -> std::convertible_to<volatile uint32_t&>;
	{ adc.JDR3 } -> std::convertible_to<volatile uint32_t&>;
	{ adc.JDR4 } -> std::convertible_to<volatile uint32_t&>;
	{ adc.AWD2CR } -> std::convertible_to<volatile uint32_t&>;
	{ adc.AWD3CR } -> std::convertible_to<volatile uint32_t&>;
	{ adc.DIFSEL } -> std::convertible_to<volatile uint32_t&>;
	{ adc.CALFACT } -> std::convertible_to<volatile uint32_t&>;
};


/*
 * adc configured with 3 channels
 * let it be
 * ch2, ch3, ch5
 * first = ch5
 * second = ch2
 * third = ch3
 * this is ortder to sample channels
 *
 *then in SQ1 we want to put 5
 *then in SQ2 we want to put 2
 *then in SQ3 we want to put 3 (if we wanted it to be channel 10, then put 10)
 *
 * */

enum class SamplingTime : uint16_t
{
	//RM 18.7.7 SMPx
	Cycles_2_5 = 0,
	Cycles_6_5 = 1,
	Cycles_12_5 = 2,
	Cycles_24_5 = 3,
	Cycles_47_5 = 4,
	Cycles_92_5 = 5,
	Cycles_247_5 = 6,
	Cycles_640_5 = 7
};

template<AdcConcept ADC, uint8_t sequenceLength_>
class Adc
{
private:
	static constexpr uint8_t sequenceLength = sequenceLength_;

	void ClockEnable()
	{
		RCC->AHB2ENR |= RCC_AHB2ENR_ADCEN; //RM 6.4.17 AHB peripheral clock enable register (AHB2ENR)
		//To be customized
		RCC->CCIPR |= RCC_CCIPR_ADCSEL; //RM 6.4.28 Clock configuration register (11 for system clock) (01 PLLSAI1, 10 PLLSAI2)
	}

	void RegulateVoltage()
	{
		//proper voltage regulator enable:
		//RM 18.7.3 ADC volate regulator enable (ADVREGEN)
		//Check which ADC you're using In Datasheet pinouts and pin description
		//I.e PA0 ADC12_IN5 means it's available for ADC1 and ADC2, channel 5
		adc->CR &= ~ADC_CR_DEEPPWD; //Deep Power Down enable (0 - ADC not in Deep-power down, 1 - in DPD)
		adc->CR |= ADC_CR_ADVREGEN; //Voltage regulator enabled
	}

	void SetResolution()
	{
		//RM 18.7.4 Configuration register - RES (Data resolution)
		adc->CFGR &= ~ADC_CFGR_RES_Msk; //00 - 12-bit (reset state); 01 - 10-bit; 10 - 8-bit; 11 - 6-bit
	}

	void Calibrate()
	{
		//Auto-calibration of ADC
		//RM 18.7.3 ADC Control register (CR)
		adc->CR |= ADC_CR_ADCAL;
		while (adc->CR & ADC_CR_ADCAL) {}; //need to wait until calibration is done
		adc->CR |= ADC_CR_ADEN; //ADC enable
		while (!(adc->ISR & ADC_ISR_ADRDY)) {}
	}

	void SetSequenceLength()
	{
		static_assert(sequenceLength >= 1 && sequenceLength <= 16, "Number of channel conversions shall be in range 1 - 16!");
		//RM 18.7.11 Reqular sequence register 1
		//0000 for 1 conversions, 0001 for 2, 0010 for 3 .... 1111 for 16 conversions
		adc->SQR1 |= ((sequenceLength - 1) << ADC_SQR1_L_Pos);
	}

public:
	volatile ADC* const adc = nullptr;

	Adc(const Adc& source) = delete;
	Adc(Adc&& source) = delete;
	Adc& operator=(const Adc& source) = delete;
	Adc& operator=(Adc&& source) = delete;
	Adc(ADC* const adc_)
		: adc(adc_)
	{
		ClockEnable();
		RegulateVoltage();
		SetResolution();
		Calibrate();
		SetSequenceLength();
	}

	void StartConversion()
	{
		//RM 18.7.3 Control register
		while (!(adc->ISR & ADC_ISR_ADRDY)) {};
		adc->CR |= ADC_CR_ADSTART;
		//Check ADSTART adn ADRDY flags
	}

	void WaitUntilEndOfConversion()
	{
		//while (adc->CR & ADC_CR_ADSTART) {}; //wait until it's done
		while (!(adc->ISR & ADC_ISR_EOC)) {}; // wait until end of conversion
		//RM 18.7.1 Interrupt and status register (ISR)
		//EOC End of conversion flag - 0 - conversion not completed; 1 - regular channel conversion complete
		//cleared by writing 1 manually or reading ADC_DR
	}

	void EnableDma(DmaChannel& dma, volatile uint16_t* buffer, std::size_t length)
	{
		dma.Configure(reinterpret_cast<uint32_t>(&adc->DR),
					  reinterpret_cast<uint32_t>(buffer),
					  length);

		//Enable ADC DMA Request
		adc->CFGR |= ADC_CFGR_DMAEN;
		adc->CFGR |= ADC_CFGR_DMACFG;//RM 18.7.5 circular mode
		adc->CFGR |= ADC_CFGR_CONT;
		dma.Enable();
	}
};
