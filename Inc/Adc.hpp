/*
 * Adc.hpp
 *
 *  Created on: Jun 16, 2025
 *      Author: bartoszlozinski
 */
#pragma once
#ifndef ADC_HPP_
#define ADC_HPP_

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


//right now only for adc1
template<std::uintptr_t adcAddr_>
class Adc
{
private:
	static constexpr std::uintptr_t adcAddr = adcAddr_;

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
		ADC()->CR &= ~ADC_CR_DEEPPWD; //Deep Power Down enable (0 - ADC not in Deep-power down, 1 - in DPD)
		ADC()->CR |= ADC_CR_ADVREGEN; //Voltage regulator enabled
	}

	void SetResolution()
	{
		//RM 18.7.4 Configuration register - RES (Data resolution)
		ADC()->CFGR &= ~ADC_CFGR_RES_Msk; //00 - 12-bit (reset state); 01 - 10-bit; 10 - 8-bit; 11 - 6-bit
	}

	void Calibrate()
	{
		//Auto-calibration of ADC
		//RM 18.7.3 ADC Control register (CR)
		auto adc = ADC();
		adc->CR |= ADC_CR_ADCAL;
		while (adc->CR & ADC_CR_ADCAL) {}; //need to wait until calibration is done
		adc->CR |= ADC_CR_ADEN; //ADC enable
		while (!(adc->ISR & ADC_ISR_ADRDY)) {}
	}



public:
	ADC_TypeDef* ADC() const { return reinterpret_cast<ADC_TypeDef*>(this->adcAddr); }
	Adc(const Adc& source) = delete;
	Adc(Adc&& source) = delete;
	Adc& operator=(const Adc& source) = delete;
	Adc& operator=(Adc&& source) = delete;
	Adc()
	{
		ClockEnable();
		RegulateVoltage();
		SetResolution();
		Calibrate();
	}

	void ChannelInit()
	{
		//Sequence for channel 1 in sequence 1
		//ADC1->SQR1 &= ~ADC_SQR1_SQ1_Msk;
		//ADC1->SQR1 |= ADC_SQR1_SQ1_0; //ADC123_IN1 for PC0 sequence 1 for channel 1 (and value 1)
		ADC1->SQR1 = ADC_SQR1_SQ1_0;
		ADC1->SQR1 &= ~ADC_SQR1_L;
		//Sampling time (for how many ADC cycles measurement is being done)
		ADC1->SMPR1 &= ~ADC_SMPR1_SMP1_Msk;
		ADC1->SMPR1 |= (0b111 << ADC_SMPR1_SMP1_Pos); //RM 18.7.7 SMPx, 640.5 ADC cycles
	}

	void StartConversion()
	{
		auto adc = ADC();
		//RM 18.7.3 Control register
		adc->CR |= ADC_CR_ADSTART;
		//while (adc->CR & ADC_CR_ADSTART) {}; //wait until it's done
		while (!(adc->ISR & ADC_ISR_EOC)) {}; // wait until end of conversion
		//RM 18.7.1 Interrupt and status register (ISR)
		//EOC End of conversion flag - 0 - conversion not completed; 1 - regular channel conversion complete
		//cleared by writing 1 manually or reading ADC_DR
	}

	uint32_t ReadData() { return ADC()->DR; }

};


void ADCConfig()
{
	//RM 6.4.17 AHB peripheral clock enable register (AHB2ENR)
	RCC->AHB2ENR |= RCC_AHB2ENR_ADCEN;
	RCC->CCIPR |= RCC_CCIPR_ADCSEL_0 | RCC_CCIPR_ADCSEL_1; //system clock RM 6.4.28 Peripherals independent clock configuration register


	//Sequence for channel 1 in sequence 1
	//ADC1->SQR1 &= ~ADC_SQR1_SQ1_Msk;
	//ADC1->SQR1 |= ADC_SQR1_SQ1_0; //ADC123_IN1 for PC0 sequence 1 for channel 1 (and value 1)
	ADC1->SQR1 = ADC_SQR1_SQ1_0;
	ADC1->SQR1 &= ~ADC_SQR1_L;

	//proper voltage regulator enable:
	//RM 18.7.3 ADC volate regulator enable (ADVREGEN)
	//Check which ADC you're using In Datasheet pinouts and pin description
	//I.e PA0 ADC12_IN5 means it's available for ADC1 and ADC2, channel 5
	ADC1->CR &= ~ADC_CR_DEEPPWD;
	ADC1->CR |= ADC_CR_ADVREGEN; //ADC1

	//Sampling time (for how many ADC cycles measurement is being done)
	ADC1->SMPR1 &= ~ADC_SMPR1_SMP1_Msk;
	ADC1->SMPR1 |= (0b111 << ADC_SMPR1_SMP1_Pos); //RM 18.7.7 SMPx, 640.5 ADC cycles

	ADC1->CFGR &= ~ADC_CFGR_RES_Msk; //by default 12 bit resolution

	//Auto-calibration of ADC
	//RM 18.7.3 ADC Control rgister (CR)
	ADC1->CR |= ADC_CR_ADCAL;
	while (ADC1->CR & ADC_CR_ADCAL) {}; //need to wait until calibration is done
	ADC1->CR |= ADC_CR_ADEN;
	while (!(ADC1->ISR & ADC_ISR_ADRDY)) {}
}

//ADC12_IN5 for PA0
void ADCInputGPIOConfigure()
{
	//Enable clock for PC0
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;

	GPIOC->MODER &= ~GPIO_MODER_MODE0; // Clear mode bits
	GPIOC->MODER |= GPIO_MODER_MODE0; //analog (by default anyway)
	GPIOC->AFR[0] &= ~GPIO_AFRL_AFSEL0; // Clear AF
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPD0);
	GPIOC->ASCR |= GPIO_ASCR_ASC0; //RM 8.5.12 GPIO port analog switch control register: 0 - disconnect analog switch to the adc input (reset state), 1 - connect
}

void ADCConversion()
{
	//RM 18.7.3 Control register
	ADC1->CR |= ADC_CR_ADSTART;
	while (ADC1->CR & ADC_CR_ADSTART) {}; //wait until it's done :(
	//ADC1->CR |= ADC_CR_ADSTART;

    //ADC1->ISR |= ADC_ISR_EOC; // clear EOC flag

	while (!(ADC1->ISR & ADC_ISR_EOC)) {}; // wait until end of conversion

	//RM 18.7.1 Interrupt and status register (ISR)
	//EOC End of conversion flag - 0 - conversion not completed; 1 - regular channel conversion complete
	//cleared by writing 1 manually or reading ADC_DR
}

uint32_t ADCReadData()
{
	return ADC1->DR;
}


#endif /* ADC_HPP_ */
