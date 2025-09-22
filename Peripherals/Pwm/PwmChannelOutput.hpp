#pragma once
#include "../Gpio/IGpio.hpp"

template<GpioPort Port
        , uint8_t pin_
		, uint8_t channel_
		, OptionsOTYPER otyperOption = OptionsOTYPER::PushPull
		, OptionsOSPEEDR ospeedrOption = OptionsOSPEEDR::MediumSpeed
		, OptionsPUPDR pupdrOption = OptionsPUPDR::None>
class PWMChannelOutput : public IGpio<PWMChannelOutput<Port, pin_, channel_, otyperOption, ospeedrOption, pupdrOption>>
{
private:
	static constexpr uint8_t channel = channel_;
	volatile TIM_TypeDef* const timer = nullptr;

	void ConfigureChannel(const AlternateFunction af, const PWMPolarity polarity)
	{
		static_assert(pin >= 0 && pin <= 15, "Invalid pin number: needs to be in range of 0 - 15!");
		static_assert(channel >= 1 && channel <= 4, "Invalid channel number: needs to be in range of 1 - 4!");

		this->template ConfigureMODER<OptionsMODER::Alternate>();
		this->template ConfigureOTYPER<otyperOption>();
		this->template ConfigureOSPEEDR<ospeedrOption>();
		this->template ConfigurePUPDR<pupdrOption>();
		ConfigureCaptureCompare();
		ConfigurePolarity(polarity);
		//Datasheet Pinouts and pin description
		//AF2 Datasheet Alternate functino 0 - 7
		//RM 8.5.10 GPIO alternate function low register (AFSEL6) because pin PA6
		//To be checked in datasheet Pinouts and pin description / alternate function
		//It need to respect for Alternate function at Datasheet
		this->template ConfigureAlternateFunction(af);
	}

	void ConfigureCaptureCompare()
	{
		//RM 31.4.8 TIMx capture/compare mode register 1/2 (TIMx_CCMR1) [alternate] - output
		//Capture/Compare 1 Selection -CC1S - is output (0) by default, but check this bit
		//Output Compare 1 Mode - OC1M - PWM mode 1 -active when CNT < CCR1
		if constexpr (channel == 1)
		{
			timer->CCMR1 |= TIM_CCMR1_OC1M_1; //0110  for PWM mode 1
			timer->CCMR1 |= TIM_CCMR1_OC1M_2;
			// Enable preload for CCR1 (this is **required** for PWM to work)
			timer->CCMR1 |= TIM_CCMR1_OC1PE;
			timer->CCR1 = 0;// just for initialization clearing the counter
			//capture/compare output enable bit
			timer->CCER |= TIM_CCER_CC1E; //Capture/compare enable register (TIMx_CCER1) RM 31.4.11

			//RM 31.4.4 DMA/Interrupt enable register (DIER)
			//Trigger interrupt enable TIE (on/off - 1/0)
			timer->DIER |= TIM_DIER_CC1IE; //capture/compare 1 interrupt 1 (CH1) enable
		}
		else if constexpr (channel == 2)
		{
			timer->CCMR1 |= TIM_CCMR1_OC2M_1; //refer to OM1M
			timer->CCMR1 |= TIM_CCMR1_OC2M_2;
			timer->CCMR1 |= TIM_CCMR1_OC2PE;
			timer->CCR2 = 0;
			timer->CCER |= TIM_CCER_CC2E;
			timer->DIER |= TIM_DIER_CC2IE;
		}
		else if constexpr (channel == 3)
		{
			timer->CCMR2 |= TIM_CCMR2_OC3M_1; //refer to OM1M
			timer->CCMR2 |= TIM_CCMR2_OC3M_2;
			timer->CCMR2 |= TIM_CCMR2_OC3PE;
			timer->CCR3 = 0;
			timer->CCER |= TIM_CCER_CC3E;
			timer->DIER |= TIM_DIER_CC3IE;

		}
		else if constexpr (channel == 4)
		{
			timer->CCMR2 |= TIM_CCMR2_OC4M_1; //refer to OM1M
			timer->CCMR2 |= TIM_CCMR2_OC4M_2;
			timer->CCMR2 |= TIM_CCMR2_OC4PE;
			timer->CCR4 = 0;
			timer->CCER |= TIM_CCER_CC4E;
			timer->DIER |= TIM_DIER_CC4IE;
		}
	}

public:
	volatile Port* const port = nullptr;
	static constexpr uint8_t pin = pin_;
	PWMChannelOutput(const PWMChannelOutput& source) = delete;
	PWMChannelOutput(PWMChannelOutput&& source) = delete;
	PWMChannelOutput& operator=(const PWMChannelOutput& source) = delete;
	PWMChannelOutput& operator=(PWMChannelOutput&& source) = delete;
	PWMChannelOutput() = delete;
	PWMChannelOutput(TIM_TypeDef* const timer_, Port* const port_, const AlternateFunction af, const PWMPolarity polarity = PWMPolarity::ActiveHigh)
		: timer(timer_)
		, port(port_)
	{
		this->EnableClock();
		ConfigureChannel(af, polarity);
	}

	//CCR1 - capture compare register 1 (CH1)
	//RM 31.4.16 Capture/compare register 1
	//Capture compare register (to which value counter will be compared to set high state
	//its pulse value in other words: 0% --> CCR1 = 0, 100% --> CCR1 = ARR
	uint32_t GetPulse() const
	{
		if constexpr (channel == 1) return timer->CCR1; //CCR1 is already defined with volatile
		else if constexpr (channel == 2) return timer->CCR2;
		else if constexpr (channel == 3) return timer->CCR3;
		else if constexpr (channel == 4) return timer->CCR4;
		else static_assert(false, "Only channels numbers 1 - 4 are available!");
	};

	void SetPulse(const uint32_t pulse)
	{
		if (pulse > timer->ARR)
		{
			if constexpr (channel == 1) timer->CCR1 = timer->ARR;
			else if constexpr (channel == 2) timer->CCR2 = timer->ARR;
			else if constexpr (channel == 3) timer->CCR3 = timer->ARR;
			else if constexpr (channel == 4) timer->CCR4 = timer->ARR;
		}
		else
		{
			if constexpr (channel == 1) timer->CCR1 = pulse;
			else if constexpr (channel == 2) timer->CCR2 = pulse;
			else if constexpr (channel == 3) timer->CCR3 = pulse;
			else if constexpr (channel == 4) timer->CCR4 = pulse;
		}
	}

	void InterruptHandler()
	{
		//RM 31.4.6 TIMx Status register (TIMx_SR) -- UIF (update interrupt flag) rc_w0 readClear_with0
		//0 No update occure; 1 update interrupt pending. This bit is set by hardware when the registers are updated
		//capture compare interrupt 1 (CH1) handler
		if constexpr (channel == 1)
		{
			if (timer->SR & TIM_SR_CC1IF)
						timer->SR &= ~(TIM_SR_CC1IF); //capture compare event
		}
		else if constexpr (channel == 2)
		{
			if (timer->SR & TIM_SR_CC2IF)
				timer->SR &= ~(TIM_SR_CC2IF);
		}
		else if constexpr (channel == 3)
		{
			if (timer->SR & TIM_SR_CC3IF)
				timer->SR &= ~(TIM_SR_CC3IF);
		}
		else
		{
			if (timer->SR & TIM_SR_CC4IF)
				timer->SR &= ~(TIM_SR_CC4IF);
		}
	}

	void ConfigurePolarity(const PWMPolarity polarity)
	{
		if (polarity == PWMPolarity::ActiveHigh)
			timer->CCER &= ~(TIM_CCER_CCxP[channel - 1]);
		else if (polarity == PWMPolarity::ActiveLow)
			timer->CCER |= TIM_CCER_CCxP[channel - 1];
	}

};