#pragma once
#include "../Gpio/IGpio.hpp"

template<GpioPort Port
        , uint8_t pin_
		, uint8_t channel_
		, OptionsPUPDR pupdrOption = OptionsPUPDR::None>
class PWMChannelInput : public IGpio<PWMChannelInput<Port, pin_, channel_, pupdrOption>>
{
private:
	static constexpr uint8_t channel = channel_;
	volatile TIM_TypeDef* const timer = nullptr;

	void ConfigureChannel(const AlternateFunction af, const Trigger trigger, const PWMDirection direction)
	{
		static_assert(pin >= 0 && pin <= 15, "Invalid pin number: needs to be in range of 0 - 15!");
		static_assert(channel >= 1 && channel <= 4, "Invalid channel number: needs to be in range of 1 - 4!");

		this->template ConfigureMODER<OptionsMODER::Alternate>();
		//this->template ConfigureOTYPER<otyperOption>();
		//this->template ConfigureOSPEEDR<ospeedrOption>();
		this->template ConfigurePUPDR<pupdrOption>();
		ConfigureCaptureCompare(trigger, direction);
		//Datasheet Pinouts and pin description
		//AF2 Datasheet Alternate functino 0 - 7
		//RM 8.5.10 GPIO alternate function low register (AFSEL6) because pin PA6
		//To be checked in datasheet Pinouts and pin description / alternate function
		//It need to respect for Alternate function at Datasheet
		this->template ConfigureAlternateFunction(af);
	}

	void ConfigureCaptureCompare(const Trigger trigger, const PWMDirection direction)
	{
		//RM 31.4.8 TIMx capture/compare mode register 1/2 (TIMx_CCMR1) [alternate] - output
		//Capture/Compare 1 Selection -CC1S - is output (0) by default, but check this bit
		//Output Compare 1 Mode - OC1M - PWM mode 1 -active when CNT < CCR1

		ConfigureDirectionMode(direction);

		if constexpr (channel == 1)
			timer->CCR1 = 0;// just for initialization clearing the counter
		else if constexpr (channel == 2)
			timer->CCR2 = 0;
		else if constexpr (channel == 3)
			timer->CCR3 = 0;
		else
			timer->CCR4 = 0;

		timer->CCER |= TIM_CCER_CCxE[channel - 1]; //Capture/compare enable register (TIMx_CCER1) RM 31.4.11
		// OCxM must be left at reset value (0b000 = Frozen) in input mode

		//Trigger Rising/falling
		ConfigureCaptureCompareTrigger(trigger);

		//RM 31.4.4 DMA/Interrupt enable register (DIER)
		//Trigger interrupt enable TIE (on/off - 1/0)
		timer->DIER |= TIM_DIER_CCxIE[channel - 1];
	}

	void ConfigureDirectionMode(const PWMDirection direction)
	{
		//Output direction is default (00);
		auto& CCMR = (channel <= 2) ? timer->CCMR1 : timer->CCMR2;
		CCMR &= ~(PWMDirection::Output << TIM_CCMR_CCxS_Pos[channel - 1]);// 00 output
		CCMR |= (direction << TIM_CCMR_CCxS_Pos[channel - 1]);
	}

public:
	volatile Port* const port = nullptr;
	static constexpr uint8_t pin = pin_;
	PWMChannelInput(const PWMChannelInput& source) = delete;
	PWMChannelInput(PWMChannelInput&& source) = delete;
	PWMChannelInput& operator=(const PWMChannelInput& source) = delete;
	PWMChannelInput& operator=(PWMChannelInput&& source) = delete;
	PWMChannelInput() = delete;
	PWMChannelInput(TIM_TypeDef* const timer_, Port* const port_, const AlternateFunction af, const Trigger trigger, const PWMDirection direction = PWMDirection::Output)
		: timer(timer_)
		, port(port_)
	{
		this->EnableClock();
		ConfigureChannel(af, trigger, direction);
	}

	//CCR1 - capture compare register 1 (CH1)
	//RM 31.4.16 Capture/compare register 1
	//Capture compare register (to which value counter will be compared to set high state
	//its pulse value in other words: 0% --> CCR1 = 0, 100% --> CCR1 = ARR
	uint32_t GetCapturedValue() const
	{
		if constexpr (channel == 1) return timer->CCR1; //CCR1 is already defined with volatile
		else if constexpr (channel == 2) return timer->CCR2;
		else if constexpr (channel == 3) return timer->CCR3;
		else if constexpr (channel == 4) return timer->CCR4;
		else static_assert(false, "Only channels numbers 1 - 4 are available!");
	};

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

	void ConfigureCaptureCompareTrigger(const Trigger trigger)
	{
		if (trigger == Trigger::Rising)
		{
			timer->CCER &= ~(TIM_CCER_CCxNP[channel - 1]);
			timer->CCER &= ~(TIM_CCER_CCxP[channel - 1]);
		}
		else if (trigger == Trigger::Falling)
		{
			timer->CCER &= ~(TIM_CCER_CCxNP[channel - 1]);
			timer->CCER |= TIM_CCER_CCxP[channel - 1];
		}
		else
		{
			timer->CCER |= TIM_CCER_CCxNP[channel - 1];
			timer->CCER |= TIM_CCER_CCxP[channel - 1];
		}
	}
};
