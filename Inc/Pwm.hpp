/*
 * Pwm.hpp
 *
 *  Created on: Jun 10, 2025
 *      Author: bartoszlozinski
 */

#pragma once
#ifndef PWM_HPP_
#define PWM_HPP_

#include "Config.hpp"
#include "Gpio.hpp"

template<std::uintptr_t timerAddr_
		, uint32_t prescalerPSC_ //0 - 15
		, uint32_t ARR_ /*auto-reload register*/>
class PWM
{
private:
	static constexpr std::uintptr_t timerAddr = timerAddr_;
	void EnableTimerClockAndInterruptsConfig(const uint8_t priority)
	{
		//RM 31.4.4 DMA/Interrupt enable register (DIER)
		auto timer = Timer(); //Update interrupt enable UIE (on/off - 1/0)
		timer->DIER |= TIM_DIER_UIE; //update interrupt is enabled

		if constexpr (timerAddr == TIM2_BASE)
		{
			RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN; //timer clock
			NVIC_SetPriority(TIM2_IRQn, priority);
			NVIC_EnableIRQ(TIM2_IRQn);
		}
		else if constexpr (timerAddr == TIM3_BASE)
		{
			RCC->APB1ENR1 |= RCC_APB1ENR1_TIM3EN;
			NVIC_SetPriority(TIM3_IRQn, priority);
			NVIC_EnableIRQ(TIM3_IRQn);
		}
		else if constexpr (timerAddr == TIM4_BASE)
		{
			RCC->APB1ENR1 |= RCC_APB1ENR1_TIM4EN;
			NVIC_SetPriority(TIM4_IRQn, priority);
			NVIC_EnableIRQ(TIM4_IRQn);
		}
		else if constexpr (timerAddr == TIM5_BASE)
		{
			RCC->APB1ENR1 |= RCC_APB1ENR1_TIM5EN;
			NVIC_SetPriority(TIM5_IRQn, priority);
			NVIC_EnableIRQ(TIM5_IRQn);
		}
		else if constexpr (timerAddr == TIM6_BASE)
		{
			RCC->APB1ENR1 |= RCC_APB1ENR1_TIM6EN;
			NVIC_SetPriority(TIM6_IRQn, priority);
			NVIC_EnableIRQ(TIM6_IRQn);
		}
		else if constexpr (timerAddr == TIM7_BASE)
		{
			RCC->APB1ENR1 |= RCC_APB1ENR1_TIM7EN;
			NVIC_SetPriority(TIM7_IRQn, priority);
			NVIC_EnableIRQ(TIM7_IRQn);
		}
	}

public:
	TIM_TypeDef* Timer() const { return reinterpret_cast<TIM_TypeDef*>(this->timerAddr); }
	PWM(const PWM& source) = delete;
	PWM(PWM&& source) = delete;
	PWM& operator=(const PWM& source) = delete;
	PWM& operator=(PWM&& source) = delete;
	PWM() = delete;
	PWM(const uint8_t priority)
	{
		//PWM
		//Timer clock - RM 6.2. Tim3 PCLK x1 or x2 -> TIMPCLK
		//HPRE = 0x00 --> div 1
		//PPRE1 = 0x00 --> div 1 (APB1PRE)
		//RM 6.4.19 APB1ENR -- 1 enabled, 0 - disabled
		EnableTimerClockAndInterruptsConfig(priority);
		auto timer = Timer();
		//TIMx_prescaler 16bit; RM 31.4.14 TIMx prescaler
		timer->PSC = prescalerPSC_; //should be 1 less, than divider
		timer->ARR = ARR_; //should be 1 less than divider
		//TIMER counter register
		timer->CNT = 0;//reset if it had different value
		//RM 31.4.1 CEN bit (Counter enable)
		timer->CR1 |= TIM_CR1_CEN;
	}

	void InterruptHandler()
	{
		//RM 31.4.6 TIMx Status register (TIMx_SR) -- UIF (update interrupt flag) rc_w0 readClear_with0
		//0 No update occure; 1 update interrupt pending. This bit is set by hardware when the registers are updated
		auto timer = Timer();
		if (timer->SR & TIM_SR_UIF)
			timer->SR &= ~(TIM_SR_UIF); //update event
	}

	uint32_t GetMaxWidth() const { return this->Timer()->ARR; };
};


template<std::uintptr_t portAddr_
        , uint8_t pin_
		, uint8_t channel_
		, OptionsOTYPER otyperOption = OptionsOTYPER::PushPull
		, OptionsOSPEEDR ospeedrOption = OptionsOSPEEDR::MediumSpeed
		, OptionsPUPDR pupdrOption = OptionsPUPDR::None>
class PWMChannel : public IGpio<PWMChannel<portAddr_, pin_, channel_, otyperOption, ospeedrOption, pupdrOption>>
{
private:
	static constexpr uint8_t channel = channel_;
	volatile TIM_TypeDef* const timer = nullptr;

	void ConfigurePWMChannel(const AlternateFunction af)
	{
		static_assert(pin >= 0 && pin <= 15, "Invalid pin number: needs to be in range of 0 - 15!");
		static_assert(channel >= 1 && channel <= 4, "Invalid channel number: needs to be in range of 1 - 4!");

		this->template ConfigureMODER<OptionsMODER::Alternate>();
		this->template ConfigureOTYPER<otyperOption>();
		this->template ConfigureOSPEEDR<ospeedrOption>();
		this->template ConfigurePUPDR<pupdrOption>();
		ConfigureCaptureCompare();
		//Datasheet Pinouts and pin description
		//AF2 Datasheet Alternate functino 0 - 7
		//RM 8.5.10 GPIO alternate function low register (AFSEL6) because pin PA6
		//To be checked in datasheet Pinouts and pin description / alternate function
		this->template ConfigureAlternateFunction(af);
	}

	void ConfigureCaptureCompare()
	{
		//RM 31.4.8 TIMx capture/compare mode register 1/2 (TIMx_CCMR1) [alternate] - output
		//Capture/Compare 1 Selection -CC1S - is output (0) by default, but check this bit
		//Output Compare 1 Mode - OC1M - PWM mode 1 -active when CNT < CCR1
		if (channel == 1)
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
		else if (channel == 2)
		{
			timer->CCMR1 |= TIM_CCMR1_OC2M_1; //refer to OM1M
			timer->CCMR1 |= TIM_CCMR1_OC2M_2;
			timer->CCMR1 |= TIM_CCMR1_OC2PE;
			timer->CCR2 = 0;
			timer->CCER |= TIM_CCER_CC2E;
			timer->DIER |= TIM_DIER_CC2IE;
		}
		else if (channel == 3)
		{
			timer->CCMR2 |= TIM_CCMR2_OC3M_1; //refer to OM1M
			timer->CCMR2 |= TIM_CCMR2_OC3M_2;
			timer->CCMR2 |= TIM_CCMR2_OC3PE;
			timer->CCR3 = 0;
			timer->CCER |= TIM_CCER_CC3E;
			timer->DIER |= TIM_DIER_CC3IE;

		}
		else if (channel == 4)
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
	static constexpr std::uintptr_t portAddr = portAddr_;
	static constexpr uint8_t pin = pin_;
	PWMChannel(const PWMChannel& source) = delete;
	PWMChannel(PWMChannel&& source) = delete;
	PWMChannel& operator=(const PWMChannel& source) = delete;
	PWMChannel& operator=(PWMChannel&& source) = delete;
	PWMChannel() = delete;
	PWMChannel(TIM_TypeDef* const timer_, const AlternateFunction af) : timer(timer_)
	{
		this->EnableClock();
		ConfigurePWMChannel(af);
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
		if (timer->SR & TIM_SR_CC1IF)
			timer->SR &= ~(TIM_SR_CC1IF); //capture compare event
	}
};

#endif /* PWM_HPP_ */
