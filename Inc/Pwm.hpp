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

enum class AlternateFunction
{
	AF0,
	AF1,
	AF2,
	AF3,
	AF4,
	AF5,
	AF6,
	AF7,
	AF8,
	AF9,
	AF10,
	AF11,
	AF12,
	AF13,
	AF14,
	AF15
};

//temporarly just for TIM3

template<std::uintptr_t timerAddr_
		, uint32_t prescalerPSC_ //0 - 15
		, uint32_t ARR_ /*auto-reload register*/>
class PWM
{
private:
	static constexpr std::uintptr_t timerAddr = timerAddr_;
	void EnableTimerClock()
	{
		if constexpr (timerAddr == TIM2_BASE)
			RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
		else if constexpr (timerAddr == TIM3_BASE)
			RCC->APB1ENR1 |= RCC_APB1ENR1_TIM3EN;
		else if constexpr (timerAddr == TIM4_BASE)
			RCC->APB1ENR1 |= RCC_APB1ENR1_TIM4EN;
		else if constexpr (timerAddr == TIM5_BASE)
			RCC->APB1ENR1 |= RCC_APB1ENR1_TIM5EN;
		else if constexpr (timerAddr == TIM6_BASE)
			RCC->APB1ENR1 |= RCC_APB1ENR1_TIM6EN;
		else if constexpr (timerAddr == TIM7_BASE)
			RCC->APB1ENR1 |= RCC_APB1ENR1_TIM7EN;
	}

	void InterruptsConfiguration()//TIM3
	{
		//RM 31.4.4 DMA/Interrupt enable register (DIER)
		//Trigger interrupt enable TIE (on/off - 1/0)
		//Update interrupt enable UIE (on/off - 1/0)

		auto tim = Tim();
		tim->DIER |= TIM_DIER_UIE; //update interrupt is enabled
		tim->DIER |= TIM_DIER_CC1IE; //capture/compare 1 interrupt 1 (CH1) enable

		//priority and enable in nvic; enum from stm32l476xx.h (CMSIS)
		NVIC_SetPriority(TIM3_IRQn, 1);
		NVIC_EnableIRQ(TIM3_IRQn);
	}

public:
	TIM_TypeDef* Tim() const { return reinterpret_cast<TIM_TypeDef*>(this->timerAddr); }
	PWM(const PWM& source) = delete;
	PWM(PWM&& source) = delete;
	PWM& operator=(const PWM& source) = delete;
	PWM& operator=(PWM&& source) = delete;
	PWM()
	{
		//PWM
		//Timer clock - RM 6.2. Tim3 PCLK x1 or x2 -> TIMPCLK
		//HPRE = 0x00 --> div 1
		//PPRE1 = 0x00 --> div 1 (APB1PRE)
		//RM 6.4.19 APB1ENR -- 1 enabled, 0 - disabled
		EnableTimerClock();
		auto tim = Tim();
		//TIMx_prescaler 16bit; RM 31.4.14 TIMx prescaler
		tim->PSC = prescalerPSC_; //should be 1 less, than divider
		tim->ARR = ARR_; //should be 1 less than divider
		//TIMER counter register
		tim->CNT = 0;//reset if it had different value
		//TIMER counter register
		tim->CNT = 0;//reset if it had different value
		//RM 31.4.1 CEN bit (Counter enable)
		tim->CR1 |= TIM_CR1_CEN;
		InterruptsConfiguration();
	}

	//TIM3
	void InterruptHandler()
	{
		//RM 31.4.6 TIMx Status register (TIMx_SR) -- UIF (update interrupt flag) rc_w0 readClear_with0
		//0 No update occure; 1 update interrupt pending. This bit is set by hardware when the registers are updated
		auto tim = Tim();
		if (tim->SR & TIM_SR_UIF)
			tim->SR &= ~(TIM_SR_UIF); //update event
	}

	uint32_t GetMaxWidth() const { return this->Tim()->ARR; };
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
	TIM_TypeDef* const timer = nullptr;

	void ConfigurePWM()
	{
		static_assert(pin >= 0 && pin <= 15, "Invalid pin number: needs to be in range of 0 - 15!");
		static_assert(channel >= 1 && channel <= 4, "Invalid channel number: needs to be in range of 1 - 4!");

		this->template ConfigureMODER<OptionsMODER::Alternate>();
		//Datasheet Pinouts and pin description
		//AF2 Datasheet Alternate functino 0 - 7
		//RM 8.5.10 GPIO alternate function low register (AFSEL6) because pin PA6
		GPIOA->AFR[0] |= GPIO_AFRL_AFSEL6_1; //to be refactored

		this->template ConfigureOTYPER<otyperOption>();
		this->template ConfigureOSPEEDR<ospeedrOption>();
		this->template ConfigurePUPDR<pupdrOption>();
		//RM 31.4.8 TIMx capture/compare mode register 1 (TIMx_CCMR1) [alternate] - output
		//Capture/Compare 1 Selection -CC1S - is output (0) by default, but check this bit
		//Output Compare 1 Mode - OC1M - PWM mode 1 -active when CNT < CCR1
		//0110  for PWM mode 1
		timer->CCMR1 |= TIM_CCMR1_OC1M_1;
		timer->CCMR1 |= TIM_CCMR1_OC1M_2;

		// Enable preload for CCR1 (this is **required** for PWM to work)
		timer->CCMR1 |= TIM_CCMR1_OC1PE;

		timer->CCR1 = 0;// just for initialization clearing the counter
		//capture/compare output enable bit
		timer->CCER |= TIM_CCER_CC1E; //Capture/compare enable register (TIMx_CCER1) RM 31.4.11
	}

public:
	static constexpr std::uintptr_t portAddr = portAddr_;
	static constexpr uint8_t pin = pin_;
	PWMChannel(const PWMChannel& source) = delete;
	PWMChannel(PWMChannel&& source) = delete;
	PWMChannel& operator=(const PWMChannel& source) = delete;
	PWMChannel& operator=(PWMChannel&& source) = delete;
	PWMChannel() = delete;
	PWMChannel(TIM_TypeDef*  const timer_) : timer(timer_)
	{
		this->EnableClock();
		ConfigurePWM();
	}

	//CCR1 - capture compare register 1 (CH1)
	//RM 31.4.16 Capture/compare register 1
	//Capture compare register (to which value counter will be compared to set high state
	//its pulse value in other words: 0% --> CCR1 = 0, 100% --> CCR1 = ARR
	uint32_t GetPulse_CH1() const { return timer->CCR1; };

	void SetPulse_CH1(const uint32_t pulse)
	{
		if (pulse > timer->ARR)
			timer->CCR1 = timer->ARR;
		else
			timer->CCR1 = pulse;
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
