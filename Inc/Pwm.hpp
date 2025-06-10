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

//temporarly just for TIM3

template<std::uintptr_t portAddr_
        , uint8_t pin_
		, std::uintptr_t timerAddr_
		, uint32_t prescalerPSC_ /*0 - 15*/
		, uint32_t ARR_ /*auto-reload register*/
		, OptionsOTYPER otyperOption = OptionsOTYPER::PushPull
		, OptionsOSPEEDR ospeedrOption = OptionsOSPEEDR::MediumSpeed
		, OptionsPUPDR pupdrOption = OptionsPUPDR::None>
class PWM : public IGpio<PWM<portAddr_, pin_, timerAddr_, prescalerPSC_, ARR_, otyperOption, ospeedrOption, pupdrOption>>
{
private:
	std::uintptr_t timerAddr = timerAddr_;
	TIM_TypeDef* Tim() const { return reinterpret_cast<TIM_TypeDef*>(this->timerAddr); }

	void ConfigurePWM()
	{
		static_assert(pin >= 0 && pin < 16, "Invalid pin number: needs to be in range of 0 - 15!");
		this->template ConfigureMODER<OptionsMODER::Alternate>();
		GPIOA->AFR[0] |= GPIO_AFRL_AFSEL6_1; //to be refactored

		this->template ConfigureOTYPER<otyperOption>();
		this->template ConfigureOSPEEDR<ospeedrOption>();
		this->template ConfigurePUPDR<pupdrOption>();
		//RM 31.4.8 TIMx capture/compare mode register 1 (TIMx_CCMR1) [alternate] - output
		//Capture/Compare 1 Selection -CC1S - is output (0) by default, but check this bit
		//Output Compare 1 Mode - OC1M - PWM mode 1 -active when CNT < CCR1
		//0110  for PWM mode 1
		auto tim = Tim(); // to be refactored for other timers
		tim->CCMR1 |= TIM_CCMR1_OC1M_1;
		tim->CCMR1 |= TIM_CCMR1_OC1M_2;

		// Enable preload for CCR1 (this is **required** for PWM to work)
		tim->CCMR1 |= TIM_CCMR1_OC1PE;

		tim->CCR1 = 0;// just for initialization clearing the counter
		//capture/compare output enable bit
		tim->CCER |= TIM_CCER_CC1E; //Capture/compare enable register (TIMx_CCER1) RM 31.4.11


	}
public:
	static constexpr std::uintptr_t portAddr = portAddr_;
	static constexpr uint8_t pin = pin_;
	PWM(const PWM& source) = delete;
	PWM(PWM&& source) = delete;
	PWM& operator=(const PWM& source) = delete;
	PWM& operator=(PWM&& source) = delete;
	PWM()
	{
		this->EnableClock();
		//PWM
		//Timer clock - RM 6.2. Tim3 PCLK x1 or x2 -> TIMPCLK
		//HPRE = 0x00 --> div 1
		//PPRE1 = 0x00 --> div 1 (APB1PRE)
		//RM 6.4.19 APB1ENR -- 1 enabled, 0 - disabled
		RCC->APB1ENR1 |= RCC_APB1ENR1_TIM3EN;
		auto tim = Tim();
		//TIMx_prescaler 16bit; RM 31.4.14 TIMx prescaler
		tim->PSC = prescalerPSC_; //should be 1 less, than divider
		tim->ARR = ARR_; //should be 1 less than divider
		//TIMER counter register
		tim->CNT = 0;//reset if it had different value
		//RM 31.4.1 CEN bit (Counter enable)
		tim->CR1 |= TIM_CR1_CEN;
		ConfigurePWM();
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

	//TIM3
	void InterruptHandler()
	{
		//RM 31.4.6 TIMx Status register (TIMx_SR) -- UIF (update interrupt flag) rc_w0 readClear_with0
		//0 No update occure; 1 update interrupt pending. This bit is set by hardware when the registers are updated
		auto tim = Tim();
		if (tim->SR & TIM_SR_UIF)
			tim->SR &= ~(TIM_SR_UIF); //update event

		//capture compare interrupt 1 (CH1) handler
		if (tim->SR & TIM_SR_CC1IF)
			tim->SR &= ~(TIM_SR_CC1IF); //capture compare event
	}

	//CCR1 - capture compare register 1 (CH1)
	//RM 31.4.16 Capture/compare register 1
	//Capture compare register (to which value counter will be compared to set high state
	//its pulse value in other words: 0% --> CCR1 = 0, 100% --> CCR1 = ARR
	uint32_t GetPulse_CH1() const { return this->Tim()->CCR1; };
	uint32_t GetMaxWidth() const { return this->Tim()->ARR; };
	void SetPulse_CH1(const uint32_t pulse)
	{
		auto tim = Tim();
		if (pulse > tim->ARR)
			tim->CCR1 = tim->ARR;
		else
			tim->CCR1 = pulse;
	}

};


#endif /* PWM_HPP_ */
