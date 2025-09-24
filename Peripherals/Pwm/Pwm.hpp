#pragma once
#include "Config.hpp"
#include "../Peripherals/Gpio/GpioInput.hpp"

static constexpr std::array<uint32_t, 4> TIM_CCER_CCxNP {
	TIM_CCER_CC1NP, TIM_CCER_CC2NP, TIM_CCER_CC3NP, TIM_CCER_CC4NP
};

static constexpr std::array<uint32_t, 4> TIM_CCMR_CCx_Pos = {
	TIM_CCMR1_CC1S_Pos, TIM_CCMR1_CC2S_Pos, TIM_CCMR2_CC3S_Pos, TIM_CCMR2_CC4S_Pos};

static constexpr std::array<uint32_t, 4> TIM_CCER_CCxE = { TIM_CCER_CC1E, TIM_CCER_CC2E, TIM_CCER_CC3E, TIM_CCER_CC4E };

static constexpr std::array<uint32_t, 4> TIM_DIER_CCxIE = { TIM_DIER_CC1IE, TIM_DIER_CC2IE, TIM_DIER_CC3IE, TIM_DIER_CC4IE };

enum class PWMDirection : uint8_t
{
	Output = 0b00,
	InputDirect = 0b01,
	InputIndirect = 0b10,
	InputTRC = 0b11
};

enum class PWMPolarity
{
	ActiveHigh,
	ActiveLow,
};

template<typename T>
concept TimerConcept = requires(T timer){
	{ timer.CR1 }   -> std::convertible_to<volatile uint32_t&>;
	{ timer.CR2 }   -> std::convertible_to<volatile uint32_t&>;
	{ timer.SMCR }  -> std::convertible_to<volatile uint32_t&>;
	{ timer.DIER }  -> std::convertible_to<volatile uint32_t&>;
	{ timer.SR }    -> std::convertible_to<volatile uint32_t&>;
	{ timer.EGR }   -> std::convertible_to<volatile uint32_t&>;
	{ timer.CCMR1 } -> std::convertible_to<volatile uint32_t&>;
	{ timer.CCMR2 } -> std::convertible_to<volatile uint32_t&>;
	{ timer.CCER }  -> std::convertible_to<volatile uint32_t&>;
	{ timer.CNT }   -> std::convertible_to<volatile uint32_t&>;
	{ timer.PSC }   -> std::convertible_to<volatile uint32_t&>;
	{ timer.ARR }   -> std::convertible_to<volatile uint32_t&>;
	{ timer.RCR }   -> std::convertible_to<volatile uint32_t&>;
	{ timer.CCR1 }  -> std::convertible_to<volatile uint32_t&>;
	{ timer.CCR2 }  -> std::convertible_to<volatile uint32_t&>;
	{ timer.CCR3 }  -> std::convertible_to<volatile uint32_t&>;
	{ timer.CCR4 }  -> std::convertible_to<volatile uint32_t&>;
	{ timer.BDTR }  -> std::convertible_to<volatile uint32_t&>;
	{ timer.DCR }   -> std::convertible_to<volatile uint32_t&>;
	{ timer.DMAR }  -> std::convertible_to<volatile uint32_t&>;
	{ timer.OR1 }   -> std::convertible_to<volatile uint32_t&>;
	{ timer.CCMR3 } -> std::convertible_to<volatile uint32_t&>;
	{ timer.CCR5 }  -> std::convertible_to<volatile uint32_t&>;
	{ timer.CCR6 }  -> std::convertible_to<volatile uint32_t&>;
	{ timer.OR2 }   -> std::convertible_to<volatile uint32_t&>;
	{ timer.OR3 }   -> std::convertible_to<volatile uint32_t&>;
};

//Add concept for Timer and use cmsis timerTypedef instead of simply address and reinterpret cast later

template<TimerConcept Timer
		, uint32_t prescalerPSC_ //0 - 15
		, uint32_t ARR_ /*auto-reload register*/>
class PWM
{
private:
	void EnableTimerClockAndInterruptsConfig(const uint8_t priority)
	{
		//RM 31.4.4 DMA/Interrupt enable register (DIER)
		timer->DIER |= TIM_DIER_UIE; //update interrupt is enabled
		if (timer == TIM2)
		{
			RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN; //timer clock
			NVIC_SetPriority(TIM2_IRQn, priority);
			NVIC_EnableIRQ(TIM2_IRQn);
		}
		else if (timer == TIM3)
		{
			RCC->APB1ENR1 |= RCC_APB1ENR1_TIM3EN;
			NVIC_SetPriority(TIM3_IRQn, priority);
			NVIC_EnableIRQ(TIM3_IRQn);
		}
		else if (timer == TIM4)
		{
			RCC->APB1ENR1 |= RCC_APB1ENR1_TIM4EN;
			NVIC_SetPriority(TIM4_IRQn, priority);
			NVIC_EnableIRQ(TIM4_IRQn);
		}
		else if (timer == TIM5)
		{
			RCC->APB1ENR1 |= RCC_APB1ENR1_TIM5EN;
			NVIC_SetPriority(TIM5_IRQn, priority);
			NVIC_EnableIRQ(TIM5_IRQn);
		}
		else if (timer == TIM6)
		{
			RCC->APB1ENR1 |= RCC_APB1ENR1_TIM6EN;
			NVIC_SetPriority(TIM6_IRQn, priority);
			NVIC_EnableIRQ(TIM6_IRQn);
		}
		else if (timer == TIM7)
		{
			RCC->APB1ENR1 |= RCC_APB1ENR1_TIM7EN;
			NVIC_SetPriority(TIM7_IRQn, priority);
			NVIC_EnableIRQ(TIM7_IRQn);
		}
	}

public:
	Timer* timer = nullptr;
	PWM(const PWM& source) = delete;
	PWM(PWM&& source) = delete;
	PWM& operator=(const PWM& source) = delete;
	PWM& operator=(PWM&& source) = delete;
	PWM() = delete;
	PWM(Timer* const timer_, const uint8_t priority)
		: timer(timer_)
	{
		//PWM
		//Timer clock - RM 6.2. Tim3 PCLK x1 or x2 -> TIMPCLK
		//HPRE = 0x00 --> div 1
		//PPRE1 = 0x00 --> div 1 (APB1PRE)
		//RM 6.4.19 APB1ENR -- 1 enabled, 0 - disabled
		EnableTimerClockAndInterruptsConfig(priority);
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
		if (timer->SR & TIM_SR_UIF)
			timer->SR &= ~(TIM_SR_UIF); //update event
	}

	uint32_t GetMaxWidth() const { return timer->ARR; };
};
