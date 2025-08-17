#pragma once
#include "IGpio.hpp"

template<GpioPort Port
        , uint8_t pin_
		, OptionsOTYPER otyperOption = GpioDefaults::otyperOption
		, OptionsOSPEEDR ospeedrOption = GpioDefaults::ospeedrOption
		, OptionsPUPDR pupdrOption = GpioDefaults::pupdrOption>
class GpioInput : public IGpio<GpioInput<GPIO_TypeDef, pin_, otyperOption, ospeedrOption, pupdrOption>>
{
protected:
	volatile bool interruptOccured = false;

	void ConfigureAsInput()
	{
		static_assert(pin >= 0 && pin < 16, "Invalid pin number: needs to be in range of 0 - 15!");
		this->template ConfigureMODER<OptionsMODER::Input>();
		this->template ConfigureOTYPER<otyperOption>();
		this->template ConfigureOSPEEDR<ospeedrOption>();
		this->template ConfigurePUPDR<pupdrOption>();
	}
	void ConfigureEXTICR()
	{
		//RM 9.2.3 - 9.2.6 - SYSCFG External interrupt configuration register X
		//EXTI enable SYSCFG_EXTICR* bit)
		//EXTICR1 (index 0): pins 0 - 3, EXTICR2 (1): pins 4 - 7; EXTICR3(2): pins 8- 11; EXTICR4(3): pins 12 - 15
		constexpr uint8_t extiCrIndex = pin / 4;
		SYSCFG->EXTICR[extiCrIndex] &= ~SYSCFG_EXTI[pin]; //reset to 0000 (defualt value);
		if (this->port == GPIOA)
			SYSCFG->EXTICR[extiCrIndex] |= SYSCFG_EXTI_PA[pin]; //example for pin PC13 = GPIOC_BASE, pin 13 (index is proper, as pins are 0 - 15)
		else if (this->port == GPIOB)
			SYSCFG->EXTICR[extiCrIndex] |= SYSCFG_EXTI_PB[pin];
		else if (this->port == GPIOC)
			SYSCFG->EXTICR[extiCrIndex] |= SYSCFG_EXTI_PC[pin];
		else if (this->port == GPIOD)
			SYSCFG->EXTICR[extiCrIndex] |= SYSCFG_EXTI_PD[pin];
		else if (this->port == GPIOE)
			SYSCFG->EXTICR[extiCrIndex] |= SYSCFG_EXTI_PE[pin];
		else if (this->port == GPIOF)
			SYSCFG->EXTICR[extiCrIndex] |= SYSCFG_EXTI_PF[pin];
		else if (this->port == GPIOG)
			SYSCFG->EXTICR[extiCrIndex] |= SYSCFG_EXTI_PG[pin];
	}

	//interrupt priority; enum from stm32l476xx.h (CMSIS file) - Interrupt number definition
	template <uint8_t priority>
	void NvicExtiPriorityConfigurator(const IRQn_Type irqnType)
	{
		static_assert(priority <= 15, "EXTI priority should be in range: 0 - 15!");
		NVIC_SetPriority(irqnType, priority); //set priority 0, priotity (0 - 15)
		NVIC_EnableIRQ(irqnType); //enable interrupt
	}

	template<uint8_t priority>
	void ConfigureExtiPriority()
	{
		if constexpr (pin == 0)
			NvicExtiPriorityConfigurator<priority>(EXTI0_IRQn);
		else if constexpr (pin == 1)
			NvicExtiPriorityConfigurator<priority>(EXTI1_IRQn);
		else if constexpr (pin == 2)
			NvicExtiPriorityConfigurator<priority>(EXTI2_IRQn);
		else if constexpr (pin == 3)
			NvicExtiPriorityConfigurator<priority>(EXTI3_IRQn);
		else if constexpr (pin == 4)
			NvicExtiPriorityConfigurator<priority>(EXTI4_IRQn);
		else if constexpr (pin >= 5 && pin <= 9)
			NvicExtiPriorityConfigurator<priority>(EXTI9_5_IRQn);
		else if constexpr (pin >= 10 && pin <= 15)
			NvicExtiPriorityConfigurator<priority>(EXTI15_10_IRQn);
	}

public:
	Port* const port = nullptr;
	static constexpr uint8_t pin = pin_;

	GpioInput(const GpioInput& source) = delete;
	GpioInput(GpioInput&& source) = delete;
	GpioInput& operator=(const GpioInput& source) = delete;
	GpioInput& operator=(GpioInput&& source) = delete;
	GpioInput(Port* const port_)
		: port(port_)
	{
		this->EnableClock();
		ConfigureAsInput();
	}

	//RM External iterrupt/event (EXTI) GPIO Mapping (multiplexer)
	//For single exti only single port
	//I.e interrupt for PC13 cannot be set for PA13 at the same time
	template<uint8_t priority>
	void ConfigureEXTI(const Trigger trigger)
	{
		//RM 9 System configuration controller SYSCFG:
		//Manages among the others external interrupt line connection to GPIOs (EXTI)
		//RM 6.4.21 - (APB2 peripheral clock enable register [lookup for SYSCFG bit]
		RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
		ConfigureEXTICR();

		//reference manual 14.5.3 Rising trigger selection register
		//from high to low -- falling; from low to high -- rising
		if (trigger == Trigger::Rising)
			EXTI->RTSR1 |= EXTI_RTSR1_RT[pin];
		else
			EXTI->FTSR1 |= EXTI_FTSR1_FT[pin];

		//Interrupt mask register IMR - unmasked
		EXTI->IMR1 |= EXTI_IMR1[pin];

		ConfigureExtiPriority<priority>();
	}

	bool ReadPin() const {return this->Port()->IDR & PinMask<pin>();} //true - high state, false - low state
	void ClearInterruptFlag() { this->interruptOccured = false; };
	void IrqHandler()
	{
		if (EXTI->PR1 & EXTI_PR1_PIF[pin])
		{
			EXTI->PR1 |= EXTI_PR1_PIF[pin]; //PIFx --> rc_w1 - read/clear with 1 - used 1 to read and clear interrupt state

			interruptOccured = true;
		}
	}
	bool InterruptOccured() const { return this->interruptOccured; }

};

