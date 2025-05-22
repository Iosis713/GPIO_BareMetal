/*
 * gpio.hpp
 *
 *  Created on: May 21, 2025
 *      Author: bartoszlozinski
 */

#pragma once
#ifndef INC_GPIO_HPP_
#define INC_GPIO_HPP_

#include <stm32l476xx.h>
#include <cstdint>

template<typename Derived>
class IGpio
{
protected:
	GPIO_TypeDef* Port() const { return reinterpret_cast<GPIO_TypeDef*>(Derived::portAddr); }

public:
	void EnableClock()
	{
		if (Derived::portAddr == GPIOA_BASE)
			RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
		else if (Derived::portAddr == GPIOB_BASE)
			RCC->AHB2ENR |=RCC_AHB2ENR_GPIOBEN;
		else if (Derived::portAddr == GPIOC_BASE)
				RCC->AHB2ENR |=RCC_AHB2ENR_GPIOCEN;
	}
};

template<typename Derived>
class IGpioOutput : public IGpio<Derived>
{
public:
	void Set() { static_cast<Derived*>(this)->SetImpl(); }
	void Clear() { static_cast<Derived*>(this)->ClearImpl(); }
	void Toggle() { static_cast<Derived*>(this)->ToggleImpl(); }
};

template<std::uintptr_t portAddr_, uint8_t pin_>
class GpioOutput : public IGpioOutput<GpioOutput<portAddr_, pin_>>
{
private:
	void ConfigureAsOutput()
	{
		auto port = this->Port();
		//to refactor in the way of ConfigureLD2 switch-case
		port->MODER &= ~(0b11 << (pin * 2));
		port->MODER |= (0b01 << (pin * 2));
		port->OTYPER &= ~(1 << pin);
		port->OSPEEDR &= ~(0b11 << (pin * 2));
		port->PUPDR &= ~(0b11 << (pin * 2));
	}
	/*
	void ConfigureLD2()
	{
		//Reference Manual - Reset and clock control RCC
		//RCC->AHB2ENR |= (1<<0);
		RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN; //enable clock for GPIOA
		//GPIO registers
		//GPIOx_MODER - port mode register
		GPIOA->MODER |= GPIO_MODER_MODE5_0; //1
		GPIOA->MODER &= ~(GPIO_MODER_MODE5_1); //0
		//output type register -  OTYPER
		GPIOA->OTYPER &= ~(GPIO_OTYPER_OT5); //bit 5, value 0
		//OSPEEDR how fast the edge rises
		GPIOA->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED5);
		//PUPDR pull-up pull-down register
		GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD5); //00 -no pull up/no pull down
											 //10 - pull down
											 //would be like this?
		//GPIOA->PUPDR |= GPIO_PUPDR_PUPD5_0;
		//GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD5_1);
	}
	*/


public:
	static constexpr std::uintptr_t portAddr = portAddr_;
	static constexpr uint8_t pin = pin_;

	GpioOutput()
	{
		this->EnableClock();
		ConfigureAsOutput();
	}

	void SetImpl() { this->Port()->BSRR |= GPIO_BSRR_BS5; }
	void ClearImpl() { this->Port()->BSRR |= GPIO_BSRR_BR5; }
	void ToggleImpl()
	{
		//should be refactored and checked what ODR means. And applied in a way that Set/Clear is implemented
		if (this->Port->ODR & (1 << pin))
			ClearImpl();
		else
			SetImpl();
	}
};


#endif /* INC_GPIO_HPP_ */
