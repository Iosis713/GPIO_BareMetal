/*
 * Button.hpp
 *
 *  Created on: May 24, 2025
 *      Author: bartoszlozinski
 */

#include "Gpio.hpp"

#ifndef BUTTON_HPP_
#define BUTTON_HPP_

template<std::uintptr_t portAddr_, uint8_t pin_, OptionsPUPDR pupdrOption>
class Button : public GpioInput<portAddr_, pin_, OptionsOTYPER::PushPull, OptionsOSPEEDR::LowSpeed, pupdrOption>
{
private:
public:
	Button(const Button& source) = delete;
	Button(Button&& source) = delete;
	Button& operator=(const Button& source) = delete;
	Button& operator=(Button&& source) = delete;
	Button()
	{
		this->EnableClock();
		this->ConfigureAsInput();
	}
	bool IsButtonPressed() const
	{
		if (this->ReadPin())
			return false; //normally opened button
		else
			return true;
	}
};



#endif /* BUTTON_HPP_ */
