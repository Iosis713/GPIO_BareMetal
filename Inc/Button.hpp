#pragma once

#ifndef BUTTON_HPP_
#define BUTTON_HPP_

#include "../Peripherals/Gpio/GpioInput.hpp"

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
		return !this->ReadPin(); //ReadPin returns true for high state. This button is normally opened
	}
};



#endif /* BUTTON_HPP_ */
