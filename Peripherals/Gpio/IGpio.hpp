#pragma once
#include "Config.hpp"
#include <cstdint>
#include <concepts>

enum class OptionsPUPDR
{
	// GPIOx_PUPDR (GPIO Derived::port pull-up/pull-down register):
    // Reset value: 0x6400 0000 (for Derived::port A) - pin15 [pull-up], pin14 [pull-down], others [No pull-up/pull-down]
	// Reset value: 0x0000 0100 (for Derived::port B) - pin4 [pull-up]
	// Reset value: 0x0000 0000 (for other ports)
    None, // 00 = No pull-up, no pull-down
    PullUp, // 01 = Pull-up
    PullDown,// 10 = Pull-down
    /*Reserved,// 11 = Reserved - but you shouldn't use reserved pin*/
};

enum class OptionsOTYPER
{
	// GPIOx_OTYPER (GPIO Derived::port output type register):
	//reset value 0x0000 0000
	PushPull, // 0 = Output push-pull (reset state)
	OpenDrain,// 1 = Output open-drain
};

enum class OptionsMODER
{
	// STM32L476RGT6 Reference Manual:
	// GPIOx_MODER (GPIO Derived::port mode register) controls the mode of each pin
	//Reset value: 0xABFF FFFF (Derived::port A) - 1010 1011 1111 1111 1111 1111 1111 1111
	//Reset value: 0xFFFF FEBF (Derived::port B) - 1111 1111 1111 1111 1111 1110 1011 1111
	Input, // 00 = Input mode
	Output, // 01 = General purpose output mode
	Alternate, // 10 = Alternate function mode
	Analog, // 11 = Analog mode (reset state for most cases)
};

enum class OptionsOSPEEDR
{
	// STM32L476RGT6 RM: 8.5.3 GPIO Derived::port output speed register
	// Reset value: 0x0C00 0000 for Derived::port A
	// Reset value: 0x0000 0000 for other ports
	LowSpeed, // 00
	MediumSpeed, // 01
	HighSpeed, // 10
	VeryHighSpeed, //11
	//Refer to the device datasheet for the frequency specifications
	//and the power supply and load conditions for each speed...
};

//Datasheet Pinouts and pin description
//AF2 Datasheet Alternate functino 0 - 7
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

enum class Trigger
{
	Rising,
	Falling,
	Both
};

/////////////////////////////////////////////////////////////////////////////

//do something like that to make it testable

template<typename T>
concept GpioPort = requires(T port) {
    { port.MODER } -> std::convertible_to<volatile uint32_t&>;
    { port.OTYPER } -> std::convertible_to<volatile uint32_t&>;
    { port.OSPEEDR } -> std::convertible_to<volatile uint32_t&>;
    { port.PUPDR } -> std::convertible_to<volatile uint32_t&>;
    { port.IDR } -> std::convertible_to<volatile uint32_t&>;
    { port.ODR } -> std::convertible_to<volatile uint32_t&>;
    { port.BSRR } -> std::convertible_to<volatile uint32_t&>;
    { port.LCKR } -> std::convertible_to<volatile uint32_t&>;
    { port.AFR } -> std::convertible_to<volatile uint32_t(&)[2]>;
    { port.BRR } -> std::convertible_to<volatile uint32_t&>;
    { port.ASCR } -> std::convertible_to<volatile uint32_t&>;
};

/*
template<GpioPort PortType, uint8_t pin>
class GpioOutput {
public:
    explicit GpioOutput(PortType* Derived::port) : port_(Derived::port) {}

    void ConfigureAsOutput() {
        port_->MODER &= ~(0b11 << (2*pin));
        port_->MODER |= (0b01 << (2*pin)); // Output mode
        // configure OTYPER, OSPEEDR, PUPDR similarly
    }

private:
    PortType* port_;
};

*/

/////////////////////////////////////////////////////////////////////////////

template<typename Derived>
class IGpio
{
protected:
	GPIO_TypeDef* Derived::port const { return reinterpret_cast<GPIO_TypeDef*>(Derived::portAddr); }

	void EnableClock()
	{
		if (Derived::port == GPIOA)
			RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
		else if (Derived::port == GPIOB)
			RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
		else if (Derived::port == GPIOC)
			RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;
		else if (Derived::port == GPIOD)
			RCC->AHB2ENR |= RCC_AHB2ENR_GPIODEN;
	}

	template<OptionsPUPDR pupdrOption>
	void ConfigurePUPDR()
	{
		using enum OptionsPUPDR;
		Derived::port->PUPDR &= ~(PUPDR_MASKS[Derived::pin]); //00- no pull-up, no pull-down
		if constexpr (pupdrOption == PullUp)
			Derived::port->PUPDR |= PUPDR_MASKS_0[Derived::pin];
		else if constexpr (pupdrOption == PullDown)
			Derived::port->PUPDR |= PUPDR_MASKS_1[Derived::pin];
	}

	template<OptionsOTYPER otyperOption>
	void ConfigureOTYPER()
	{
		using enum OptionsOTYPER;
		if constexpr (otyperOption == PushPull)
			Derived::port->OTYPER &= ~OTYPER_BITS[Derived::pin];
		else if constexpr (otyperOption == OpenDrain)
			Derived::port->OTYPER |= OTYPER_BITS[Derived::pin];
	}

	template<OptionsMODER moderOption>
	void ConfigureMODER()
	{
		using enum OptionsMODER;
		Derived::port->MODER &= ~(MODER_MASKS[Derived::pin]); //reset to 00
		if constexpr (moderOption == Input)
			return;
		else if constexpr (moderOption == Output)
			Derived::port->MODER |= MODER_MASKS_0[Derived::pin];
		else if constexpr (moderOption == Alternate)
			Derived::port->MODER |= MODER_MASKS_1[Derived::pin];
		else if constexpr (moderOption == Analog)
			Derived::port->MODER |= MODER_MASKS[Derived::pin];
	}

	template<OptionsOSPEEDR ospeedrOption>
	void ConfigureOSPEEDR()
	{
		using enum OptionsOSPEEDR;
		Derived::port->OSPEEDR &= ~OSPEEDR_MASKS[Derived::pin];
		if constexpr (ospeedrOption == LowSpeed)
			return;
		else if constexpr (ospeedrOption == MediumSpeed)
			Derived::port->OSPEEDR |= OSPEEDR_MASKS_0[Derived::pin];
		else if constexpr (ospeedrOption == HighSpeed)
			Derived::port->OSPEEDR |= OSPEEDR_MASKS_1[Derived::pin];
		else if constexpr (ospeedrOption == VeryHighSpeed)
			Derived::port->OSPEEDR |= OSPEEDR_MASKS[Derived::pin];
	}

public:
	//Datasheet Pinouts and pin description
	//AF2 Datasheet Alternate functino 0 - 7 / 8 -15
	//RM 8.5.10 GPIO alternate function low/high register
	void ConfigureAlternateFunction(const AlternateFunction af)
	{
		uint8_t LowOrHigh = Derived::pin <= 7 ? 0 : 1;
		Derived::port->AFR[LowOrHigh] &= ~GPIO_AFR_AFSEL_MASKS[Derived::pin][4]; //clearing bits

		using enum AlternateFunction;
		switch (af)
		{
		case AF0:
			break; //0000 (reset value)
		case AF1:
			Derived::port->AFR[LowOrHigh] |= GPIO_AFR_AFSEL_MASKS[Derived::pin][0]; //0001
			break;
		case AF2:
			Derived::port->AFR[LowOrHigh] |= GPIO_AFR_AFSEL_MASKS[Derived::pin][1]; //0010
			break;
		case AF3:
			Derived::port->AFR[LowOrHigh] |= GPIO_AFR_AFSEL_MASKS[Derived::pin][0]; //0011
			Derived::port->AFR[LowOrHigh] |= GPIO_AFR_AFSEL_MASKS[Derived::pin][1];
			break;
		case AF4:
			Derived::port->AFR[LowOrHigh] |= GPIO_AFR_AFSEL_MASKS[Derived::pin][2]; //0100
			break;
		case AF5:
			Derived::port->AFR[LowOrHigh] |= GPIO_AFR_AFSEL_MASKS[Derived::pin][0]; //0101
			Derived::port->AFR[LowOrHigh] |= GPIO_AFR_AFSEL_MASKS[Derived::pin][2];
			break;
		case AF6:
			Derived::port->AFR[LowOrHigh] |= GPIO_AFR_AFSEL_MASKS[Derived::pin][1]; //0110
			Derived::port->AFR[LowOrHigh] |= GPIO_AFR_AFSEL_MASKS[Derived::pin][2];
			break;
		case AF7:
			Derived::port->AFR[LowOrHigh] |= GPIO_AFR_AFSEL_MASKS[Derived::pin][4]; //0111
			Derived::port->AFR[LowOrHigh] &= ~GPIO_AFR_AFSEL_MASKS[Derived::pin][3];
			break;
		case AF8:
			Derived::port->AFR[LowOrHigh] |= GPIO_AFR_AFSEL_MASKS[Derived::pin][3]; //1000
			break;
		case AF9:
			Derived::port->AFR[LowOrHigh] |= GPIO_AFR_AFSEL_MASKS[Derived::pin][0]; //1001
			Derived::port->AFR[LowOrHigh] |= GPIO_AFR_AFSEL_MASKS[Derived::pin][3];
			break;
		case AF10:
			Derived::port->AFR[LowOrHigh] |= GPIO_AFR_AFSEL_MASKS[Derived::pin][1]; //1010
			Derived::port->AFR[LowOrHigh] |= GPIO_AFR_AFSEL_MASKS[Derived::pin][3];
			break;
		case AF11:
			Derived::port->AFR[LowOrHigh] |= GPIO_AFR_AFSEL_MASKS[Derived::pin][4]; //1011
			Derived::port->AFR[LowOrHigh] &= ~GPIO_AFR_AFSEL_MASKS[Derived::pin][2];
			break;
		case AF12:
			Derived::port->AFR[LowOrHigh] |= GPIO_AFR_AFSEL_MASKS[Derived::pin][2]; //1100
			Derived::port->AFR[LowOrHigh] |= GPIO_AFR_AFSEL_MASKS[Derived::pin][3];
			break;
		case AF13:
			Derived::port->AFR[LowOrHigh] |= GPIO_AFR_AFSEL_MASKS[Derived::pin][4]; //1101
			Derived::port->AFR[LowOrHigh] &= ~GPIO_AFR_AFSEL_MASKS[Derived::pin][1];
			break;
		case AF14:
			Derived::port->AFR[LowOrHigh] |= GPIO_AFR_AFSEL_MASKS[Derived::pin][4]; //1110
			Derived::port->AFR[LowOrHigh] &= ~GPIO_AFR_AFSEL_MASKS[Derived::pin][0];
			break;
		case AF15:
			Derived::port->AFR[LowOrHigh] |= GPIO_AFR_AFSEL_MASKS[Derived::pin][4]; //1111
			break;
		default:
			break;
		}
	}
};

struct GpioDefaults
{
    static constexpr auto otyperOption = OptionsOTYPER::PushPull;
	static constexpr auto ospeedrOption = OptionsOSPEEDR::HighSpeed;
	static constexpr auto pupdrOption = OptionsPUPDR::None;
};
