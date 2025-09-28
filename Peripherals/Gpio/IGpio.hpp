#pragma once
#include "Config.hpp"
#include <cstdint>
#include <concepts>

enum class OptionsPUPDR
{
	// GPIOx_PUPDR (GPIO port pull-up/pull-down register):
    // Reset value: 0x6400 0000 (port A) - pin15 [pull-up], pin14 [pull-down], others [No pull-up/pull-down]
	// Reset value: 0x0000 0100 (port B) - pin4 [pull-up]
	// Reset value: 0x0000 0000 (for other ports)
    None = 0b00, // 00 = No pull-up, no pull-down
    PullUp = 0b01, // 01 = Pull-up
    PullDown = 0b10,// 10 = Pull-down
    /*Reserved,// 11 = Reserved - but you shouldn't use reserved pin*/
};

enum class OptionsOTYPER
{
	// GPIOx_OTYPER (GPIO port output type register):
	//reset value 0x0000 0000
	PushPull = 0b0, // 0 = Output push-pull (reset state)
	OpenDrain = 0b1,// 1 = Output open-drain
};

enum class OptionsMODER
{
	// STM32L476RGT6 Reference Manual:
	// GPIOx_MODER (GPIO port mode register) controls the mode of each pin
	//Reset value: 0xABFF FFFF (port A) - 1010 1011 1111 1111 1111 1111 1111 1111
	//Reset value: 0xFFFF FEBF (port B) - 1111 1111 1111 1111 1111 1110 1011 1111
	Input = 0b00, // 00 = Input mode
	Output = 0b01, // 01 = General purpose output mode
	Alternate = 0b10, // 10 = Alternate function mode
	Analog = 0b11, // 11 = Analog mode (reset state for most cases)
};

enum class OptionsOSPEEDR
{
	// STM32L476RGT6 RM: 8.5.3 GPIO port output speed register
	// Reset value: 0x0C00 0000 for port A
	// Reset value: 0x0000 0000 for other ports
	LowSpeed = 0b00, // 00
	MediumSpeed = 0b01, // 01
	HighSpeed = 0b10, // 10
	VeryHighSpeed = 0b11, //11
	//Refer to the device datasheet for the frequency specifications
	//and the power supply and load conditions for each speed...
};

//Datasheet Pinouts and pin description
//AF2 Datasheet Alternate functinon 0 - 15
enum class AlternateFunction
{
	AF0 = 0b0000,
	AF1 = 0b0001,
	AF2 = 0b0010,
	AF3 = 0b0011,
	AF4 = 0b0100,
	AF5 = 0b0101,
	AF6 = 0b0110,
	AF7 = 0b0111,
	AF8 = 0b1000,
	AF9 = 0b1001,
	AF10 = 0b1010,
	AF11 = 0b1011,
	AF12 = 0b1100,
	AF13 = 0b1101,
	AF14 = 0b1110,
	AF15 = 0b1111
};

enum class Trigger
{
	Rising,
	Falling,
	Both
};

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

template<typename Derived>
class IGpio
{
#ifdef UNIT_TESTS
public:
#else
protected:
#endif	

	void EnableClock()
	{
		if (static_cast<Derived*>(this)->port == GPIOA)
			RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
		else if (static_cast<Derived*>(this)->port == GPIOB)
			RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
		else if (static_cast<Derived*>(this)->port == GPIOC)
			RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;
		else if (static_cast<Derived*>(this)->port == GPIOD)
			RCC->AHB2ENR |= RCC_AHB2ENR_GPIODEN;
	}

	template<OptionsPUPDR pupdrOption>
	void ConfigurePUPDR()
	{
		using enum OptionsPUPDR;
		static constexpr uint8_t pupdrMask = 0b11;
		static constexpr uint8_t bitShift = 2 * Derived::pin;

		static_cast<Derived*>(this)->port->PUPDR &= ~(pupdrMask << bitShift);
		static_cast<Derived*>(this)->port->PUPDR |= (static_cast<uint32_t>(pupdrOption) << bitShift);
	}

	template<OptionsOTYPER otyperOption>
	void ConfigureOTYPER()
	{
		using enum OptionsOTYPER;
		static constexpr uint8_t otyperMask = 0b1;
		static_cast<Derived*>(this)->port->OTYPER &= ~(otyperMask << Derived::pin);//clear bits
		static_cast<Derived*>(this)->port->OTYPER |= (static_cast<uint32_t>(otyperOption) << Derived::pin);
	}

	template<OptionsMODER moderOption>
	void ConfigureMODER()
	{
		using enum OptionsMODER;
		static constexpr uint8_t moderMask = 0b11;
		static constexpr uint8_t bitShift = 2 * Derived::pin;

		static_cast<Derived*>(this)->port->MODER &= ~(moderMask << bitShift);//clear bits
		static_cast<Derived*>(this)->port->MODER |= (static_cast<uint32_t>(moderOption) << bitShift);
	}

	template<OptionsOSPEEDR ospeedrOption>
	void ConfigureOSPEEDR()
	{
		using enum OptionsOSPEEDR;
		static constexpr uint8_t ospeedrMask = 0b11;
		static constexpr uint8_t bitShift = 2 * Derived::pin;

		static_cast<Derived*>(this)->port->OSPEEDR &= ~(ospeedrMask << bitShift);//clear bits
		static_cast<Derived*>(this)->port->OSPEEDR |= (static_cast<uint32_t>(ospeedrOption) << bitShift);
	}

public:
	//Datasheet Pinouts and pin description
	//AF2 Datasheet Alternate functino 0 - 7 / 8 -15
	//RM 8.5.10 GPIO alternate function low/high register
	void ConfigureAlternateFunction(const AlternateFunction af)
	{
		using enum AlternateFunction;

		uint8_t LowOrHigh = Derived::pin <= 7 ? 0 : 1;
		static constexpr uint32_t AFMask = 0b1111;
		static constexpr uint32_t bitShift = 4 * (Derived::pin % 8); //AF is 4 bits wide
		static_cast<Derived*>(this)->port->AFR[LowOrHigh] &= ~(AFMask << bitShift);
		static_cast<Derived*>(this)->port->AFR[LowOrHigh] |= (static_cast<uint32_t>(af) << bitShift);
	}
};

struct GpioDefaults
{
    static constexpr auto otyperOption = OptionsOTYPER::PushPull;
	static constexpr auto ospeedrOption = OptionsOSPEEDR::HighSpeed;
	static constexpr auto pupdrOption = OptionsPUPDR::None;
};
