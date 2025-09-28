#pragma once
#include "../Peripherals/Gpio/GpioAlternate.hpp"

enum class SpiMOSI
{
	SPI1_PA7_AF5,
	SPI1_PB5_AF5,
	//SPI1_PE15_AF5,

	SPI2_PB15_AF5,
	SPI2_PC3_AF5,
	SPI2_PD4_AF5,

	SPI3_PB5_AF6,
	SPI3_PC12_AF6,
};

enum class SpiMISO
{
	SPI1_PA6_AF5,
	SPI1_PB4_AF5,
	//SPI1_PE14_AF5,

	SPI2_PB14_AF5,
	SPI2_PC2_AF5,
	SPI2_PD3_AF5,

	SPI3_PB4_AF6,
	SPI3_PC11_AF6,
};


enum class SpiSCK
{
	SPI1_PA5_AF5,
	SPI1_PB3_AF5,
	//SPI1_PE13_AF5,

	SPI2_PB10_AF5,
	SPI2_PD1_AF5,

	SPI3_PB3_AF6,
	SPI3_PC10_AF6
};

class ISpiPins
{
protected:
	template<SpiSCK spiSCK_>
	static constexpr auto /*GpioAlternate<T1, T2, T3>*/ ConfigureSCK()
	{
		using enum SpiSCK;
		if constexpr (spiSCK_ == SPI1_PA5_AF5)
			return GpioAlternate<GPIO_TypeDef, 5, AlternateFunction::AF5>{GPIOA};
		else if constexpr (spiSCK_ == SPI1_PB3_AF5)
			return GpioAlternate<GPIO_TypeDef, 3, AlternateFunction::AF5>{GPIOB};
		else if constexpr (spiSCK_ == SPI2_PB10_AF5)
			return GpioAlternate<GPIO_TypeDef, 10, AlternateFunction::AF5>{GPIOB};
		else if constexpr (spiSCK_ == SPI2_PD1_AF5)
			return GpioAlternate<GPIO_TypeDef, 1, AlternateFunction::AF5>{GPIOD};
		else if constexpr (spiSCK_ == SPI3_PB3_AF6)
			return GpioAlternate<GPIO_TypeDef, 3, AlternateFunction::AF6>{GPIOB};
		else
			return GpioAlternate<GPIO_TypeDef, 10, AlternateFunction::AF6>{GPIOC};
	}

	template<SpiMISO spiMISO_>
	static constexpr auto ConfigureMISO()
	{
		using enum SpiMISO;
		if constexpr (spiMISO_ == SPI1_PA6_AF5)
			return GpioAlternate<GPIO_TypeDef, 6, AlternateFunction::AF5>{GPIOA};
		else if constexpr (spiMISO_ == SPI1_PB4_AF5)
			return GpioAlternate<GPIO_TypeDef, 4, AlternateFunction::AF5>{GPIOB};
		else if constexpr (spiMISO_ == SPI2_PB14_AF5)
			return GpioAlternate<GPIO_TypeDef, 14, AlternateFunction::AF5>{GPIOB};
		else if constexpr (spiMISO_ == SPI2_PC2_AF5)
			return GpioAlternate<GPIO_TypeDef, 2, AlternateFunction::AF5>{GPIOC};
		else if constexpr (spiMISO_ == SPI2_PD3_AF5)
			return GpioAlternate<GPIO_TypeDef, 3, AlternateFunction::AF5>{GPIOD};
		else if constexpr (spiMISO_ == SPI3_PB4_AF6)
			return GpioAlternate<GPIO_TypeDef, 4, AlternateFunction::AF6>{GPIOB};
		else
			return GpioAlternate<GPIO_TypeDef, 11, AlternateFunction::AF6>{GPIOC};
	}

	template<SpiMOSI spiMOSI_>
	static constexpr auto ConfigureMOSI()
	{
		using enum SpiMOSI;
		if constexpr (spiMOSI_ == SPI1_PA7_AF5)
			return GpioAlternate<GPIO_TypeDef, 7, AlternateFunction::AF5>{GPIOA};
		else if constexpr (spiMOSI_ == SPI1_PB5_AF5)
			return GpioAlternate<GPIO_TypeDef, 5, AlternateFunction::AF5>{GPIOB};
		else if constexpr (spiMOSI_ == SPI2_PB15_AF5)
			return GpioAlternate<GPIO_TypeDef, 15, AlternateFunction::AF5>{GPIOB};
		else if constexpr (spiMOSI_ == SPI2_PC3_AF5)
			return GpioAlternate<GPIO_TypeDef, 3, AlternateFunction::AF5>{GPIOC};
		else if constexpr (spiMOSI_ == SPI2_PD4_AF5)
			return GpioAlternate<GPIO_TypeDef, 4, AlternateFunction::AF5>{GPIOD};
		else if constexpr (spiMOSI_ == SPI3_PB5_AF6)
			return GpioAlternate<GPIO_TypeDef, 5, AlternateFunction::AF6>{GPIOB};
		else
			return GpioAlternate<GPIO_TypeDef, 12, AlternateFunction::AF6>{GPIOC};
	}
public:
	ISpiPins& operator=(const ISpiPins& source) = delete;
	ISpiPins& operator=(ISpiPins&& source) = delete;
};
