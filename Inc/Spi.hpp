#pragma once
#ifndef SPI_HPP_
#define SPI_HPP_

#include "Config.hpp"
#include "../Peripherals/Gpio/GpioAlternate.hpp"

//just for spi 2 right now, according to:
//https://forbot.pl/blog/kurs-stm32l4-ekspander-portow-spi-quiz-id47763

//////////////////////////////////////////////////
//////////////////////////////////////////////////
//			NEED TO ADD ALSO DMA				//
//////////////////////////////////////////////////
//////////////////////////////////////////////////

void SpiConfigHalfDuplex();
void SpiConfigFullDuplex();
void EnableSpiClocks();
void Spi2Transmit(const uint8_t data);
void Spi2TransmitHalfDuplex(const uint8_t data);
uint8_t Spi2Receive();

enum class SpiMode
{
	HalfDuplex,
	FullDuplex
};

enum class HalfDuplexDirection
{
	Receive,
	Transmit
};

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
			return GpioAlternate<GPIOA_BASE, 5, AlternateFunction::AF5>{};
		else if constexpr (spiSCK_ == SPI1_PB3_AF5)
			return GpioAlternate<GPIOB_BASE, 3, AlternateFunction::AF5>{};
		else if constexpr (spiSCK_ == SPI2_PB10_AF5)
			return GpioAlternate<GPIOB_BASE, 10, AlternateFunction::AF5>{};
		else if constexpr (spiSCK_ == SPI2_PD1_AF5)
			return GpioAlternate<GPIOD_BASE, 1, AlternateFunction::AF5>{};
		else if constexpr (spiSCK_ == SPI3_PB3_AF6)
			return GpioAlternate<GPIOB_BASE, 3, AlternateFunction::AF6>{};
		else
			return GpioAlternate<GPIOC_BASE, 10, AlternateFunction::AF6>{};
	}

	template<SpiMISO spiMISO_>
	static constexpr auto ConfigureMISO()
	{
		using enum SpiMISO;
		if constexpr (spiMISO_ == SPI1_PA6_AF5)
			return GpioAlternate<GPIOA_BASE, 6, AlternateFunction::AF5>{};
		else if constexpr (spiMISO_ == SPI1_PB4_AF5)
			return GpioAlternate<GPIOB_BASE, 4, AlternateFunction::AF5>{};
		else if constexpr (spiMISO_ == SPI2_PB14_AF5)
			return GpioAlternate<GPIOB_BASE, 14, AlternateFunction::AF5>{};
		else if constexpr (spiMISO_ == SPI2_PC2_AF5)
			return GpioAlternate<GPIOC_BASE, 2, AlternateFunction::AF5>{};
		else if constexpr (spiMISO_ == SPI2_PD3_AF5)
			return GpioAlternate<GPIOD_BASE, 3, AlternateFunction::AF5>{};
		else if constexpr (spiMISO_ == SPI3_PB4_AF6)
			return GpioAlternate<GPIOB_BASE, 4, AlternateFunction::AF6>{};
		else
			return GpioAlternate<GPIOC_BASE, 11, AlternateFunction::AF6>{};
	}

	template<SpiMOSI spiMOSI_>
	static constexpr auto ConfigureMOSI()
	{
		using enum SpiMOSI;
		if constexpr (spiMOSI_ == SPI1_PA7_AF5)
			return GpioAlternate<GPIOA_BASE, 7, AlternateFunction::AF5>{};
		else if constexpr (spiMOSI_ == SPI1_PB5_AF5)
			return GpioAlternate<GPIOB_BASE, 5, AlternateFunction::AF5>{};
		else if constexpr (spiMOSI_ == SPI2_PB15_AF5)
			return GpioAlternate<GPIOB_BASE, 15, AlternateFunction::AF5>{};
		else if constexpr (spiMOSI_ == SPI2_PC3_AF5)
			return GpioAlternate<GPIOC_BASE, 3, AlternateFunction::AF5>{};
		else if constexpr (spiMOSI_ == SPI2_PD4_AF5)
			return GpioAlternate<GPIOD_BASE, 4, AlternateFunction::AF5>{};
		else if constexpr (spiMOSI_ == SPI3_PB5_AF6)
			return GpioAlternate<GPIOB_BASE, 5, AlternateFunction::AF6>{};
		else
			return GpioAlternate<GPIOC_BASE, 12, AlternateFunction::AF6>{};
	}
public:
	//ISpiPins(const ISpiPins& source) = delete;
	//ISpiPins(ISpiPins&& source) = delete;
	ISpiPins& operator=(const ISpiPins& source) = delete;
	ISpiPins& operator=(ISpiPins&& source) = delete;
};

template<SpiSCK spiSCK_, SpiMISO spiMISO_, SpiMOSI spiMOSI_>
class SpiPinsFullDuplex : public ISpiPins
{
protected:
public:
	SpiPinsFullDuplex(const SpiPinsFullDuplex& source) = delete;
	SpiPinsFullDuplex(SpiPinsFullDuplex&& source) = delete;
	SpiPinsFullDuplex& operator=(const SpiPinsFullDuplex& source) = delete;
	SpiPinsFullDuplex& operator=(SpiPinsFullDuplex&& source) = delete;
	SpiPinsFullDuplex() = default;

	static constexpr auto sck = ConfigureSCK<spiSCK_>();
	static constexpr auto miso = ConfigureMISO<spiMISO_>();
	static constexpr auto mosi = ConfigureMOSI<spiMOSI_>();
};

template<SpiSCK spiSCK_, SpiMOSI spiMOSI_>
class SpiPinsHalfDuplexTX : public ISpiPins
{
protected:
public:
	SpiPinsHalfDuplexTX(const SpiPinsHalfDuplexTX& source) = delete;
	SpiPinsHalfDuplexTX(SpiPinsHalfDuplexTX&& source) = delete;
	SpiPinsHalfDuplexTX& operator=(const SpiPinsHalfDuplexTX& source) = delete;
	SpiPinsHalfDuplexTX& operator=(SpiPinsHalfDuplexTX&& source) = delete;
	SpiPinsHalfDuplexTX() = default;

	static const inline auto sck = ConfigureSCK<spiSCK_>();
	static const inline auto mosi = ConfigureMOSI<spiMOSI_>();
};

template<SpiSCK spiSCK_, SpiMOSI spiMISO_>
class SpiPinsHalfDuplexRX : public ISpiPins
{
protected:
public:
	SpiPinsHalfDuplexRX(const SpiPinsHalfDuplexRX& source) = delete;
	SpiPinsHalfDuplexRX(SpiPinsHalfDuplexRX&& source) = delete;
	SpiPinsHalfDuplexRX& operator=(const SpiPinsHalfDuplexRX& source) = delete;
	SpiPinsHalfDuplexRX& operator=(SpiPinsHalfDuplexRX&& source) = delete;
	SpiPinsHalfDuplexRX() = default;

	static constexpr auto sck = ConfigureSCK<spiSCK_>();
	static constexpr auto miso = ConfigureMOSI<spiMISO_>();
};

template<typename Derived>
class ISpi
{
protected:
	inline volatile SPI_TypeDef* SPI() const { return reinterpret_cast<SPI_TypeDef*>(Derived::spiAddr); }
	inline void WaitUntilTXEIsEmpty() { while(!(SPI()->SR & SPI_SR_TXE));}
	inline void WriteData(const uint8_t data) { *reinterpret_cast<volatile uint8_t*>(&SPI()->DR) = data; }
	inline void WaitUntilRXNEIsNotEmpty() { while(!(SPI()->SR & SPI_SR_RXNE)); }
	inline uint8_t ReadData() { return *reinterpret_cast<volatile uint8_t*>(&SPI()->DR); }

	HalfDuplexDirection GetTransmissionDirection()
	{
		if (SPI()->CR1 & SPI_CR1_BIDIOE)
			return HalfDuplexDirection::Transmit;
		return HalfDuplexDirection::Receive;
	}

	void SetHalfDuplexDirection(const HalfDuplexDirection direction)
	{
		auto spi = SPI();
		spi->CR1 &= ~(SPI_CR1_SPE);
		if (direction == HalfDuplexDirection::Receive)
			spi->CR1 &= SPI_CR1_BIDIOE;
		else if (direction == HalfDuplexDirection::Transmit)
			spi->CR1 |= SPI_CR1_BIDIOE;
		spi->CR1 |= SPI_CR1_SPE;
	}

public:
	void EnableClock()
	{
		if (Derived::spiAddr == SPI1_BASE)
			RCC->APB2ENR |= RCC_APB2ENR_SPI1EN; //RM 6.4.21 APB2 clock
		else if (Derived::spiAddr == SPI2_BASE)
			RCC->APB1ENR1 |= RCC_APB1ENR1_SPI2EN; /*RM 6.4.19 enable spi2 clock*/
		else if (Derived::spiAddr == SPI3_BASE)
			RCC->APB1ENR1 |= RCC_APB1ENR1_SPI3EN;
	}

	void ConfigureSPI()
	{
		//Reference manual 42.6.1
		auto spi = this->SPI();
		spi->CR1 &= ~(SPI_CR1_SPE); //disable before configuration
		//SPI2->CR1 |= SPI_CR1_SPE;
		//these two should be seen in MCP23S08 documentation
		spi->CR1 &= ~SPI_CR1_CPOL;      // CPOL = 0 (clock idle low)
		spi->CR1 &= ~SPI_CR1_CPHA;      // CPHA = 0 (capture on 1st edge)
		spi->CR1 |= SPI_CR1_MSTR; //select as master (micro-controller as a master)
		spi->CR1 |= SPI_CR1_SSI; //internal slave select (1 - software slave managemenet enabled (0 for disabled)
		spi->CR1 |= SPI_CR1_SSM; //software slave management enabled
		//BR - baud rate prescaler - 000 --> fPCLK/2
		spi->CR1 &= ~SPI_CR1_BR; //000 for fPCLK/2
		spi->CR1 &= ~(SPI_CR1_CRCEN); //hardware crc calculation disabled
		spi->CR1 &= ~SPI_CR1_CRCL; //CRC lenght (0 -> 8 bit, 1 -> 16 bit

		///////////////////////////////////////////////////
		//// FRAME SELECTION (MOTOROLA - MSB)    //////////
		//MSB Most significant bit / LSB - least significant bit
		spi->CR1 &= ~(SPI_CR1_LSBFIRST); //0 for MSB, 1 for LSB

		spi->CR1 &= ~(SPI_CR1_BIDIMODE); //Bidirectional data mode enable (2-line unidirectional data mode for full-duplex, 1 line for half-duplex)
		if (Derived::spiMode == SpiMode::HalfDuplex)
		{
			//SPI2->CR1 &= ~(SPI_CR1_RXONLY); //Receive only mode enabled (0 for full duplex)
			spi->CR1 |= SPI_CR1_BIDIMODE; //Bidirectional data mode enable (2-line unidirectional data mode for full-duplex, 1 line for half-duplex)
			spi->CR1 |= SPI_CR1_BIDIOE; //BIDIOE output enable in bidirectional mode 0 - receive only, 1 - transmit only -- to set up for final class
		}
		else if (Derived::spiMode == SpiMode::FullDuplex)
		{
			//bidimode already 0 by default;
			spi->CR1 &= ~(SPI_CR1_RXONLY); //Receive only mode enabled (0 for full duplex)
		}
		spi->CR2 &= ~SPI_CR2_DS; //clear all bits
		spi->CR2 |= (0b0111 << SPI_CR2_DS_Pos); //0111 for 8 bit
		spi->CR2 |= SPI_CR2_FRXTH; //RXNE event on 8-bit
		spi->CR2 &= ~(SPI_CR2_FRF);
		spi->CR2 &= ~(SPI_CR2_SSOE);
		spi->CR2 &= ~(SPI_CR2_NSSP);
		spi->CR1 |= SPI_CR1_SPE; //spi enabled
	}

	void Transmit(const uint8_t data)
	{
		WaitUntilTXEIsEmpty();
		WriteData(data);
		if constexpr (Derived::spiMode == SpiMode::FullDuplex)
			WaitUntilRXNEIsNotEmpty();
		[[maybe_unused]] volatile uint8_t dummyRead = ReadData();
	}

	[[nodiscard]] uint8_t Receive()
	{
		//not configured for switching mode for half duplex yet
		if constexpr (Derived::spiMode == SpiMode::HalfDuplex)
		{
			if (GetTransmissionDirection() != HalfDuplexDirection::Receive)
				SetHalfDuplexDirection(HalfDuplexDirection::Receive);
		}

		WaitUntilTXEIsEmpty();
		WriteData(0x00); //dummy write to generate clock for receiving
		WaitUntilRXNEIsNotEmpty();
		return ReadData();
	}
};

template<std::uintptr_t spiAddr_, SpiMode spiMode_>
class Spi : public ISpi<Spi<spiAddr_, spiMode_>>
{
protected:

public:
	static constexpr std::uintptr_t spiAddr = spiAddr_;
	static constexpr SpiMode spiMode = spiMode_;

	Spi(const Spi& source) = delete;
	Spi(Spi&& source) = delete;
	Spi& operator=(const Spi& source) = delete;
	Spi& operator=(Spi&& source) = delete;
	Spi()
	{
		this->EnableClock();
		this->ConfigureSPI();
	}
	~Spi() = default;
};

#endif /* SPI_HPP_ */
