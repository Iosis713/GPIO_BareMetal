#pragma once

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

template<typename Derived>
class ISpi
{
protected:
	inline void WaitUntilTXEIsEmpty() { while(!(static_cast<Derived*>(this)->spi->SR & SPI_SR_TXE));}
	inline void WriteData(const uint8_t data) { *reinterpret_cast<volatile uint8_t*>(&(static_cast<Derived*>(this))->spi->DR) = data; }
	inline void WaitUntilRXNEIsNotEmpty() { while(!(static_cast<Derived*>(this)->spi->SR & SPI_SR_RXNE)); }
	inline uint8_t ReadData() { return *reinterpret_cast<volatile uint8_t*>(&(static_cast<Derived*>(this))->spi->DR); }

	HalfDuplexDirection GetTransmissionDirection()
	{
		if (static_cast<Derived*>(this)->spi->CR1 & SPI_CR1_BIDIOE)
			return HalfDuplexDirection::Transmit;
		return HalfDuplexDirection::Receive;
	}

	void SetHalfDuplexDirection(const HalfDuplexDirection direction)
	{
		static_cast<Derived*>(this)->spi->CR1 &= ~(SPI_CR1_SPE);
		if (direction == HalfDuplexDirection::Receive)
			static_cast<Derived*>(this)->spi->CR1 &= SPI_CR1_BIDIOE;
		else if (direction == HalfDuplexDirection::Transmit)
			static_cast<Derived*>(this)->spi->CR1 |= SPI_CR1_BIDIOE;
		static_cast<Derived*>(this)->spi->CR1 |= SPI_CR1_SPE;
	}

public:
	void EnableClock()
	{
		if (static_cast<Derived*>(this)->spi == SPI1)
			RCC->APB2ENR |= RCC_APB2ENR_SPI1EN; //RM 6.4.21 APB2 clock
		else if (static_cast<Derived*>(this)->spi == SPI2)
			RCC->APB1ENR1 |= RCC_APB1ENR1_SPI2EN; /*RM 6.4.19 enable spi2 clock*/
		else if (static_cast<Derived*>(this)->spi == SPI3)
			RCC->APB1ENR1 |= RCC_APB1ENR1_SPI3EN;
	}

	void ConfigureSPI()
	{
		//Reference manual 42.6.1
		static_cast<Derived*>(this)->spi->CR1 &= ~(SPI_CR1_SPE); //disable before configuration
		//SPI2->CR1 |= SPI_CR1_SPE;
		//these two should be seen in MCP23S08 documentation
		static_cast<Derived*>(this)->spi->CR1 &= ~SPI_CR1_CPOL;      // CPOL = 0 (clock idle low)
		static_cast<Derived*>(this)->spi->CR1 &= ~SPI_CR1_CPHA;      // CPHA = 0 (capture on 1st edge)
		static_cast<Derived*>(this)->spi->CR1 |= SPI_CR1_MSTR; //select as master (micro-controller as a master)
		static_cast<Derived*>(this)->spi->CR1 |= SPI_CR1_SSI; //internal slave select (1 - software slave managemenet enabled (0 for disabled)
		static_cast<Derived*>(this)->spi->CR1 |= SPI_CR1_SSM; //software slave management enabled
		//BR - baud rate prescaler - 000 --> fPCLK/2
		static_cast<Derived*>(this)->spi->CR1 &= ~SPI_CR1_BR; //000 for fPCLK/2
		static_cast<Derived*>(this)->spi->CR1 &= ~(SPI_CR1_CRCEN); //hardware crc calculation disabled
		static_cast<Derived*>(this)->spi->CR1 &= ~SPI_CR1_CRCL; //CRC lenght (0 -> 8 bit, 1 -> 16 bit

		///////////////////////////////////////////////////
		//// FRAME SELECTION (MOTOROLA - MSB)    //////////
		//MSB Most significant bit / LSB - least significant bit
		static_cast<Derived*>(this)->spi->CR1 &= ~(SPI_CR1_LSBFIRST); //0 for MSB, 1 for LSB

		static_cast<Derived*>(this)->spi->CR1 &= ~(SPI_CR1_BIDIMODE); //Bidirectional data mode enable (2-line unidirectional data mode for full-duplex, 1 line for half-duplex)
		if (Derived::spiMode == SpiMode::HalfDuplex)
		{
			//SPI2->CR1 &= ~(SPI_CR1_RXONLY); //Receive only mode enabled (0 for full duplex)
			static_cast<Derived*>(this)->spi->CR1 |= SPI_CR1_BIDIMODE; //Bidirectional data mode enable (2-line unidirectional data mode for full-duplex, 1 line for half-duplex)
			static_cast<Derived*>(this)->spi->CR1 |= SPI_CR1_BIDIOE; //BIDIOE output enable in bidirectional mode 0 - receive only, 1 - transmit only -- to set up for final class
		}
		else if (Derived::spiMode == SpiMode::FullDuplex)
		{
			//bidimode already 0 by default;
			static_cast<Derived*>(this)->spi->CR1 &= ~(SPI_CR1_RXONLY); //Receive only mode enabled (0 for full duplex)
		}
		static_cast<Derived*>(this)->spi->CR2 &= ~SPI_CR2_DS; //clear all bits
		static_cast<Derived*>(this)->spi->CR2 |= (0b0111 << SPI_CR2_DS_Pos); //0111 for 8 bit
		static_cast<Derived*>(this)->spi->CR2 |= SPI_CR2_FRXTH; //RXNE event on 8-bit
		static_cast<Derived*>(this)->spi->CR2 &= ~(SPI_CR2_FRF);
		static_cast<Derived*>(this)->spi->CR2 &= ~(SPI_CR2_SSOE);
		static_cast<Derived*>(this)->spi->CR2 &= ~(SPI_CR2_NSSP);
		static_cast<Derived*>(this)->spi->CR1 |= SPI_CR1_SPE; //spi enabled
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
