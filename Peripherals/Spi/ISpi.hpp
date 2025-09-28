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
