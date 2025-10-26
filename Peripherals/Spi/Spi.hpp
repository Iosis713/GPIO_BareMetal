#pragma once
#include "Config.hpp"
#include "../Dma/DmaChannel.hpp"

//////////////////////////////////////////////////
//////////////////////////////////////////////////
//			NEED TO ADD ALSO DMA				//
//////////////////////////////////////////////////
//////////////////////////////////////////////////

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

template<typename T>
concept SpiStructure = requires(T spi)
{
	{ spi.CR1 } -> std::convertible_to<volatile uint32_t>;
	{ spi.CR2 } -> std::convertible_to<volatile uint32_t>;
	{ spi.SR } -> std::convertible_to<volatile uint32_t>;
	{ spi.DR } -> std::convertible_to<volatile uint32_t>;
	{ spi.CRCPR } -> std::convertible_to<volatile uint32_t>;
	{ spi.RXCRCR } -> std::convertible_to<volatile uint32_t>;
	{ spi.TXCRCR } -> std::convertible_to<volatile uint32_t>;
};

template<SpiStructure SpiStruct, SpiMode spiMode_>
class Spi
{
private:
	bool busy = false;
	const uint8_t* txBuffer = nullptr;
	std::size_t txSize = 0;
	std::size_t txIndex = 0;

	volatile bool txReadyFlag = false;
	volatile bool rxReadyFlag = false;

	inline void WaitUntilTXEIsEmpty() { while(!(spi->SR & SPI_SR_TXE));}
	inline void WriteData(const uint8_t data) { *reinterpret_cast<volatile uint8_t*>(&spi->DR) = data; }
	inline void WaitUntilRXNEIsNotEmpty() { while(!(spi->SR & SPI_SR_RXNE)); }
	inline uint8_t ReadData() { return *reinterpret_cast<volatile uint8_t*>(&spi->DR); }

	HalfDuplexDirection GetTransmissionDirection()
	{
		if (spi->CR1 & SPI_CR1_BIDIOE)
			return HalfDuplexDirection::Transmit;
		return HalfDuplexDirection::Receive;
	}

	void SetHalfDuplexDirection(const HalfDuplexDirection direction)
	{
		spi->CR1 &= ~(SPI_CR1_SPE);
		if (direction == HalfDuplexDirection::Receive)
			spi->CR1 &= SPI_CR1_BIDIOE;
		else if (direction == HalfDuplexDirection::Transmit)
			spi->CR1 |= SPI_CR1_BIDIOE;
		spi->CR1 |= SPI_CR1_SPE;
	}

	void EnableClock()
	{
		if (spi == SPI1)
			RCC->APB2ENR |= RCC_APB2ENR_SPI1EN; //RM 6.4.21 APB2 clock
		else if (spi == SPI2)
			RCC->APB1ENR1 |= RCC_APB1ENR1_SPI2EN; /*RM 6.4.19 enable spi2 clock*/
		else if (spi == SPI3)
			RCC->APB1ENR1 |= RCC_APB1ENR1_SPI3EN;
	}

	void ConfigureSPI()
	{
		//Reference manual 42.6.1
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
		if (spiMode == SpiMode::HalfDuplex)
		{
			//SPI2->CR1 &= ~(SPI_CR1_RXONLY); //Receive only mode enabled (0 for full duplex)
			spi->CR1 |= SPI_CR1_BIDIMODE; //Bidirectional data mode enable (2-line unidirectional data mode for full-duplex, 1 line for half-duplex)
			spi->CR1 |= SPI_CR1_BIDIOE; //BIDIOE output enable in bidirectional mode 0 - receive only, 1 - transmit only -- to set up for final class
		}
		else if (spiMode == SpiMode::FullDuplex)
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

	inline void EnableDmaTX() { spi->CR2 |= SPI_CR2_TXDMAEN; }
	inline void EnableDmaRX() { spi->CR2 |= SPI_CR2_RXDMAEN; }
	inline void EnableInterruptTX() { spi->CR2 |= SPI_CR2_TXEIE; }
	inline void DisableInterruptTX() { spi->CR2 &= ~SPI_CR2_TXEIE; }

	void IrqHandlerTX()
	{
		if (spi->SR & SPI_SR_TXE)
		{
			txReadyFlag = true;
			DisableInterruptTX();
		}
	}

	void IrqHandlerRX()
	{
		if (spi->SR & SPI_SR_RXNE)
		{
			rxReadyFlag = true;
			[[maybe_unused]] volatile uint8_t dummy = ReadData();
		}
	}

public:
	volatile SpiStruct* const spi = nullptr;
	static constexpr SpiMode spiMode = spiMode_;

	Spi(const Spi& source) = delete;
	Spi(Spi&& source) = delete;
	Spi& operator=(const Spi& source) = delete;
	Spi& operator=(Spi&& source) = delete;
	Spi() = delete;
	Spi(volatile SpiStruct* const spi_)
		: spi(spi_)
	{
		this->EnableClock();
		ConfigureSPI();
	}
	~Spi() = default;

	void Transmit(const uint8_t data)
	{
		WaitUntilTXEIsEmpty();
		WriteData(data);
		if constexpr (spiMode == SpiMode::FullDuplex)
			WaitUntilRXNEIsNotEmpty();
		[[maybe_unused]] volatile uint8_t dummyRead = ReadData();
	}

	void Transmit(const std::initializer_list<uint8_t> dataList)
	{
		for (const auto& value : dataList)
			Transmit(value);
	}

	void TransmitIT(const uint8_t value)
	{
		if (busy)
			return;

		busy = true;
		WriteData(value);
		EnableInterruptTX();
	}

	template<std::size_t N>
	void TransmitIT(const std::array<uint8_t, N>& data)
	{
		if (busy)
		{
			if (txReadyFlag)
			{
				if (txIndex < txSize)
				{
					WriteData(txBuffer[txIndex++]);
				}
				else
				{
					busy = false;
					txReadyFlag = false;
				}
			}
		}
		else
		{
			busy = true;
			txBuffer = data.data();
			txSize = data.size();
			txIndex = 0;
			txReadyFlag = true;
			EnableInterruptTX();
		}
	}

	void IRQHandlerTX()
	{
		if ((spi->SR & SPI_SR_TXE) && (spi->CR2 & SPI_CR2_TXEIE))
		{
			if constexpr (spiMode == SpiMode::FullDuplex)
			{
				IrqHandlerTX();
				IrqHandlerRX();
			}
			else
			{
				if (GetTransmissionDirection() == HalfDuplexDirection::Transmit)
					IrqHandlerTX();
				else
					IrqHandlerRX();
			}
		}
	}

	//to be changed
	void TransmitDma(DmaChannel& dma, volatile uint8_t* buffer, const std::size_t length, const uint8_t dmaRequest /*RM 11.6.7 - 4 bit*/)
	{
		if (buffer && IsReady())
		{	
			spi->CR1 &= ~SPI_CR1_SPE;
			dma.Configure(reinterpret_cast<uint32_t>(&spi->DR),
						  reinterpret_cast<uint32_t>(buffer),
						  length,
						  dmaRequest,
						  DmaMemoryPeripheralSize::bits_8);
			dma.EnableInterruptTC();
			EnableDmaTX();
			dma.Enable();
			spi->CR1 |= SPI_CR1_SPE;
		}
	}

	[[nodiscard]] uint8_t Receive()
	{
		//not configured for switching mode for half duplex yet
		if constexpr (spiMode == SpiMode::HalfDuplex)
		{
			if (GetTransmissionDirection() != HalfDuplexDirection::Receive)
				SetHalfDuplexDirection(HalfDuplexDirection::Receive);
		}

		WaitUntilTXEIsEmpty();
		WriteData(0x00); //dummy write to generate clock for receiving
		WaitUntilRXNEIsNotEmpty();
		return ReadData();
	}
	
	void EnableDma(DmaChannel& dma, volatile uint8_t* buffer, const std::size_t length, const uint8_t dmaRequest /*RM 11.6.7 - 4 bit*/, const HalfDuplexDirection direction)
	{
		if (buffer)
		{
			dma.Configure(reinterpret_cast<uint32_t>(&spi->DR),
						  reinterpret_cast<uint32_t>(buffer),
						  length,
						  dmaRequest,
						  DmaMemoryPeripheralSize::bits_8);

			if (direction == HalfDuplexDirection::Receive)
				EnableDmaRX();
			else
				EnableDmaTX();

			dma.EnableInterruptTC();
			dma.Enable();
		}
	}

	void SetInterruptPriority(const uint8_t priority)
	{
		IRQn_Type spiIrqn = SPI1_IRQn;
		if (spi == SPI2)
			spiIrqn = SPI2_IRQn;
		else if (spi == SPI3)
			spiIrqn = SPI3_IRQn;

		NVIC_EnableIRQ(spiIrqn);
		NVIC_SetPriority(spiIrqn, priority);
	}

	inline void DisableDmaTX() { spi->CR2 &= ~SPI_CR2_TXDMAEN; }
	inline void DisableDmaRX() { spi->CR2 &= ~SPI_CR2_RXDMAEN; }
	inline bool IsReady() { return !(spi->SR & SPI_SR_BSY); } 
};

