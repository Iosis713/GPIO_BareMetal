#pragma once
#include "../../Inc/Config.hpp"
#include <cassert>

/*

TO DO
Add CCR_DIR selection (runtime) to configure memory->peripheral and peripheral->memory data transfer
as it is required for SPI

*/
enum class DmaDirection
{
	ReadFromPeripheralRX,
	ReadFromMemoryTX
};

enum class DmaMemoryPeripheralSize : uint8_t
{
	bits_8 = 0b00, //RM 11.6.3
	bits_16 = 0b01,
	bits_32 = 0b10
};

class DmaChannel 
{
private:
	constexpr bool IsDma1()
	{
		return (channel == DMA1_Channel1 || channel == DMA1_Channel2 || 
				channel == DMA1_Channel3 || channel == DMA1_Channel4 ||
				channel == DMA1_Channel5 || channel == DMA1_Channel6 ||
				channel == DMA1_Channel7) ? true : false;
	}

	constexpr uint32_t RccAhb1enrDma()
	{
		return (IsDma1())? RCC_AHB1ENR_DMA1EN : RCC_AHB1ENR_DMA2EN;
	}

	constexpr DMA_Request_TypeDef* DmaCSELR()
	{
		return (IsDma1()) ? DMA1_CSELR : DMA2_CSELR;
	}

	constexpr std::size_t GetChannelIndex() const
	{
		if (channel == DMA1_Channel1 || channel == DMA2_Channel1) return 1;
		if (channel == DMA1_Channel2 || channel == DMA2_Channel2) return 2;
		if (channel == DMA1_Channel3 || channel == DMA2_Channel3) return 3;
		if (channel == DMA1_Channel4 || channel == DMA2_Channel4) return 4;
		if (channel == DMA1_Channel5 || channel == DMA2_Channel5) return 5;
		if (channel == DMA1_Channel6 || channel == DMA2_Channel6) return 6;
		return 7;
	}

	void Init(const DmaDirection dmaDir = DmaDirection::ReadFromPeripheralRX);

public:
	volatile DMA_Channel_TypeDef* const channel = nullptr;
	DmaChannel(volatile DMA_Channel_TypeDef* const channel_, const DmaDirection direction);
		
	DmaChannel() = delete;
	~DmaChannel() = default;

	void Configure(const uint32_t peripheralAddress, const uint32_t memoryAddress, const uint32_t length, const uint8_t dmaRequest /*RM 11.6.7 - 4 bit*/, const DmaMemoryPeripheralSize dmaSize = DmaMemoryPeripheralSize::bits_16);
	void Enable();
	void Disable();
	void SetCircularMode();

	void EnableInterruptTC(); // TC-Transfer Complete

	[[nodiscard]] bool TransferComplete() const noexcept
	{
		return ( DMA1->ISR & (DMA_ISR_TCIF1 << ((GetChannelIndex() - 1) * 4)));
	} 

	void ClearTransferCompleteFlag() noexcept
	{
		DMA1->IFCR = (DMA_IFCR_CTCIF1 << (GetChannelIndex() - 1) * 4);		
	}
};