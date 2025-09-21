#pragma once
#include "../../Inc/Config.hpp"
#include <cassert>

class DmaChannel 
{
private:
	static constexpr bool IsDma1(volatile DMA_Channel_TypeDef* channel)
	{
		return (channel == DMA1_Channel1 || channel == DMA1_Channel2 || 
				channel == DMA1_Channel3 || channel == DMA1_Channel4 ||
				channel == DMA1_Channel5 || channel == DMA1_Channel6 ||
				channel == DMA1_Channel7) ? true : false;
	}

	static constexpr uint32_t RccAhb1enrDma(volatile DMA_Channel_TypeDef* channel)
	{
		return (IsDma1(channel))? RCC_AHB1ENR_DMA1EN : RCC_AHB1ENR_DMA2EN;
	}

	static constexpr DMA_Request_TypeDef* DmaCSELR(volatile DMA_Channel_TypeDef* channel)
	{
		return (IsDma1(channel)) ? DMA1_CSELR : DMA2_CSELR;
	}

	static constexpr std::size_t GetChannelIndex(volatile DMA_Channel_TypeDef* channel)
	{
		if (channel == DMA1_Channel1 || channel == DMA2_Channel1) return 1;
		if (channel == DMA1_Channel2 || channel == DMA2_Channel2) return 2;
		if (channel == DMA1_Channel3 || channel == DMA2_Channel3) return 3;
		if (channel == DMA1_Channel4 || channel == DMA2_Channel4) return 4;
		if (channel == DMA1_Channel5 || channel == DMA2_Channel5) return 5;
		if (channel == DMA1_Channel6 || channel == DMA2_Channel6) return 6;
		return 7;
	}

public:
	volatile DMA_Channel_TypeDef* const channel = nullptr;
	DmaChannel(volatile DMA_Channel_TypeDef* const channel_);
		
	DmaChannel() = delete;
	~DmaChannel() = default;

	void Configure(const uint32_t peripheralAddress, const uint32_t memoryAddress, const uint32_t length, const uint8_t dmaRequest /*RM 11.6.7 - 4 bit*/);
	void Enable();
	void Disable();
};