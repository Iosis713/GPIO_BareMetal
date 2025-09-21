#pragma once
#include "../../Inc/Config.hpp"
#include <cassert>

//configured for dma1ch1 only right now
class DmaChannel 
{
private:
	static constexpr uint32_t RccAhb1enrDma(volatile DMA_Channel_TypeDef* channel)
	{
		return (channel == DMA1_Channel1 || channel == DMA1_Channel2 || 
				channel == DMA1_Channel3 || channel == DMA1_Channel4 ||
				channel == DMA1_Channel5 || channel == DMA1_Channel6 ||
				channel == DMA1_Channel7) ? RCC_AHB1ENR_DMA1EN : RCC_AHB1ENR_DMA2EN;
	}

public:
	volatile DMA_Channel_TypeDef* const channel = nullptr;
	DmaChannel(volatile DMA_Channel_TypeDef* const channel_);
		
	DmaChannel() = delete;
	~DmaChannel() = default;

	//REFACTOR REQUIRED!!!!!!
	//MORE GENERIC class to configure also other dma(1,2) and channels

	void Configure(const uint32_t peripheralAddress, const uint32_t memoryAddress, const uint32_t length);

	void Enable();
	void Disable();
};