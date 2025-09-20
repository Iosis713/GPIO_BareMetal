#pragma once
#include "../../Inc/Config.hpp"
#include <cassert>

//configured for dma1ch1 only right now
class DmaChannel 
{
protected:
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