#pragma once
#include "../../Inc/Config.hpp"

//configured for dma1ch1 only right now
class DmaChannel 
{
protected:
public:
	volatile DMA_Channel_TypeDef* const channel = nullptr;
	DmaChannel(volatile DMA_Channel_TypeDef* const channel_)
	 	: channel(channel_)
	{
		RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
	}
		
	DmaChannel() = delete;
	~DmaChannel() = default;

	//REFACTOR REQUIRED!!!!!!
	//MORE GENERIC class to configure also other dma(1,2) and channels

	void Configure(uint32_t peripheralAddress, uint32_t memoryAddress, const uint32_t length)
	{
		channel->CCR &= ~DMA_CCR_EN;
		channel->CPAR = peripheralAddress; //RM 11.6.5 Channel Peripheral Address Register
		channel->CMAR = memoryAddress; //RM 11.6.6
		channel->CNDTR = length; //RM 11.6.4 Channel x Number of Data to Transfer Register

		channel->CCR |= DMA_CCR_MINC; //memory increment enabled RM 11.6.3
		channel->CCR |= DMA_CCR_CIRC; //circular mode
		channel->CCR |= DMA_CCR_PL_1;
		channel->CCR |= DMA_CCR_PSIZE_0; //Peripheral to memory - 16 bits data;
		channel->CCR |= DMA_CCR_MSIZE_0; //memory size 16 bits;

		DMA1_CSELR->CSELR &= ~DMA_CSELR_C1S; // clear C1S
	}

	void Enable() { channel->CCR |= DMA_CCR_EN; }
	void Disable() { channel->CCR &= ~DMA_CCR_EN; }
};