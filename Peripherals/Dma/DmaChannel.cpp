#include "DmaChannel.hpp"

DmaChannel::DmaChannel(volatile DMA_Channel_TypeDef* const channel_)
	: channel(channel_)
{
	assert(channel && "DMA channel must not be nullptr!");
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
}

void DmaChannel::Configure(const uint32_t peripheralAddress, const uint32_t memoryAddress, const uint32_t length)
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

void DmaChannel::Enable() { channel->CCR |= DMA_CCR_EN; }
void DmaChannel::Disable() { channel->CCR &= ~DMA_CCR_EN; }