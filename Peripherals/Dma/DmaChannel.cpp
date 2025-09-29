#include "DmaChannel.hpp"
#include <algorithm>
#include <ranges>

DmaChannel::DmaChannel(volatile DMA_Channel_TypeDef* const channel_)
	: channel(channel_)
{
	assert(channel && "DMA channel must not be nullptr!");
	RCC->AHB1ENR |= RccAhb1enrDma(channel);
}

void DmaChannel::Configure(const uint32_t peripheralAddress, const uint32_t memoryAddress, const uint32_t length, const uint8_t dmaRequest /*RM 11.6.7 - 4 bit*/)
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

	const uint32_t bitShift = GetChannelIndex(channel) - 1;
	constexpr uint8_t cselrMask = 0b0000;
	DmaCSELR(channel)->CSELR &= cselrMask << bitShift;
	DmaCSELR(channel)->CSELR |= dmaRequest << bitShift;
}

void DmaChannel::Enable() { channel->CCR |= DMA_CCR_EN; }
void DmaChannel::Disable() { channel->CCR &= ~DMA_CCR_EN; }
