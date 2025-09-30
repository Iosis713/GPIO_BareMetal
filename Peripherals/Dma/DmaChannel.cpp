#include "DmaChannel.hpp"
#include <algorithm>
#include <ranges>

DmaChannel::DmaChannel(volatile DMA_Channel_TypeDef* const channel_)
	: channel(channel_)
{
	assert(channel && "DMA channel must not be nullptr!");
	RCC->AHB1ENR |= RccAhb1enrDma();
}

void DmaChannel::Configure(const uint32_t peripheralAddress, const uint32_t memoryAddress, const uint32_t length, const uint8_t dmaRequest /*RM 11.6.7 - 4 bit*/, const DmaDirection dmaDir /*= DmaDirection::ReadFromPeripheralRX*/, const DmaMemoryPeripheralSize dmaSize /* = DmaMemoryPeripheralSize::bits_16*/)
{
	channel->CCR &= ~DMA_CCR_EN;
	channel->CPAR = peripheralAddress; //RM 11.6.5 Channel Peripheral Address Register
	channel->CMAR = memoryAddress; //RM 11.6.6
	channel->CNDTR = length; //RM 11.6.4 Channel x Number of Data to Transfer Register
	channel->CCR |= DMA_CCR_MINC; //memory increment enabled RM 11.6.3
	channel->CCR |= DMA_CCR_PL_1;
	if (dmaDir == DmaDirection::ReadFromPeripheralRX)
		channel->CCR &= ~(DMA_CCR_DIR);
	else
		channel->CCR |= DMA_CCR_DIR;
	channel->CCR |= (static_cast<uint8_t>(dmaSize) << DMA_CCR_PSIZE_Pos);
	channel->CCR |= (static_cast<uint8_t>(dmaSize) << DMA_CCR_MSIZE_Pos);

	const uint32_t bitShift = GetChannelIndex() - 1;
	constexpr uint8_t cselrMask = 0b0000;
	DmaCSELR()->CSELR &= (cselrMask << bitShift);
	DmaCSELR()->CSELR |= (dmaRequest << bitShift);
}

void DmaChannel::Enable() { channel->CCR |= DMA_CCR_EN; }
void DmaChannel::Disable() { channel->CCR &= ~DMA_CCR_EN; }

void DmaChannel::SetCircularMode()
{
	const bool wasDmaEnabled = channel->CCR & DMA_CCR_EN;
	if (wasDmaEnabled)
		Disable();

	channel->CCR |= DMA_CCR_CIRC; //circular mode

	if (wasDmaEnabled)
		Enable();
};

void DmaChannel::EnableInterruptTC()
{
	channel->CCR |= DMA_CCR_TCIE;

	if (channel == DMA1_Channel1) NVIC_EnableIRQ(DMA1_Channel1_IRQn);
	else if (channel == DMA1_Channel2) NVIC_EnableIRQ(DMA1_Channel2_IRQn);
	else if (channel == DMA1_Channel3) NVIC_EnableIRQ(DMA1_Channel3_IRQn);
	else if (channel == DMA1_Channel4) NVIC_EnableIRQ(DMA1_Channel4_IRQn);
	else if (channel == DMA1_Channel5) NVIC_EnableIRQ(DMA1_Channel5_IRQn);
	else if (channel == DMA1_Channel6) NVIC_EnableIRQ(DMA1_Channel6_IRQn);
	else if (channel == DMA1_Channel7) NVIC_EnableIRQ(DMA1_Channel7_IRQn);

	else if (channel == DMA2_Channel1) NVIC_EnableIRQ(DMA2_Channel1_IRQn);
	else if (channel == DMA2_Channel2) NVIC_EnableIRQ(DMA2_Channel2_IRQn);
	else if (channel == DMA2_Channel3) NVIC_EnableIRQ(DMA2_Channel3_IRQn);
	else if (channel == DMA2_Channel4) NVIC_EnableIRQ(DMA2_Channel4_IRQn);
	else if (channel == DMA2_Channel5) NVIC_EnableIRQ(DMA2_Channel5_IRQn);
	else if (channel == DMA2_Channel6) NVIC_EnableIRQ(DMA2_Channel6_IRQn);
	else NVIC_EnableIRQ(DMA2_Channel7_IRQn);
}

