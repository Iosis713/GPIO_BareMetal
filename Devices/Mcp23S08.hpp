#pragma once

#include <cstdint>
#include "../Peripherals/Spi/Spi.hpp"
#include "../Peripherals/Gpio/GpioOutput.hpp"
#include "../Peripherals/Dma/DmaChannel.hpp"
#include "../Peripherals/Dma/DmaRequest.hpp"

enum class MCP23S08Reg : uint8_t
{
	IODIR = 0x00,
	IPOL = 0x01,
	GPINTEN = 0x02,
	DEFVAL	= 0x03,
	INTCON	= 0x04,
	IOCON= 0x05,
	GPPU = 0x06,
	INTF = 0x07,
	INTCAP	= 0x08,
	GPIO = 0x09,
	OLAT = 0x0A,
};

namespace MCP23S08 {
static constexpr uint8_t GP0 = 0x01;
static constexpr uint8_t GP1 = 0x02;
static constexpr uint8_t GP2 = 0x04;
static constexpr uint8_t GP3 = 0x08;
static constexpr uint8_t GP4 = 0x10;
static constexpr uint8_t GP5 = 0x20;
static constexpr uint8_t GP6 = 0x40;
static constexpr uint8_t GP7 = 0x80;
};

template<typename T>
concept GpioOutputConcept = requires (T gpio)
{
	gpio.Clear();
	gpio.Set();
};

template<typename T>
concept SpiConcept = requires(T spi, uint8_t value)
{
	spi.Transmit(value);
	spi.Receive();
};

template <GpioOutputConcept GPIO, SpiConcept SPI>
class Mcp23S08
{
private:
	GPIO& csLine;
	SPI& spi;

public:
	Mcp23S08() = delete;
	Mcp23S08(const Mcp23S08& source) = delete;
	Mcp23S08(Mcp23S08&& source) = delete;
	Mcp23S08& operator=(const Mcp23S08& source) = delete;
	Mcp23S08& operator=(Mcp23S08&& source) = delete;
	~Mcp23S08() = default;
	
	Mcp23S08(GPIO& csLine_, SPI& spi_)
		: csLine(csLine_)
		, spi(spi_)
		{};

	void Write(const MCP23S08Reg reg, const uint8_t value)
	{
		csLine.Clear();
		spi.Transmit(0x40);
		spi.Transmit(static_cast<uint8_t>(reg));
		spi.Transmit(value);
		csLine.Set();
	}

	uint8_t Read(const MCP23S08Reg reg)
	{
		csLine.Clear();
		spi.Transmit(0x41); //opcode for read (R/W = 1)
		spi.Transmit(static_cast<uint8_t>(reg)); //register address
		volatile const uint8_t received = spi.Receive();
		csLine.Set();
		return received;
	}
};

template<GpioOutputConcept Gpio, SpiConcept Spi>
void McpWriteRegisterDma(Gpio& CSline, Spi& spi, DmaChannel& dmaTx, const uint8_t reg, const uint8_t value, const DMA1Request dmaRequest)
{
	static uint8_t txBuffer[3] { 0x40, reg, value };
	if (spi.IsReady())
	{
		CSline.Clear();
		dmaTx.Enable();
		spi.EnableDma(dmaTx, txBuffer, 3, static_cast<uint8_t>(dmaRequest), HalfDuplexDirection::Transmit);
		spi.TransmitDma(dmaTx, txBuffer, 3, static_cast<uint8_t>(dmaRequest));
	}
}

template<GpioOutputConcept Gpio, SpiConcept Spi>
void McpWriteRegisterDma_IRQHandler(Gpio& CSline, Spi& spi, DmaChannel& dmaTx)
{
	if (dmaTx.TransferComplete())
	{
		dmaTx.ClearTransferCompleteFlag();
		dmaTx.Disable();
		spi.DisableDmaTX();
		while(!spi.IsReady());
		CSline.Set();
	}
}
