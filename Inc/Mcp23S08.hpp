/*
 * Mcp23S08.hpp
 *
 *  Created on: Jun 28, 2025
 *      Author: bartoszlozinski
 */

#pragma once
#ifndef MCP23S08_HPP_
#define MCP23S08_HPP_

#include <cstdint>
#include "Spi.hpp"

namespace MCP23S08 {
static constexpr uint8_t IODIR = 0x00;
static constexpr uint8_t IPOL = 0x01;
static constexpr uint8_t GPINTEN = 0x02;
static constexpr uint8_t DEFVAL	= 0x03;
static constexpr uint8_t INTCON	= 0x04;
static constexpr uint8_t IOCON= 0x05;
static constexpr uint8_t GPPU = 0x06;
static constexpr uint8_t INTF = 0x07;
static constexpr uint8_t INTCAP	= 0x08;
static constexpr uint8_t GPIO = 0x09;
static constexpr uint8_t OLAT = 0x0A;

static constexpr uint8_t GP0 = 0x01;
static constexpr uint8_t GP1 = 0x02;
static constexpr uint8_t GP2 = 0x04;
static constexpr uint8_t GP3 = 0x08;
static constexpr uint8_t GP4 = 0x10;
static constexpr uint8_t GP5 = 0x20;
static constexpr uint8_t GP6 = 0x40;
static constexpr uint8_t GP7 = 0x80;
};

//abbreviated template
void McpWriteRegister(auto& CSline /*GpioOutput*/, auto& spi, const uint8_t reg, const uint8_t value)
{
	CSline.Clear(); // select MCP
	spi.Transmit(0x40);
	spi.Transmit(reg);
	spi.Transmit(value);
	CSline.Set();
}

uint8_t McpReadRegister(auto& CSline /*GpioOutput*/, auto& spi, const uint8_t reg)
{
	CSline.Clear();
	spi.Transmit(0x41); //opcode for read (R/W = 1)
	spi.Transmit(reg); //register address
	volatile const uint8_t received = spi.Receive();
	CSline.Set();
	return received;
}

#endif /* MCP23S08_HPP_ */
