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

//abbreviated template
void McpWriteRegister(auto& CSline /*GpioOutput*/, const uint8_t reg, const uint8_t value)
{
	CSline.Clear(); // select MCP
	Spi2Transmit(0x40); //opcode for write to MCP23S08 (A2:A0 = 000);
	Spi2Transmit(reg); //register address
	Spi2Transmit(value); //data
	CSline.Set();
}

uint8_t McpReadRegister(auto& CSline /*GpioOutput*/, const uint8_t reg)
{
	CSline.Clear();
	Spi2Transmit(0x41); //opcode for read (R/W = 1)
	Spi2Transmit(reg); //register address
	volatile const uint8_t received = Spi2Receive();
	CSline.Set();
	return received;
}

#endif /* MCP23S08_HPP_ */
