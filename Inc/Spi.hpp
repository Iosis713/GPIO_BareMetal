/*
 * Spi.hpp
 *
 *  Created on: Jun 23, 2025
 *      Author: bartoszlozinski
 */

#ifndef SPI_HPP_
#define SPI_HPP_

#include "Config.hpp"
#include "Gpio.hpp"

//just for spi 2 right now, according to:
//https://forbot.pl/blog/kurs-stm32l4-ekspander-portow-spi-quiz-id47763

//PC3 - MOSI
//PC2 - MISO
//PB

static constexpr uint8_t MCP_IODIR = 0x00;
static constexpr uint8_t MCP_IPOL = 0x01;
static constexpr uint8_t MCP_GPINTEN = 0x02;
static constexpr uint8_t MCP_DEFVAL	= 0x03;
static constexpr uint8_t MCP_INTCON	= 0x04;
static constexpr uint8_t MCP_IOCON= 0x05;
static constexpr uint8_t MCP_GPPU = 0x06;
static constexpr uint8_t MCP_INTF = 0x07;
static constexpr uint8_t MCP_INTCAP	= 0x08;
static constexpr uint8_t MCP_GPIO = 0x09;
static constexpr uint8_t MCP_OLAT = 0x0A;

void SpiConfig();
void EnableSpiClocks();
void Spi2Transmit(const uint8_t data);
uint8_t Spi2Receive();

#endif /* SPI_HPP_ */
