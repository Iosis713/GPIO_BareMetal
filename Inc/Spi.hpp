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
/*
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
static constexpr uint8_t MCP_OLAT = 0x0A;*/

#define MCP_IODIR  0x00
#define MCP_IPOL  0x01
#define MCP_GPINTEN  0x02
#define MCP_DEFVAL	 0x03
#define MCP_INTCON	 0x04
#define MCP_IOCON 0x05
#define MCP_GPPU  0x06
#define MCP_INTF  0x07
#define MCP_INTCAP	0x08
#define MCP_GPIO  0x09
#define MCP_OLAT  0x0A


void SpiConfig()
{
	//Reference manual 42.6.1

	//SPI2->CR1 &= ~(SPI_CR1_SPE); //disable before configuration
	//SPI2->CR1 |= SPI_CR1_SPE;
	//these two should be seen in MCP23S08 documentation
	SPI2->CR1 &= ~SPI_CR1_CPOL;      // CPOL = 0 (clock idle low)
	SPI2->CR1 &= ~SPI_CR1_CPHA;      // CPHA = 0 (capture on 1st edge)
	SPI2->CR1 |= SPI_CR1_MSTR; //select as master (micro-controller as a master)
	SPI2->CR1 |= SPI_CR1_SSI; //internal slave select (1 - software slave managemenet enabled (0 for disabled)
	SPI2->CR1 |= SPI_CR1_SSM; //software slave management enabled
	//BR - baud rate prescaler - 000 --> fPCLK/2
	SPI2->CR1 &= ~SPI_CR1_BR; //000 for fPCLK/0
	SPI2->CR1 &= ~SPI_CR1_CRCL;


	///////////////////////////////////////////////////
	//// FRAME SELECTION (MOTOROLA - MSB)    //////////
	//MSB Most significant bit / LSB - least significant bit
	SPI2->CR1 &= ~(SPI_CR1_LSBFIRST); //0 for MSB, 1 for LSB
	SPI2->CR1 &= ~(SPI_CR1_RXONLY); //Receive only mode enabled (0 for full duplex)
	SPI2->CR1 &= ~(SPI_CR1_BIDIMODE); //Bidirectional data mode enable (2-line unidirectional data mode for full-duplex, 1 line for half-duplex)
	//BIDIOE output enable in bidirectional mode

	SPI2->CR2 &= ~SPI_CR2_DS; //clear all bits
	SPI2->CR2 |= (0b0111 << SPI_CR2_DS_Pos); //0111 for 8 bit
	SPI2->CR2 |= SPI_CR2_FRXTH; //RXNE event on 8-bit
	//SPI2->CR2 |= SPI_CR2_NSSP;
	SPI2->CR1 |= SPI_CR1_SPE; //spi enabled
}

void EnableSpiClocks()
{
	RCC->APB1ENR1 |= RCC_APB1ENR1_SPI2EN; //RM 6.4.19 enable espi2 clock
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN; //PB10 for SCK
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN; // PC2, PC3 for MOSI and MISO
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
}

void Spi2Transmit(const uint8_t data)
{
	//wait unitl TXE (Transmit buffer Empty)
	while (!(SPI2->SR & SPI_SR_TXE));

	//Write data to SPI data register
	SPI2->DR = data;

	while (!(SPI2->SR & SPI_SR_TXE));
	while (SPI2->SR & SPI_SR_BSY);
}

uint8_t Spi2Receive()
{
	SPI2->DR = 0xFF; //dummy write
	//wait until RXNE (Receive buffer not empty)
	while (!(SPI2->SR & SPI_SR_RXNE));
	while (SPI2->SR & SPI_SR_BSY);
	return SPI2->DR;
}

void McpWriteRegister(auto& CSline /*GpioOutput*/, const uint8_t reg, const uint8_t value)
{
	CSline.Clear(); // select MCP
	__NOP(); __NOP(); __NOP(); // ~ short delay (~50-100ns)
	Spi2Transmit(0x40); //opcode for write to MCP23S08 (A2:A0 = 000);
	Spi2Transmit(reg); //register address
	Spi2Transmit(value); //data
	__NOP(); __NOP(); __NOP(); // ~ short delay (~50-100ns)
	CSline.Set();
}

uint8_t McpReadRegister(auto& CSline /*GpioOutput*/, const uint8_t reg)
{
	CSline.Clear();
	__NOP(); __NOP(); __NOP(); // ~ short delay (~50-100ns)
	Spi2Transmit(0x41); //opcode for read (R/W = 1)
	Spi2Transmit(reg); //register address
	volatile const uint8_t received = Spi2Receive();
	__NOP(); __NOP(); __NOP(); // ~ short delay (~50-100ns)
	CSline.Set();
	return received;
}

#endif /* SPI_HPP_ */
