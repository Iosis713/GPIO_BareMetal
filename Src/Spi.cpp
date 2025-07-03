/*
 * Spi.cpp
 *
 *  Created on: Jun 28, 2025
 *      Author: bartoszlozinski
 */

#include "../Inc/Spi.hpp"

void SpiConfigHalfDuplex()
{
	//Reference manual 42.6.1

	SPI2->CR1 &= ~(SPI_CR1_SPE); //disable before configuration
	//SPI2->CR1 |= SPI_CR1_SPE;
	//these two should be seen in MCP23S08 documentation
	SPI2->CR1 &= ~SPI_CR1_CPOL;      // CPOL = 0 (clock idle low)
	SPI2->CR1 &= ~SPI_CR1_CPHA;      // CPHA = 0 (capture on 1st edge)
	SPI2->CR1 |= SPI_CR1_MSTR; //select as master (micro-controller as a master)
	SPI2->CR1 |= SPI_CR1_SSI; //internal slave select (1 - software slave managemenet enabled (0 for disabled)
	SPI2->CR1 |= SPI_CR1_SSM; //software slave management enabled
	//BR - baud rate prescaler - 000 --> fPCLK/2
	SPI2->CR1 &= ~SPI_CR1_BR; //000 for fPCLK/2
	SPI2->CR1 &= ~(SPI_CR1_CRCEN); //hardware crc calculation disabled
	SPI2->CR1 &= ~SPI_CR1_CRCL;


	///////////////////////////////////////////////////
	//// FRAME SELECTION (MOTOROLA - MSB)    //////////
	//MSB Most significant bit / LSB - least significant bit
	SPI2->CR1 &= ~(SPI_CR1_LSBFIRST); //0 for MSB, 1 for LSB
	//SPI2->CR1 &= ~(SPI_CR1_RXONLY); //Receive only mode enabled (0 for full duplex)
	SPI2->CR1 |= SPI_CR1_BIDIMODE; //Bidirectional data mode enable (2-line unidirectional data mode for full-duplex, 1 line for half-duplex)
	SPI2->CR1 |= SPI_CR1_BIDIOE; //0 - receive only, 1 - transmit only -- to set up for final class
	//BIDIOE output enable in bidirectional mode

	SPI2->CR2 &= ~SPI_CR2_DS; //clear all bits
	SPI2->CR2 |= (0b0111 << SPI_CR2_DS_Pos); //0111 for 8 bit
	SPI2->CR2 |= SPI_CR2_FRXTH; //RXNE event on 8-bit
	SPI2->CR2 &= ~(SPI_CR2_FRF);
	SPI2->CR2 &= ~(SPI_CR2_SSOE);
	SPI2->CR2 &= ~(SPI_CR2_NSSP);
	SPI2->CR1 |= SPI_CR1_SPE; //spi enabled
}

void SpiConfigFullDuplex()
{
	//Reference manual 42.6.1

	SPI2->CR1 &= ~(SPI_CR1_SPE); //disable before configuration
	//SPI2->CR1 |= SPI_CR1_SPE;
	//these two should be seen in MCP23S08 documentation
	SPI2->CR1 &= ~SPI_CR1_CPOL;      // CPOL = 0 (clock idle low)
	SPI2->CR1 &= ~SPI_CR1_CPHA;      // CPHA = 0 (capture on 1st edge)
	SPI2->CR1 |= SPI_CR1_MSTR; //select as master (micro-controller as a master)
	SPI2->CR1 |= SPI_CR1_SSI; //internal slave select (1 - software slave managemenet enabled (0 for disabled)
	SPI2->CR1 |= SPI_CR1_SSM; //software slave management enabled
	//BR - baud rate prescaler - 000 --> fPCLK/2
	SPI2->CR1 &= ~SPI_CR1_BR; //000 for fPCLK/2
	SPI2->CR1 &= ~(SPI_CR1_CRCEN); //hardware crc calculation disabled
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
	SPI2->CR2 &= ~(SPI_CR2_FRF);
	SPI2->CR2 &= ~(SPI_CR2_SSOE);
	SPI2->CR2 &= ~(SPI_CR2_NSSP);
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
    // Wait until TXE (Transmit buffer empty)
    while (!(SPI2->SR & SPI_SR_TXE));

    // Write data to DR (this starts transmission)
    *reinterpret_cast<volatile uint8_t*>(&SPI2->DR) = data;

    // Wait until RXNE (Receive buffer not empty)
    while (!(SPI2->SR & SPI_SR_RXNE));

    // Read to clear RXNE (discard if you don't need it)
    [[maybe_unused]] volatile uint8_t dummyRead = *(volatile uint8_t *)&SPI2->DR;
}

void Spi2TransmitHalfDuplex(const uint8_t data)
{
    // Wait until TXE (Transmit buffer empty)
    while (!(SPI2->SR & SPI_SR_TXE));

    // Write data to DR (this starts transmission)
    *reinterpret_cast<volatile uint8_t*>(&SPI2->DR) = data;

    // Read to clear RXNE (discard if you don't need it)
    [[maybe_unused]] volatile uint8_t dummyRead = *(volatile uint8_t *)&SPI2->DR;
}

uint8_t Spi2Receive()
{
    // Wait until TXE (Transmit buffer empty)
    while (!(SPI2->SR & SPI_SR_TXE));

    // Write data to DR (dummy)
    *reinterpret_cast<volatile uint8_t*>(&SPI2->DR) = 0x00;

    // Wait until RXNE (Receive buffer not empty)
    while (!(SPI2->SR & SPI_SR_RXNE));

    // Return received byte
    return *reinterpret_cast<volatile uint8_t*>(&SPI2->DR);
}





