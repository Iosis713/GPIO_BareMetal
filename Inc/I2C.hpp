/*
 * I2C.hpp
 *
 *  Created on: Jul 12, 2025
 *      Author: bartoszlozinski
 */


#pragma once
#ifndef I2C_HPP_
#define I2C_HPP_

#include "Config.hpp"

//right now only for I2C_1 like in forbot course:
//https://forbot.pl/blog/kurs-stm32l4-zewnetrzna-pamiec-eeprom-i2c-id47820

class I2C
{
private:
	void EnableClock()
	{
		//RM 6.2 --> PCLK1 or HSI16 or SYSCLK for I2Cx (x = 1, 2, 3)
		//PCLK1 is under APB1 (SYSCLK -->AHB PRES --> APB1 PRESC)
		RCC->APB1ENR1 |= RCC_APB1ENR1_I2C1EN; //RM 6.4.19

		//Reset I2C1 RM 6.4.13
		RCC->APB1RSTR1 |= RCC_APB1RSTR1_I2C1RST;
		RCC->APB1RSTR1 &= ~(RCC_APB1RSTR1_I2C1RST);

		//Set I2C1 clock source to pclk
		RCC->CCIPR &= ~RCC_CCIPR_I2C1SEL_Msk;
	}

	void ConfigI2C()
	{
		//CR1 = 0 by default RM 39.9.1.
		I2C1->CR1 &= ~(I2C_CR1_PE);
		I2C1->CR1 &= ~(I2C_CR1_DNF_Msk);
		I2C1->CR1 &= ~(I2C_CR1_ANFOFF); //enabled analog filer

		I2C1->CR2 &= ~(I2C_CR2_ADD10); //master operates in 7 bit addressing mode
		I2C1->CR2 &= ~(I2C_CR2_SADD); //slave operates in 7 bit addressing mode (ADD10 = 0); by default anyway

		//TIMINGR 0x10909CEC from CubeMX Forbot
		//but from CubeMX for 4MH MSI setup: 0x00100D14
		I2C1->TIMINGR = 0x00100D14;


		I2C1->OAR1 &= ~(I2C_OAR1_OA1MODE);
		I2C1->OAR1 &= ~(I2C_OAR1_OA1EN);
		I2C1->OAR2 &= ~(I2C_OAR2_OA2EN);
		I2C1->CR1 |= I2C_CR1_PE;
	}

public:
	I2C()
	{
		EnableClock();
		ConfigI2C();
	}
	~I2C() = default;

	bool WriteMemory(const uint8_t devAddr, const uint8_t memAddr, const uint8_t* const data, const std::size_t size)
	{
		if (size == 0 || size > 8) //24AA01 has 8-byte write page limit
			return false;

		while (I2C1->ISR & I2C_ISR_BUSY) {} //wait if bus is busy

		//Configure I2C1 to write:
		// - dev_addr: 8-bit address (e.g. 0xA0 = 0x50 << 1)
		// - (1 + len): 1 byte for mem_addr + N data bytes
		// - START: trigger start condition

		I2C1->CR2 = (devAddr & 0xFE)						//Ensure LSB=0 for write (xxxx xxx0)
			      | ((size + 1) << I2C_CR2_NBYTES_Pos)		//total number of bytes to send
				  | I2C_CR2_START;							//generate start condition

		while (!(I2C1->ISR & I2C_ISR_TXIS)) {};	//wait until TX buffer is ready, then send memory address
		I2C1->TXDR = memAddr;

		//send each byte of data
		for (std::size_t i = 0; i < size; ++i)
		{
			while (!(I2C1->ISR & I2C_ISR_TXIS)) {} // Wait for TX buffer ready
			I2C1->TXDR = data[i];
		}

		while (!(I2C1->ISR & I2C_ISR_TC)) {} //wait Until transfer complete (NBYTES sent)
		I2C1->CR2 |= I2C_CR2_STOP;			 //Send STOP condition
		while (I2C1->CR2 & I2C_CR2_STOP) {}	 //Wait for STOP to complete (optional, just to ensure bus is idle

		Delay(5); //EEPROM write time

		return true;
	}

	bool ReadMemory(const uint8_t devAddr, const uint8_t memAddr, uint8_t* const data, const std::size_t size)
	{
		if (size == 0)
			return false;

		while (I2C1->ISR & I2C_ISR_BUSY) {} // Wait until I2C bus is idle
		// First, send a dummy write to set EEPROM's itnernal memory address pointer
		I2C1->CR2 = (devAddr & 0xFE)			// Write address (LSB = 0)
				  | (1 << I2C_CR2_NBYTES_Pos)	//Send 1 bte (memory address)
				  | I2C_CR2_START;				// Generate start condition

		while (!(I2C1->ISR & I2C_ISR_TXIS)) {}  //Wait for TX ready
		I2C1->TXDR = memAddr;					//send memory address
		while (!(I2C1->ISR & I2C_ISR_TC)) {}	//wait until transfer complete

		//Read: repeat start - read
		I2C1->CR2 = (devAddr & 0xFE)				// Device address
				  | (size << I2C_CR2_NBYTES_Pos)	// number of bytes to read
				  | I2C_CR2_RD_WRN					// Read mode (RM 639.9.3 Transfer direction (0 - write, 1 - read)
				  | I2C_CR2_START;					//Generated repeated start

		//Read loop
		for (size_t i = 0; i < size; ++i)
		{
			while (!(I2C1->ISR & I2C_ISR_RXNE)) {} //Wait until RX buffer not empty - RM 39.9.8 Receive data register not empty. Cleared when RXDX is read
			data[i] = static_cast<uint8_t>(I2C1->RXDR);	//register is 32bit, even if data is only 8 bit
		}

		while (!(I2C1->ISR & I2C_ISR_TC)) {}		//Wait until all bytes received; CR39.9.7. - Transfer complete (master mode)
		I2C1->CR2 |= I2C_CR2_STOP;					//Send STOP condition
		while (I2C1->CR2 & I2C_CR2_STOP) {}			//Wait for STOP to complete

		return true;
	}
};


#endif /* I2C_HPP_ */
