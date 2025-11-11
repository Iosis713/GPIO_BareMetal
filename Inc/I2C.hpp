#pragma once
#ifndef I2C_HPP_
#define I2C_HPP_

#include <cstdint>

//right now only for I2C_1 like in forbot course:
//https://forbot.pl/blog/kurs-stm32l4-zewnetrzna-pamiec-eeprom-i2c-id47820

template<typename Derived>
class I2cInterface
{
protected:
	inline I2C_TypeDef* I2C() const { return reinterpret_cast<I2C_TypeDef*>(Derived::i2cAddr); }

	void EnableClock()
	{
		//RM 6.2 --> PCLK1 or HSI16 or SYSCLK for I2Cx (x = 1, 2, 3)
		//PCLK1 is under APB1 (SYSCLK -->AHB PRES --> APB1 PRESC)
		if constexpr (Derived::i2cAddr == I2C1_BASE)
		{
			RCC->APB1ENR1 |= RCC_APB1ENR1_I2C1EN; //RM 6.4.19
			//Reset I2C1 RM 6.4.13
			RCC->APB1RSTR1 |= RCC_APB1RSTR1_I2C1RST;
			RCC->APB1RSTR1 &= ~(RCC_APB1RSTR1_I2C1RST);
			//Set I2C1 clock source to pclk -- just for now. Probably will be extended to other clock sources
			RCC->CCIPR &= ~RCC_CCIPR_I2C1SEL_Msk;

		}
		else if constexpr (Derived::i2cAddr == I2C2_BASE)
		{
			RCC->APB1ENR1 |= RCC_APB1ENR1_I2C2EN;
			RCC->APB1RSTR1 |= RCC_APB1RSTR1_I2C2RST;
			RCC->APB1RSTR1 &= ~(RCC_APB1RSTR1_I2C2RST);
			RCC->CCIPR &= ~RCC_CCIPR_I2C2SEL_Msk;
		}
		else if constexpr (Derived::i2cAddr == I2C3_BASE)
		{
			RCC->APB1ENR1 |= RCC_APB1ENR1_I2C3EN;
			RCC->APB1RSTR1 |= RCC_APB1RSTR1_I2C3RST;
			RCC->APB1RSTR1 &= ~(RCC_APB1RSTR1_I2C3RST);
			RCC->CCIPR &= ~RCC_CCIPR_I2C3SEL_Msk;
		}
	}

	void ConfigI2C(const uint32_t timingRegister)
	{
		//CR1 = 0 by default RM 39.9.1.
		auto i2c = I2C();
		i2c->CR1 &= ~(I2C_CR1_PE);
		i2c->CR1 &= ~(I2C_CR1_DNF_Msk);
		i2c->CR1 &= ~(I2C_CR1_ANFOFF); //enabled analog filer

		i2c->CR2 &= ~(I2C_CR2_ADD10); //master operates in 7 bit addressing mode
		i2c->CR2 &= ~(I2C_CR2_SADD); //slave operates in 7 bit addressing mode (ADD10 = 0); by default anyway

		//TIMINGR 0x10909CEC from CubeMX Forbot
		//but from CubeMX for 4MH MSI setup: 0x00100D14
		SetTimingRegister(timingRegister);
		//i2c->TIMINGR = 0x00100D14;


		i2c->OAR1 &= ~(I2C_OAR1_OA1MODE);
		i2c->OAR1 &= ~(I2C_OAR1_OA1EN);
		i2c->OAR2 &= ~(I2C_OAR2_OA2EN);
		i2c->CR1 |= I2C_CR1_PE;
	}

	inline void SetTimingRegister(const uint32_t timingRegister)
	{
		I2C()->TIMINGR = timingRegister;
	}

	inline bool IsrBusy()
	{
		return (I2C()->ISR & I2C_ISR_BUSY); // true = busy, false = idle
	}

	inline bool IsrTxisEmpty()
	{
		return (I2C()->ISR & I2C_ISR_TXIS); // true = empty (can write data), false = not empty
	}

	inline bool IsrTransferCompleted()
	{
		return (I2C()->ISR & I2C_ISR_TC); //true =TransferCompleted
	}

	inline bool IsrStopDetected()
	{
		return (I2C()->ISR & I2C_ISR_STOPF); // true = stop detected
	}

	inline bool IsrReceiveDataNotEmpty()
	{
		return (I2C()->ISR & I2C_ISR_RXNE); //true = data avaiable in RXDR
	}

	inline void StopDetectionClear()
	{
		I2C()->ICR |= I2C_ICR_STOPCF;
	}
};

template<std::uintptr_t i2cAddr_>
class I2c : public I2cInterface<I2c<i2cAddr_>>
{
	friend class I2cInterface<I2c<i2cAddr_>>;
protected:
	static constexpr std::uintptr_t i2cAddr = i2cAddr_;

public:
	I2c(const I2c& source) = delete;
	I2c(I2c&& source) = delete;
	I2c& operator=(const I2c& source) = delete;
	I2c& operator=(I2c&& source) = delete;
	I2c() = delete;
	I2c(const uint32_t timingRegister)
	{
		this->template EnableClock();
		this->template ConfigI2C(timingRegister);
	}

	bool Write(const uint8_t devAddr, const uint8_t memAddr, const uint8_t* const data, const std::size_t size)
	{
		if (size == 0 || size > 8) //24AA01 has 8-byte write page limit
			return false;

		auto i2c = this->I2C();
		while (this->IsrBusy()) {} //wait if bus is busy

		//Configure I2C1 to write:
		// - dev_addr: 8-bit address (e.g. 0xA0 = 0x50 << 1)
		// - (1 + len): 1 byte for mem_addr + N data bytes
		// - START: trigger start condition

		i2c->CR2 = (devAddr & 0xFE)						//Ensure LSB=0 for write (xxxx xxx0)
			      | ((size + 1) << I2C_CR2_NBYTES_Pos)		//total number of bytes to send
				  | I2C_CR2_START;							//generate start condition

		while (!this->IsrTxisEmpty()) {};	//wait until TX buffer is ready, then send memory address
		i2c->TXDR = memAddr;

		//send each byte of data
		for (std::size_t i = 0; i < size; ++i)
		{
			while (!this->IsrTxisEmpty()) {} // Wait for TX buffer ready
			i2c->TXDR = data[i];
		}

		while (!this->IsrTransferCompleted()) {} //wait Until transfer complete (NBYTES sent)
		i2c->CR2 |= I2C_CR2_STOP;			 //Send STOP condition
		while (!this->IsrStopDetected()) {}	 //Wait for STOP to complete (optional, just to ensure bus is idle
		this->StopDetectionClear();
		return true;
	}

	bool Read(const uint8_t devAddr, const uint8_t memAddr, uint8_t* const data, const std::size_t size)
	{
		if (size == 0)
			return false;

		auto i2c = this->I2C();

		while (this->IsrBusy()) {} // Wait until I2C bus is idle
		// First, send a dummy write to set EEPROM's itnernal memory address pointer
		i2c->CR2 = (devAddr & 0xFE)			// Write address (LSB = 0)
				  | (1 << I2C_CR2_NBYTES_Pos)	//Send 1 bte (memory address)
				  | I2C_CR2_START;				// Generate start condition

		while (!this->IsrTxisEmpty()) {}  //Wait for TX ready
		i2c->TXDR = memAddr;					//send memory address
		while (!this->IsrTransferCompleted()) {}	//wait until transfer complete

		//Read: repeat start - read
		i2c->CR2 = (devAddr & 0xFE)				// Device address
				  | (size << I2C_CR2_NBYTES_Pos)	// number of bytes to read
				  | I2C_CR2_RD_WRN					// Read mode (RM 639.9.3 Transfer direction (0 - write, 1 - read)
				  | I2C_CR2_START;					//Generated repeated start

		//Read loop
		for (size_t i = 0; i < size; ++i)
		{
			while (!this->IsrReceiveDataNotEmpty()) {} //Wait until RX buffer not empty - RM 39.9.8 Receive data register not empty. Cleared when RXDX is read
			data[i] = static_cast<uint8_t>(I2C1->RXDR);	//register is 32bit, even if data is only 8 bit
		}

		while (!this->IsrTransferCompleted()) {}		//Wait until all bytes received; CR39.9.7. - Transfer complete (master mode)
		i2c->CR2 |= I2C_CR2_STOP;					//Send STOP condition
		while (!this->IsrStopDetected()) {}			//Wait for STOP to complete
		this->StopDetectionClear();
		return true;
	}
};


#endif /* I2C_HPP_ */
