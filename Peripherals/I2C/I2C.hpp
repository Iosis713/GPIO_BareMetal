#pragma once

#include "I2cInterface.hpp"
#include <array>

//right now only for I2C_1 like in forbot course:
//https://forbot.pl/blog/kurs-stm32l4-zewnetrzna-pamiec-eeprom-i2c-id47820

template<typename T>
concept I2CStructure = requires(T structure)
{
	{ structure.CR1 }	  -> std::convertible_to<volatile uint32_t&>; 
	{ structure.CR1 }     -> std::convertible_to<volatile uint32_t&>; 
	{ structure.CR2 }     -> std::convertible_to<volatile uint32_t&>;
	{ structure.OAR1 }    -> std::convertible_to<volatile uint32_t&>;
	{ structure.OAR2 }    -> std::convertible_to<volatile uint32_t&>;
	{ structure.TIMINGR } -> std::convertible_to<volatile uint32_t&>;
	{ structure.TIMEOUTR }-> std::convertible_to<volatile uint32_t&>;
	{ structure.ISR }     -> std::convertible_to<volatile uint32_t&>;
	{ structure.ICR }     -> std::convertible_to<volatile uint32_t&>;
	{ structure.PECR }    -> std::convertible_to<volatile uint32_t&>;
	{ structure.RXDR }    -> std::convertible_to<volatile uint32_t&>;
	{ structure.TXDR }    -> std::convertible_to<volatile uint32_t&>;
};

template<I2CStructure I2CStruct>
class I2c : public I2cInterface<I2c<I2CStruct>>
{
	friend class I2cInterface<I2c<I2CStruct>>;
protected:
	volatile I2CStruct* const i2c = nullptr;
	inline void WaitWhileIsrBusy() {while (this->IsrBusy()) {}} //wait if bus is busy
	inline void WaitUntilTXBufferIsReady() {while (!this->IsrTxisEmpty()) {};}	//wait until TX buffer is ready, then send memory address
	inline void WaitUntilTransferCompleted() { while (!this->IsrTransferCompleted()) {};} //wait Until transfer complete (NBYTES sent)
	inline void WaitUntilIsrStopDetected() {while (!this->IsrStopDetected()) {};}	 //Wait for STOP to complete (optional, just to ensure bus is idle
	inline void WaitUntilRxBufferNotEmpty() {while (!this->IsrReceiveDataNotEmpty()) {}} //Wait until RX buffer not empty - RM 39.9.8 Receive data register not empty. Cleared when RXDX is read

public:
	I2c(const I2c& source) = delete;
	I2c(I2c&& source) = delete;
	I2c& operator=(const I2c& source) = delete;
	I2c& operator=(I2c&& source) = delete;
	I2c() = delete;
	I2c(I2CStruct* const i2c_, const uint32_t timingRegister)
		: i2c(i2c_)
	{
		this->template EnableClock();
		this->template ConfigI2C(timingRegister);
	}

	template<std::size_t DataSize>
	bool Write(const uint8_t devAddr, const uint8_t memAddr, std::array<uint8_t, DataSize> data)
	{
		if (data.size() == 0 || data.size() > 8) //24AA01 has 8-byte write page limit
			return false;

		WaitWhileIsrBusy();

		//Configure I2C1 to write:
		// - dev_addr: 8-bit address (e.g. 0xA0 = 0x50 << 1)
		// - (1 + len): 1 byte for mem_addr + N data bytes
		// - START: trigger start condition

		i2c->CR2 = (devAddr & 0xFE)								//Ensure LSB=0 for write (xxxx xxx0)
			      | ((data.size() + 1) << I2C_CR2_NBYTES_Pos)	//total number of bytes to send
				  | I2C_CR2_START;								//generate start condition

		WaitUntilTXBufferIsReady();
		i2c->TXDR = memAddr;

		//send each byte of data
		for (const auto& value : data)
		{
			WaitUntilTXBufferIsReady();
			i2c->TXDR = value;
		}

		WaitUntilTransferCompleted();
		i2c->CR2 |= I2C_CR2_STOP;			 //Send STOP condition
		WaitUntilIsrStopDetected();
		this->StopDetectionClear();
		return true;
	}

	template<std::size_t DataSize>
	bool Read(const uint8_t devAddr, const uint8_t memAddr, std::array<uint8_t, DataSize>& data)
	{
		if (data.size() == 0)
			return false;

		WaitWhileIsrBusy();
		// First, send a dummy write to set EEPROM's itnernal memory address pointer
		i2c->CR2 = (devAddr & 0xFE)			// Write address (LSB = 0)
				  | (1 << I2C_CR2_NBYTES_Pos)	//Send 1 bte (memory address)
				  | I2C_CR2_START;				// Generate start condition

		WaitUntilTXBufferIsReady();
		i2c->TXDR = memAddr;					//send memory address
		WaitUntilTransferCompleted();

		//Read: repeat start - read
		i2c->CR2 = (devAddr & 0xFE)				// Device address
				  | (data.size() << I2C_CR2_NBYTES_Pos)	// number of bytes to read
				  | I2C_CR2_RD_WRN					// Read mode (RM 639.9.3 Transfer direction (0 - write, 1 - read)
				  | I2C_CR2_START;					//Generated repeated start

		//Read loop
		for (auto& value : data)
		{
			WaitUntilRxBufferNotEmpty();
			value = static_cast<uint8_t>(I2C1->RXDR);	//register is 32bit, even if data is only 8 bit
		}

		WaitUntilTransferCompleted();//Wait until all bytes received; CR39.9.7. - Transfer complete (master mode)
		i2c->CR2 |= I2C_CR2_STOP;					//Send STOP condition
		WaitUntilIsrStopDetected();
		this->StopDetectionClear();
		return true;
	}
};
