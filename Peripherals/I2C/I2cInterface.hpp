#pragma once

#include <cstdint>

template<typename Derived>
class I2cInterface
{
protected:
	void EnableClock()
	{
		//RM 6.2 --> PCLK1 or HSI16 or SYSCLK for I2Cx (x = 1, 2, 3)
		//PCLK1 is under APB1 (SYSCLK -->AHB PRES --> APB1 PRESC)
		if (static_cast<Derived*>(this)->i2c == I2C1)
		{
			RCC->APB1ENR1 |= RCC_APB1ENR1_I2C1EN; //RM 6.4.19
			//Reset I2C1 RM 6.4.13
			RCC->APB1RSTR1 |= RCC_APB1RSTR1_I2C1RST;
			RCC->APB1RSTR1 &= ~(RCC_APB1RSTR1_I2C1RST);
			//Set I2C1 clock source to pclk -- just for now. Probably will be extended to other clock sources
			RCC->CCIPR &= ~RCC_CCIPR_I2C1SEL_Msk;

		}
		else if (static_cast<Derived*>(this)->i2c == I2C2)
		{
			RCC->APB1ENR1 |= RCC_APB1ENR1_I2C2EN;
			RCC->APB1RSTR1 |= RCC_APB1RSTR1_I2C2RST;
			RCC->APB1RSTR1 &= ~(RCC_APB1RSTR1_I2C2RST);
			RCC->CCIPR &= ~RCC_CCIPR_I2C2SEL_Msk;
		}
		else if (static_cast<Derived*>(this)->i2c == I2C3)
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
		static_cast<Derived*>(this)->i2c->CR1 &= ~(I2C_CR1_PE);
		static_cast<Derived*>(this)->i2c->CR1 &= ~(I2C_CR1_DNF_Msk);
		static_cast<Derived*>(this)->i2c->CR1 &= ~(I2C_CR1_ANFOFF); //enabled analog filer

		static_cast<Derived*>(this)->i2c->CR2 &= ~(I2C_CR2_ADD10); //master operates in 7 bit addressing mode
		static_cast<Derived*>(this)->i2c->CR2 &= ~(I2C_CR2_SADD); //slave operates in 7 bit addressing mode (ADD10 = 0); by default anyway

		//TIMINGR 0x10909CEC from CubeMX Forbot
		//but from CubeMX for 4MH MSI setup: 0x00100D14
		SetTimingRegister(timingRegister);
		//i2c->TIMINGR = 0x00100D14;

		static_cast<Derived*>(this)->i2c->OAR1 &= ~(I2C_OAR1_OA1MODE);
		static_cast<Derived*>(this)->i2c->OAR1 &= ~(I2C_OAR1_OA1EN);
		static_cast<Derived*>(this)->i2c->OAR2 &= ~(I2C_OAR2_OA2EN);
		static_cast<Derived*>(this)->i2c->CR1 |= I2C_CR1_PE;
	}

	inline void SetTimingRegister(const uint32_t timingRegister){ static_cast<Derived*>(this)->i2c->TIMINGR = timingRegister; }
	inline bool IsrBusy() { return (static_cast<Derived*>(this)->i2c->ISR & I2C_ISR_BUSY); } // true = busy, false = idle
	inline bool IsrTxisEmpty() { return (static_cast<Derived*>(this)->i2c->ISR & I2C_ISR_TXIS); } // true = empty (can write data), false = not empty
	inline bool IsrTransferCompleted() { return (static_cast<Derived*>(this)->i2c->ISR & I2C_ISR_TC); } //true =TransferCompleted
	inline bool IsrStopDetected() {	return (static_cast<Derived*>(this)->i2c->ISR & I2C_ISR_STOPF); } // true = stop detected
	inline bool IsrReceiveDataNotEmpty() { return (static_cast<Derived*>(this)->i2c->ISR & I2C_ISR_RXNE); } //true = data avaiable in RXDR
	inline void StopDetectionClear() { static_cast<Derived*>(this)->i2c->ICR |= I2C_ICR_STOPCF;	}
};
