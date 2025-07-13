/*
 * LPS25HB.hpp
 *
 *  Created on: Jul 13, 2025
 *      Author: bartoszlozinski
 */

#pragma once
#ifndef LPS25HB_HPP_
#define LPS25HB_HPP_
#include "I2C.hpp"

namespace LPS25HB_Registers
{
	static constexpr uint8_t WHO_AM_I 		= 0x0F;
	static constexpr uint8_t CTRL_REG1 		= 0x20; //Control register
	static constexpr uint8_t CTRL_REG2 		= 0x21;
	static constexpr uint8_t CTRL_REG3 		= 0x22;
	static constexpr uint8_t CTRL_REG4 		= 0x23;
	static constexpr uint8_t PRESS_OUT_XL 	= 0x28;	//pressure output registers
	static constexpr uint8_t PRESS_OUT_L 	= 0x29;
	static constexpr uint8_t PRESS_OUT_H 	= 0x2A;
	static constexpr uint8_t TEMP_OUT_L 	= 0x2B;	//temperature output register
	static constexpr uint8_t TEMP_OUT_H 	= 0x2C;
}

template<typename I2cType>
class LPS25HB
{
private:
	I2cType& i2c;
	static constexpr uint8_t LPS25HB_ADDR 			= 0xBA;



public:
	LPS25HB() = delete;
	LPS25HB(LPS25HB&& source) = delete;
	LPS25HB operator=(const LPS25HB& source) = delete;
	LPS25HB operator=(LPS25HB&& source) = delete;
	~LPS25HB() = default;
	explicit LPS25HB(I2cType& i2c_) //no implicit conversion
		: i2c(i2c_)	{}

	uint8_t ReadRegister(const uint8_t reg)
	{
		uint8_t resultValue = 0;
		i2c.Read(LPS25HB_ADDR, reg, &resultValue, 1);
		return resultValue;
	}
};

#endif /* LPS25HB_HPP_ */
