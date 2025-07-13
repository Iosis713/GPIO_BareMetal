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
	static constexpr uint8_t WHO_AM_I 		= 0x0F; //0b0000 1111
	static constexpr uint8_t CTRL_REG1 		= 0x20; //0b0010 0000 Control register
	static constexpr uint8_t CTRL_REG2 		= 0x21; //0b0010 0001
	static constexpr uint8_t CTRL_REG3 		= 0x22; //0b0010 0010
	static constexpr uint8_t CTRL_REG4 		= 0x23; //0b0010 0011
	static constexpr uint8_t PRESS_OUT_XL 	= 0x28;	//0b0010 1000 pressure output registers
	static constexpr uint8_t PRESS_OUT_L 	= 0x29; //0b0010 1001
	static constexpr uint8_t PRESS_OUT_H 	= 0x2A; //0b0010 1010
	static constexpr uint8_t TEMP_OUT_L 	= 0x2B;	//0b0010 1011 temperature output register
	static constexpr uint8_t TEMP_OUT_H 	= 0x2C; //0b0010 1100

	static constexpr uint16_t TEMP_OUT_L_16bit = TEMP_OUT_L | 0x80; //0b1010 1011

	namespace CTRL_REG1_bits
	{
		static constexpr uint8_t PD = 0x80;
		static constexpr uint8_t ODR2 = 0x40;
		static constexpr uint8_t ODR1 = 0x20;
		static constexpr uint8_t ODR0 = 0x10;
	}
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

	uint8_t ReadRegister(const uint8_t reg, const std::size_t size = 1)
	{
		uint8_t resultValue = 0;
		i2c.Read(LPS25HB_ADDR, reg, &resultValue, size);
		return resultValue;
	}

	void WriteRegister(const uint8_t reg, const uint8_t value)
	{
		i2c.Write(LPS25HB_ADDR, reg, &value, 1);
	}

	int16_t ReadTemperatureRaw()
	{
		const uint8_t tempL = ReadRegister(LPS25HB_Registers::TEMP_OUT_L);
		const uint8_t tempH = ReadRegister(LPS25HB_Registers::TEMP_OUT_H);
		return static_cast<int16_t>((tempH << 8) | tempL);
	}

	float ReadTemperatureC()
	{
		const int16_t tempRaw = ReadTemperatureRaw();
		return 42.5f + tempRaw / 480.0f;
	}

	//ReadPressure


};

#endif /* LPS25HB_HPP_ */
