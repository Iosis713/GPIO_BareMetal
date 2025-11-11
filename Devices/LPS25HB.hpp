#pragma once
#ifndef LPS25HB_HPP_
#define LPS25HB_HPP_

#include "../Peripherals/I2C/I2C.hpp"
#include <cmath>

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
	static constexpr uint8_t RPDS_L			= 0x39; //0b0011 1001 pressure offset LSB data
	static constexpr uint8_t RPDS_H			= 0x3A; //0b0011 1010 pressure offset MSB data (_H _L order)
	static constexpr uint8_t FIFO_CTRL		= 0x2E;

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
	//static constexpr uint8_t LPS25HB_ADDR 			= 0xBA;
	static constexpr uint8_t LPS25HB_ADDR				= 0x5D << 1;

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
		i2c.Write(LPS25HB_ADDR, reg, std::array<uint8_t, 1>{ value });
	}

	void SetPressureCalibration(const uint16_t value)
	{
		// value should be more or less equal to 16 * (relative pressure - absolute pressure)
		WriteRegister(LPS25HB_Registers::RPDS_L, value);
		WriteRegister(LPS25HB_Registers::RPDS_H, value >> 8);
	}

	void SetMeanMode()
	{
		WriteRegister(LPS25HB_Registers::CTRL_REG2, 0x40);
		WriteRegister(LPS25HB_Registers::FIFO_CTRL, 0xDF);
	}

	float ReadTemperatureC()
	{
		const uint8_t tempL = ReadRegister(LPS25HB_Registers::TEMP_OUT_L);
		const uint8_t tempH = ReadRegister(LPS25HB_Registers::TEMP_OUT_H);
		const int16_t tempRaw = static_cast<int16_t>((tempH << 8) | tempL);
		return 42.5f + tempRaw / 480.0f; // *C
	}

	float ReadTemperatureK()
	{
		return ReadTemperatureC() + 273.15f;
	}

	//ReadPressure
	float ReadAbsolutePressure()
	{
		const uint8_t pressureOutL = ReadRegister(LPS25HB_Registers::PRESS_OUT_L);
		const uint8_t pressureOutXL = ReadRegister(LPS25HB_Registers::PRESS_OUT_XL);
		const uint8_t pressureOutH = ReadRegister(LPS25HB_Registers::PRESS_OUT_H);

		int32_t pressureRaw = 0;
		pressureRaw = static_cast<int32_t>((pressureOutH << 16) | (pressureOutL << 8) | pressureOutXL);
		if (pressureRaw & 0x00800000)
			pressureRaw |= 0xFF000000;

		return static_cast<float>(pressureRaw) / 4096.0f; //hPa
	}

	float ReadRelativePressure(const float measurementHeight = 0.0f) //m
	{
		const float absolutePressure = ReadAbsolutePressure(); // hPa
		const float temperatureK = ReadTemperatureK(); // K
		static constexpr float airMolarMass = 0.0289644; // kg/mol
		static constexpr float g = 9.80665; // m/s2
		static constexpr float airGasConst = 8.31446261815324; // J/(mol K)

		const float relativePressure = absolutePressure * exp ((airMolarMass * g * measurementHeight) / (airGasConst * temperatureK)); //hPa
		return relativePressure;
	}

	float MeasureHeight()
	{
		static constexpr float seaLevelPressure = 1013.25; // hPa
		const float temperatureK = ReadTemperatureK(); // K
		const float absolutePressure = ReadAbsolutePressure(); //hPa
		static constexpr float airGasConst = 8.31446261815324; // J/(mol K)
		static constexpr float airMolarMass = 0.0289644; // kg/mol
		static constexpr float g = 9.80665; // m/s2
		const float height = -airGasConst / (g * airMolarMass) * temperatureK * log (absolutePressure / seaLevelPressure); //m
		return height;
	}

};

#endif /* LPS25HB_HPP_ */
