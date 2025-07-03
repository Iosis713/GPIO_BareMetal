/*
 * LCD_TFT_ST7735S.hpp
 *
 *  Created on: Jul 3, 2025
 *      Author: bartoszlozinski
 */
#pragma once
#ifndef LCD_TFT_ST7735S_HPP_
#define LCD_TFT_ST7735S_HPP_

#include <stdint.h>

#include "../Inc/Gpio.hpp"
#include "../Inc/LCD_TFT_ST7735S.hpp"
#include "../Inc/Spi.hpp"
#include "../Inc/Timer.hpp"
#include <array>

static constexpr uint8_t ST7735S_SLPOUT			=	0x11;
static constexpr uint8_t ST7735S_DISPOFF		=	0x28;
static constexpr uint8_t ST7735S_DISPON			=	0x29;
static constexpr uint8_t ST7735S_CASET			=	0x2a;
static constexpr uint8_t ST7735S_RASET			=	0x2b;
static constexpr uint8_t ST7735S_RAMWR			=	0x2c;
static constexpr uint8_t ST7735S_MADCTL			=	0x36;
static constexpr uint8_t ST7735S_COLMOD			=	0x3a;
static constexpr uint8_t ST7735S_FRMCTR1		=	0xb1;
static constexpr uint8_t ST7735S_FRMCTR2		=	0xb2;
static constexpr uint8_t ST7735S_FRMCTR3		=	0xb3;
static constexpr uint8_t ST7735S_INVCTR			=	0xb4;
static constexpr uint8_t ST7735S_PWCTR1			=	0xc0;
static constexpr uint8_t ST7735S_PWCTR2			=	0xc1;
static constexpr uint8_t ST7735S_PWCTR3			=	0xc2;
static constexpr uint8_t ST7735S_PWCTR4			=	0xc3;
static constexpr uint8_t ST7735S_PWCTR5			=	0xc4;
static constexpr uint8_t ST7735S_VMCTR1			=	0xc5;
static constexpr uint8_t ST7735S_GAMCTRP1		=	0xe0;
static constexpr uint8_t ST7735S_GAMCTRN1		=	0xe1;
uint16_t constexpr CMD(const uint8_t x) { return x | 0x100 ;}

static constexpr auto initTable = std::to_array<uint16_t>({
  CMD(ST7735S_FRMCTR1), 0x01, 0x2c, 0x2d,
  CMD(ST7735S_FRMCTR2), 0x01, 0x2c, 0x2d,
  CMD(ST7735S_FRMCTR3), 0x01, 0x2c, 0x2d, 0x01, 0x2c, 0x2d,
  CMD(ST7735S_INVCTR), 0x07,
  CMD(ST7735S_PWCTR1), 0xa2, 0x02, 0x84,
  CMD(ST7735S_PWCTR2), 0xc5,
  CMD(ST7735S_PWCTR3), 0x0a, 0x00,
  CMD(ST7735S_PWCTR4), 0x8a, 0x2a,
  CMD(ST7735S_PWCTR5), 0x8a, 0xee,
  CMD(ST7735S_VMCTR1), 0x0e,
  CMD(ST7735S_GAMCTRP1), 0x0f, 0x1a, 0x0f, 0x18, 0x2f, 0x28, 0x20, 0x22,
                         0x1f, 0x1b, 0x23, 0x37, 0x00, 0x07, 0x02, 0x10,
  CMD(ST7735S_GAMCTRN1), 0x0f, 0x1b, 0x0f, 0x17, 0x33, 0x2c, 0x29, 0x2e,
                         0x30, 0x30, 0x39, 0x3f, 0x00, 0x07, 0x03, 0x10,
  CMD(0xf0), 0x01,
  CMD(0xf6), 0x00,
  CMD(ST7735S_COLMOD), 0x05,
  CMD(ST7735S_MADCTL), 0xa0,
});

void LCDInit(auto& RST, auto& DC, auto& CS)
{
	RST.Clear();
	Delay(100);

	RST.Set();
	Delay(100);

	for (const auto& element : initTable)
	{
		LCDSend(DC, CS, element);
	}
	Delay(200);

	LCDCmd(DC, CS, ST7735S_SLPOUT);
	Delay(120);

	LCDCmd(DC, CS, ST7735S_DISPON);
};

//static so it wont we used out of the file
//private in class
static void LCDCmd(auto& DC, auto& CS, const uint8_t cmd)
{
	DC.Clear();
	CS.Clear();
	Spi2TransmitHalfDuplex(cmd);
	CS.Set();
}

static void LCDData(auto& DC, auto& CS, const uint8_t data)
{
	DC.Set();
	CS.Clear();
	Spi2TransmitHalfDuplex(data);
	CS.Set();
}

static void LCDSend(auto& DC, auto& CS, const uint16_t value)
{
	if (value & 0x100)
		LCDCmd(DC, CS, value);
	else
		LCDData(DC, CS, value);
}



#endif /* LCD_TFT_ST7735S_HPP_ */
