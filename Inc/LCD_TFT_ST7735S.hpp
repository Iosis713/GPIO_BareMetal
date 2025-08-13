#pragma once
#ifndef LCD_TFT_ST7735S_HPP_
#define LCD_TFT_ST7735S_HPP_

#include <stdint.h>

#include "../Peripherals/Gpio/GpioOutput.hpp"
#include "../Inc/LCD_TFT_ST7735S.hpp"
#include "../Inc/Spi.hpp"
#include "../Inc/Timer.hpp"
#include <array>
#include <span>

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

static constexpr uint16_t BLACK   =  0x0000;
static constexpr uint16_t RED     =  0xf800;
static constexpr uint16_t GREEN   =  0x07e0;
static constexpr uint16_t BLUE    =  0x001f;
static constexpr uint16_t YELLOW  =  0xffe0;
static constexpr uint16_t MAGENTA =  0xf81f;
static constexpr uint16_t CYAN    =  0x07ff;
static constexpr uint16_t WHITE   =  0xffff;

static constexpr uint8_t LCD_WIDTH = 160;
static constexpr uint8_t LCD_HEIGHT = 128;

static std::array<uint16_t, LCD_WIDTH * LCD_HEIGHT> frameBuffer;

void LCDFillBox(const int x, const int y, const int width, const int height, const uint16_t color);

void LCDInit(auto& RST, auto& DC, auto& CS, auto& spi)
{
	RST.Clear();
	Delay(100);

	RST.Set();
	Delay(100);

	for (const auto& element : initTable)
	{
		LCDSend(DC, CS, spi, element);
	}
	Delay(200);

	LCDCmd(DC, CS, spi, ST7735S_SLPOUT);
	Delay(120);

	LCDCmd(DC, CS, spi, ST7735S_DISPON);
};

//static so it wont we used out of the file
//private in class
static void LCDCmd(auto& DC, auto& CS, auto& spi, const uint8_t cmd)
{
	DC.Clear();
	CS.Clear();
	spi.Transmit(cmd);
	CS.Set();
}

static void LCDData(auto& DC, auto& CS, auto& spi, const uint8_t data)
{
	DC.Set();
	CS.Clear();
	spi.Transmit(data);
	CS.Set();
}

static void LCDData16(auto& DC, auto& CS, auto& spi, const uint16_t data)
{
	LCDData(DC, CS, spi, data >> 8);
	LCDData(DC, CS, spi, data);
}


static void LCDSend(auto& DC, auto& CS, auto& spi, const uint16_t value)
{
	if (value & 0x100)
		LCDCmd(DC, CS, spi, value);
	else
		LCDData(DC, CS, spi, value);
}

void LCDSetWindow(auto& DC, auto& CS, auto& spi, const int x, const int y, const int width, const int height)
{
	static constexpr uint16_t LCD_OFFSET_X = 1;//to fit view to the display
	static constexpr uint16_t LCD_OFFSET_Y = 2;

	LCDCmd(DC, CS, spi, ST7735S_CASET); //set initial and last column
	LCDData16(DC, CS, spi, LCD_OFFSET_X + x); //x position
	LCDData16(DC, CS, spi, LCD_OFFSET_X + x + width - 1);// last column

	LCDCmd(DC, CS, spi, ST7735S_RASET);// set initial and last row
	LCDData16(DC, CS, spi, LCD_OFFSET_Y + y);
	LCDData16(DC, CS, spi, LCD_OFFSET_Y + y + height - 1);//last column
}

void LCDFillBox(auto& DC, auto& CS, auto& spi, const int x, const int y, const int width, const int height, const uint16_t color)
{
	LCDSetWindow(DC, CS, spi, x, y, width, height);
	LCDCmd(DC, CS, spi, ST7735S_RAMWR);// starts sendign data to defined region (window)
	for (int i = 0; i < width * height; i++)
		LCDData16(DC, CS, spi, color);
}

void LCDPutPixel(const int x, const int y, const uint16_t color)
{
	frameBuffer[x + y * LCD_WIDTH] = color;
}

void LCDDrawImage(auto& DC, auto& CS, auto& spi, const int x, const int y, const int width, const int height, std::span<const uint16_t> data)
{
	LCDSetWindow(DC, CS, spi, x, y, width, height);
	LCDCmd(DC, CS, spi, ST7735S_RAMWR);
	DC.Set();
	CS.Clear();

	for (const auto& element : data)
	{
		spi.Transmit(element >> 8); //High byte first
		spi.Transmit(element & 0xFF); //low byte
	}

	CS.Set();
}

void LCDCopy(auto& DC, auto& CS, auto& spi)
{
	LCDSetWindow(DC, CS, spi, 0, 0, LCD_WIDTH, LCD_HEIGHT);
	LCDCmd(DC, CS, spi, ST7735S_RAMWR);
	DC.Set();
	CS.Clear();

	for (const auto& element : frameBuffer)
	{
		//Spi2TransmitHalfDuplex(element >> 8); //High byte first
		//Spi2TransmitHalfDuplex(element & 0xFF); //low byte
		spi.Transmit(element >> 8); //High byte first
		spi.Transmit(element & 0xFF); //low byte
	}

	CS.Set();
}


#endif /* LCD_TFT_ST7735S_HPP_ */
