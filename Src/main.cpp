#include "../Peripherals/Gpio/GpioOutput.hpp"

#include "../Inc/Button.hpp"
#include "../Inc/Timer.hpp"
#include "../Inc/Uart.hpp"
#include "../Inc/Config.hpp"
#include "../Inc/Pwm.hpp"
#include "../Inc/Adc.hpp"
#include "../Inc/Spi.hpp"
#include "../Inc/Mcp23S08.hpp"
#include "../Inc/LCD_TFT_ST7735S.hpp"
#include "../Inc/I2C.hpp"
#include "../Inc/LPS25HB.hpp"
#include <stdio.h>
#include <cstring>

//drivers/cmsis/include/core_cm0plus////systick_config - method
//To Cortex system timer - in hal clock config

//from startup file
extern "C" void TIM3_IRQHandler(void);

void ExampleUseOfTFTDisplayST7735S(auto& LCD_CS, auto& LCD_RST, auto& LCD_DC, auto& spi);
void ExampleUseOfLSB25HB_I2c(auto& lsb25hb);
float LM35CalculateTemperatureC(const uint32_t rawTemp);
float CalculateAirSoundSpeed(const float tempC);
void SetRGBSignal(const float distance);

GpioOutput<GPIO_TypeDef, 5> ld2(GPIOA);
UART<USART_TypeDef, GPIO_TypeDef, 115200, 80> uart2(USART2);
Adc<ADC_TypeDef, 1> adc1{ADC1};
AdcChannel<GPIO_TypeDef, ADC_TypeDef, 0, 1> adc1Channel1{adc1.portAdc, GPIOC, 1};

//PWM<TIM3_BASE, (4 - 1), (1000 - 1)> pwmTim3(1);
//PWMChannel<GPIOA_BASE, 6, 1> channel1(pwmTim3.Timer(), AlternateFunction::AF2);
//Button<GPIO_TypeDef, 13, OptionsPUPDR::PullUp> userButton(GPIOC);

int main(void)
{
	SystemTimer::Init(4000);
	uart2.ConfigureExtiReceive();
	Timer timerADCPrint{300};

	while (true)
	{
		////////////////_____UART/GPIO EXTI_____////////////////
		
		if (uart2.GetStringIT() == ERROR_CODE::OK)
		{
			uart2.SendString(uart2.GetBuffer().data());

			if (strcmp(uart2.GetBuffer().data(), "set") == 0)
				ld2.Set();
			else if (strcmp(uart2.GetBuffer().data(), "clear") == 0)
				ld2.Clear();
			else if (strcmp(uart2.GetBuffer().data(), "toggle") == 0)
				ld2.Toggle();

			uart2.ClearBuffer();
		}
		
		////////////////_____UART/GPIO EXTI_____////////////////

		////////////////_____ADC_____////////////////
		
		adc1.StartConversion();
		adc1Channel1.Read();

		if (timerADCPrint.IsExpired())
		{
			char buffer[64];
			snprintf(buffer, sizeof(buffer), "ADC channel 1: %lu\n", static_cast<unsigned long>(adc1Channel1.Get()));

			//snprintf(buffer, sizeof(buffer), "ADC channel 1: %lu, ADC channel 2: %lu\n", static_cast<unsigned long>(adc1Channel1.Get()), static_cast<unsigned long>(adcChannel2.Get()));
			uart2.SendString(buffer);
		}

		
		////////////////_____ADC_____////////////////

		////////////////_____GPIO EXTI_____////////////////

		/*if (userButton.InterruptOccured())
		{
			ld2.Toggle();
			userButton.ClearInterruptFlag();
		}*/

		////////////////_____GPIO EXTI_____////////////////
	}
}


//interrupt handling function from start-up
//startup_stm32l476rgtx.s
//EXTI15_10_IRQHandler
extern "C" void EXTI15_10_IRQHandler(void)
{
	//userButton.IrqHandler();
}

//startup_stm32l476rgtx.s
extern "C" void USART2_IRQHandler(void)
{
	uart2.IRQ_Handler();
	/*
	 * there should be long, time consuming operations
	 * flag can be enabled (data is ready to send/receive)
	 * uart can rise interrupt when data is ready to receive
	 * then add to buffer
	 * when the sign is  '\0' it can be send
	 */
}

extern "C" void TIM2_IRQHandler(void)
{
	//pwmTim2.InterruptHandler();
	//hc_sr04_echoCh1.InterruptHandler();
	//hc_sr04_echoCh2.InterruptHandler();
	//hc_sr04_trig.InterruptHandler();
}


extern "C" void TIM3_IRQHandler(void)
{
	//pwmTim3.InterruptHandler();
	//channel1.InterruptHandler();

	//rgbGreen.InterruptHandler();
	//rgbRed.InterruptHandler();
}


void ExampleUseOfTFTDisplayST7735S(auto& LCD_CS, auto& LCD_RST, auto& LCD_DC, auto& spi)
{
	//TFTDisplay_ST7735S
	LCDInit(LCD_RST, LCD_DC, LCD_CS, spi);

	LCDFillBox(LCD_DC, LCD_CS, spi, 0, 0, 160, 16, RED);
	LCDFillBox(LCD_DC, LCD_CS, spi, 0, 16, 160, 16, GREEN);
	LCDFillBox(LCD_DC, LCD_CS, spi, 0, 32, 160, 16, BLUE);
	LCDFillBox(LCD_DC, LCD_CS, spi, 0, 48, 160, 16, YELLOW);
	LCDFillBox(LCD_DC, LCD_CS, spi, 0, 64, 160, 16, MAGENTA);
	LCDFillBox(LCD_DC, LCD_CS, spi, 0, 80, 160, 16, CYAN);
	LCDFillBox(LCD_DC, LCD_CS, spi, 0, 96, 160, 16, WHITE);
	LCDFillBox(LCD_DC, LCD_CS, spi, 0, 112, 160, 16, BLACK);

	std::array<uint16_t, 64 *64> testImage;
	for (auto& pixel : testImage)
		pixel = BLUE;

	LCDDrawImage(LCD_DC, LCD_CS, spi, 0, 0, 64, 64, testImage);
	LCDDrawImage(LCD_DC, LCD_CS, spi, 16, 16, 64, 64, testImage);
	LCDDrawImage(LCD_DC, LCD_CS, spi, 32, 32, 64, 64, testImage);
	LCDDrawImage(LCD_DC, LCD_CS, spi, 48, 48, 64, 64, testImage);
	LCDDrawImage(LCD_DC, LCD_CS, spi, 64, 64, 64, 64, testImage);

	for (int i = 0; i < 128; i++)
	{
		LCDPutPixel(i, i, RED);
		LCDPutPixel(127 - i, i, RED);
	}


	for (int y = 0; y < LCD_HEIGHT; y++)
	{
	  for (int x = 0; x < LCD_WIDTH; x++)
	  {
		  LCDPutPixel(x, y, (x / 10 + y * 16));
	  }
	}

	LCDCopy(LCD_DC, LCD_CS, spi);
}

void ExampleUseOfLSB25HB_I2c(auto& lsb25hb)
{
	////////////////////////////////////////////////////////////
		//////___________________I2C__________________________//////

		GpioAlternate<GPIO_TypeDef, 6, AlternateFunction::AF4, OptionsOTYPER::OpenDrain> i2c1SCL(GPIOB);
		GpioAlternate<GPIO_TypeDef, 7, AlternateFunction::AF4, OptionsOTYPER::OpenDrain> i2c1SDA(GPIOB);

		I2c<I2C1_BASE> i2c1{0x00100D14}; //from cubemx for specifc clock

		/*
		 *EEPROM
		 * static constexpr uint8_t deviceAddress = 0xA0;
		static constexpr uint8_t memoryAddress = 0x10;
		const uint8_t testData = 90;
		uint8_t readResult = 0;

		i2c1.Write(deviceAddress, memoryAddress, &testData, 1);
		Delay(5); //EEPROM write time
		i2c1.Read(deviceAddress, memoryAddress, &readResult, 1);
		*/

		LPS25HB lps25hb(i2c1);
		uart2.SendString("Searching...\n");
		const uint8_t whoAmI = lps25hb.ReadRegister(LPS25HB_Registers::WHO_AM_I);

		uart2.SendChar(whoAmI);
		if (whoAmI == 0xBD)
			uart2.SendString("Found: LPS25HB");
		else
			uart2.SendString("Error: (0x%02X), whoAmI");

		lps25hb.WriteRegister(LPS25HB_Registers::CTRL_REG1, (LPS25HB_Registers::CTRL_REG1_bits::PD
														  | LPS25HB_Registers::CTRL_REG1_bits::ODR2));
		Delay(100); //25Hz temp measurement frequency

		float temp = lps25hb.ReadTemperatureC();
		char buffer[32];
		snprintf(buffer, sizeof(buffer), "Temp = %.1f *C", temp);
		uart2.SendString(buffer);

		float absolutePressure = lps25hb.ReadAbsolutePressure();
		snprintf(buffer, sizeof(buffer), "Absolute pressure = %.2f hPa", absolutePressure);
		uart2.SendString(buffer);

		float relativePressure = lps25hb.ReadRelativePressure(77);
		snprintf(buffer, sizeof(buffer), "Relative pressure = %.2f hPa", relativePressure);
		uart2.SendString(buffer);

		float height = lps25hb.MeasureHeight();
		snprintf(buffer, sizeof(buffer), "Measured Height = %.2f m", height);
		uart2.SendString(buffer);

		//////___________________I2C__________________________//////
		////////////////////////////////////////////////////////////
}

float LM35CalculateTemperatureC(const uint32_t rawTemp)
{
	static constexpr float tempMeasurementResolution = 0.01f; //[V/C]
	static constexpr float maxVoltage = 3.3f; // [V]
	static constexpr float maxADCValue = 4096.f; // [-]
	return rawTemp * maxVoltage / (tempMeasurementResolution * maxADCValue);
}

float CalculateAirSoundSpeed(const float tempC)
{
	return 331.8f + 0.6f * tempC;
}

/*
void SetRGBSignal(const float distance)
{
	const uint32_t maxPWMwidth = pwmTim3.GetMaxWidth();
	static constexpr float minDistance = 10.0f; //cm = 0%
	static constexpr float maxDistance = 100.f; //cm = 100%

	uint32_t distancePercentGreen = static_cast<uint32_t>(distance) / (maxDistance - minDistance) * 100;

	if (distancePercentGreen > 100)
		distancePercentGreen = 100;
	//narrowing conversion of anything below 1

	rgbGreen.SetPulse(distancePercentGreen * maxPWMwidth / 100);
	rgbRed.SetPulse((100 - distancePercentGreen) * maxPWMwidth / 100);
}*/
