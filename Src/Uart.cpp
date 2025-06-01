/*
 * Uart.cpp
 *
 *  Created on: Jun 1, 2025
 *      Author: bartoszlozinski
 */

#include "../Inc/Uart.hpp"
#include "../Inc/Timer.hpp"

UART2::UART2(const uint32_t baudRate)
{
	this->EnableClock();
	UartConfig(baudRate);
};

void UART2::UartConfig(const uint32_t baudRate)
{
	GPIOA->MODER &= ~GPIO_MODER_MODE2_0;//11 after reset -- analog = 0b10;
	//AF7 needs to be 0111 ___ AFR[0] low register [1] high
	GPIOA->AFR[0] |= GPIO_AFRL_AFSEL2_0;//datasheet alternate function AF7
	GPIOA->AFR[0] |= GPIO_AFRL_AFSEL2_1;
	GPIOA->AFR[0] |= GPIO_AFRL_AFSEL2_2;
	//OTYPER = 0b0 - push-pull for reset state
	//GPIOA->OSPEEDR = 0b00 - very low speed by reset state

	GPIOA->MODER &= ~GPIO_MODER_MODE3_0;
	GPIOA->AFR[0] |= GPIO_AFRL_AFSEL3_0;
	GPIOA->AFR[0] |= GPIO_AFRL_AFSEL3_1;
	GPIOA->AFR[0] |= GPIO_AFRL_AFSEL3_2;
	GPIOA->OTYPER |= GPIO_OTYPER_OT3; //why should it be open-drain
	//GPIOA->OSPEEDR = 0b00 - very low speed by reset state

	//TO BE REFACTORED FOR UART1 also!!!

	//UART2 clock enable
	RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;

	//USART Baud Rate Register BRR - speed of the USART1
	//UARTDIV (RM 40.8 USART baud rate register) = 4 000 000 / 115200 = 34,7
	//USART2 uses PCLK1 clock by default (reset state 00)
	//know from USART2SEL bits value
	USART2->BRR = 4000000 / baudRate;

	//frame 8m1 -- 0b00 - for USART_CR1 (reset value)
	//PCE parity control enable: 0 - disabled (reset value), 1 - enabled
	//USART_CR2 STOP register - 00 - 1 stop bit; (RM USART_CR2) reset value
	//USART_CR1 UE (UART enable bit) - 0 -disabled (reset value): 1 - enabled
	USART2->CR1 |= USART_CR1_UE; // UART enabled
	USART2->CR1 |= USART_CR1_TE; //transmitter enabled - 0 disable, 1- enabled
	USART2->CR1 |= USART_CR1_RE; //receiver enabled - 0 disable, 1- enabled
};

void UART2::SendChar(const char ch)
{
	//put data to transmit register
	//Transmit data register TDR
	USART2->TDR = ch;
	while(!(USART2->ISR & USART_ISR_TC)) {} //just wait
	//interrupt status register ISR - what happens inside UART
	//bit TXE - transmit data register empty
	//0 - data is not transferred to the shift register
	//1 - data is transferred to the shift register
	//TC transmission complete - 0 (not complete) - 1 (complete)
}

void UART2::SendString(const char str[])
{
	if (str)
	{
		while (*str != '\0')
		{
			SendChar(*str);
			str++;
		}
		SendChar('\r');
		SendChar('\n');
	}
}


ERROR_CODE UART2::GetChar()
{
	//_______________RECEIVE___________________
	//RXNE - dead data register is not empty
	//if not empty - we can receive something
	//0 - data is not received
	//1 - received data is ready to be read

	//by one sign
	using enum ERROR_CODE;
	if (USART2->ISR & USART_ISR_RXNE)
	{
		actualChar = USART2->RDR;
		return OK;
	}
	return NOK;
}

ERROR_CODE UART2::GetString()
{
	uint8_t i = 0;
	while (i < maxLength - 1)
	{
		Timer receiverTimer(50);
		if (USART2->ISR & USART_ISR_RXNE)
		{
			volatile char c = static_cast<char>(USART2->RDR);
			if (c == '\r' || c == '\n')
				break;
			else
				buffer_[i++] = c;
		}
		if (receiverTimer.IsExpired())
			break;
	}

	buffer_[i] = '\0';
	return (i > 0) ? ERROR_CODE::OK : ERROR_CODE::NOK;
}
