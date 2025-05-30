/*
 * UART.hpp
 *
 *  Created on: May 30, 2025
 *      Author: bartoszlozinski
 */

//TO connect with you linux terminal (for example at ubuntu)
//
// check tty com ports with
// dmesg | grep tty 		(or sudo if not allow due to reading the kernel buffer failed: Operation not permitted)
// screen /dev/tty*ACMx* [baud rate] (i.e. screen /dev/ttyACM1 11520
//
// to kill screen process: ctrl + A --> K to kill process
// to detach session:      ctrl + A --> D
// available sockets:      screen -ls
// reconnect:              screen -r [ID]
//
// if you cannot reconnect, try: lsof /dev/ttyACM1
// kill [PID]
//
// try once more

#include "Config.hpp"
#include "Gpio.hpp"

#ifndef UART_HPP_
#define UART_HPP_

template <std::uintptr_t portAddr_, uint8_t pinTX_, uint8_t pinRX_>
class UART : public IGpio<UART<portAddr_, pinTX_, pinRX_>>
{
protected:

	char actualChar;

	void UartConfig(const uint32_t baudRate = 115200)
	{
	    GPIOA->MODER &= ~MODER_OUTPUT_BITS[pinTX];//11 after reset -- analog = 0b10;
	    //AF7 needs to be 0111 ___ AFR[0] low register [1] high
	    GPIOA->AFR[0] |= AFRL_AFSEL_0_MASKS[pinTX];
	    GPIOA->AFR[0] |= AFRL_AFSEL_1_MASKS[pinTX];
	    GPIOA->AFR[0] |= AFRL_AFSEL_2_MASKS[pinTX];
	    //OTYPER = 0b0 - push-pull for reset state
	    //GPIOA->OSPEEDR = 0b00 - very low speed by reset state

	    GPIOA->MODER &= ~MODER_OUTPUT_BITS[pinRX];
	    GPIOA->AFR[0] |= AFRL_AFSEL_0_MASKS[pinRX];
	    GPIOA->AFR[0] |= AFRL_AFSEL_1_MASKS[pinRX];
	    GPIOA->AFR[0] |= AFRL_AFSEL_2_MASKS[pinRX];
	    GPIOA->OTYPER |= OTYPER_BITS[pinRX]; //why should it be open-drain
	    //GPIOA->OSPEEDR = 0b00 - very low speed by reset state

	    //TO BE REFACTORED FOR UART1 also!!!

	    //UART2 clock enable
	    RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;

	    //USART Baud Rate Register BRR - speed of the USART1
	    //UARTDIV (RM 40.8 USART baud rate register) = 4 000 000 / 115200 = 34,7
	    //USART2 uses PCLK1 clock by default (reset state 00)
	    //know from USART2SEL bits value
	    //USART2->BRR = ClockUtils::GetPLCK1Freq() / baudRate;
	    //just for now - to be refactored
	    USART2->BRR = 4000000 / baudRate;


	    //frame 8m1 -- 0b00 - for USART_CR1 (reset value)
	    //PCE parity control enable: 0 - disabled (reset value), 1 - enabled
	    //USART_CR2 STOP register - 00 - 1 stop bit; (RM USART_CR2) reset value
	    //USART_CR1 UE (UART enable bit) - 0 -disabled (reset value): 1 - enabled
	    USART2->CR1 |= USART_CR1_UE; // UART enabled
	    //transmitter and receiver enable TE/RE bits --> 0 - disabled, 1 - enabled
	    USART2->CR1 |= USART_CR1_TE; //transmitter enabled
	    USART2->CR1 |= USART_CR1_RE; //receiver enabled
	}

public:
	static constexpr std::uintptr_t portAddr = portAddr_;
	static constexpr uint8_t pinTX = pinTX_;
	static constexpr uint8_t pinRX = pinRX_;

	UART(const UART& source) = delete;
	UART(UART&& source) = delete;
	UART& operator=(const UART& source) = delete;
	UART& operator=(UART&& source) = delete;
	UART(const uint32_t baudRate = 115200)
	{
		this->EnableClock();
		UartConfig(baudRate);
	}

	void SendChar(const char ch)
	{

		//_____________TRANSMISSION_____________
		//put data to transmit register
		//Receive data register RDR
		//Transmit data register TDR
		USART2->TDR = ch;
		//while (!(USART2->ISR & USART_ISR_TXE)) {}

		//interrupt status register ISR - what happens inside UART
		//bit TXE - transmit data register empty
		//0 - data is not transferred to the shift register
		//1 - data is transferred to the shift register
		// wait when it's not transferred

		//or
		//TC transmission complete - 0 (not complete) - 1 (complete)
		while(!(USART2->ISR & USART_ISR_TC)) {} //just wait
	}

	void SendString(const char str[])
	{
		if (str)
		{
			while (*str != '\0')
			{
				SendChar(*str);
				str++;
			}
		}
	}

	ERROR_CODE GetChar()
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
	char GetActualChar() {return this->actualChar; }
};



#endif /* UART_HPP_ */
