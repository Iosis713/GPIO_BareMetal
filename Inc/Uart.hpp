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
#include "array"

#ifndef UART_HPP_
#define UART_HPP_

class UART2
{
protected:

	char actualChar;
	std::array<char, 80> buffer_ {};
	inline void EnableClock() { RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN; };
	void UartConfig(const uint32_t baudRate = 115200);
	//void StoreReceivedCharIT(const char c);

public:
	UART2(const UART2& source) = delete;
	UART2(UART2&& source) = delete;
	UART2& operator=(const UART2& source) = delete;
	UART2& operator=(UART2&& source) = delete;
	UART2(const uint32_t baudRate = 115200);

	void SendChar(const char ch);
	void SendString(const char str[]);

	ERROR_CODE GetChar();
	ERROR_CODE GetString();
	inline std::array<char, 80>GetBuffer() { return this->buffer_; }
	inline char GetActualChar() { return this->actualChar; }

	void ConfigureExtiReceive();
};

#endif /* UART_HPP_ */
