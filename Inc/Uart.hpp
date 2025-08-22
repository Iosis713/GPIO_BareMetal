#pragma once
//TO connect with you linux terminal (for example at ubuntu)
//
// check tty com ports with
// dmesg | grep tty 		(or sudo if not allow due to reading the kernel buffer failed: Operation not permitted)
// screen /dev/tty*ACMx* [baud rate] (i.e. screen /dev/ttyACM1 115200
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
#include <array>
#include <concepts>

#include "GpioAlternate.hpp"
#include "Timer.hpp"

//concept basing on USART_TypeDef struct from CMSIS
template<typename T>
concept USARTx = requires (T uart)
{
	{ uart.CR1 } 		-> std::convertible_to<volatile uint32_t&>;		/*!< USART Control register 1,                 Address offset: 0x00 */
    { uart.CR2 } 		-> std::convertible_to<volatile uint32_t&>;		/*!< USART Control register 2,                 Address offset: 0x04 */
    { uart.CR3 } 		-> std::convertible_to<volatile uint32_t&>;		/*!< USART Control register 3,                 Address offset: 0x08 */
    { uart.BRR } 		-> std::convertible_to<volatile uint32_t&>;		/*!< USART Baud rate register,                 Address offset: 0x0C */
    { uart.GTPR}  		-> std::convertible_to<volatile uint16_t&>;		/*!< USART Guard time and prescaler register,  Address offset: 0x10 */
    { uart.RESERVED2} 	-> std::convertible_to<uint16_t&>;     			/*!< Reserved, 0x12                                                 */
    { uart.RTOR } 		-> std::convertible_to<volatile uint32_t&>;     /*!< USART Receiver Time Out register,         Address offset: 0x14 */
	{ uart.RQR} 		-> std::convertible_to<volatile uint16_t&>;     /*!< USART Request register,                   Address offset: 0x18 */
    { uart.RESERVED3 } 	-> std::convertible_to<uint16_t&>;       		/*!< Reserved, 0x1A                                                 */
	{ uart.ISR} 		-> std::convertible_to<volatile uint32_t&>;     /*!< USART Interrupt and status register,      Address offset: 0x1C */
	{ uart.ICR } 		-> std::convertible_to<volatile uint32_t&>;     /*!< USART Interrupt flag Clear register,      Address offset: 0x20 */
	{ uart.RDR }        -> std::convertible_to<volatile uint16_t&>;		/*!< USART Receive Data register,              Address offset: 0x24 */
	{ uart.RESERVED4 }  -> std::convertible_to<uint16_t&>;				/*!< Reserved, 0x26                                                 */
	{ uart.TDR }        -> std::convertible_to<volatile uint16_t&>;		/*!< USART Transmit Data register,             Address offset: 0x28 */
	{ uart.RESERVED5 }  -> std::convertible_to<uint16_t&>;				/*!< Reserved, 0x2A                                                 */
};

template<USARTx Usart, GpioPort RxTx, uint32_t baudRate = 115200, std::size_t bufferSize = 80>
class UART
{
protected:
	volatile Usart* const usart = nullptr;
	volatile RxTx* rx = nullptr;
	volatile RxTx* tx = nullptr;

	char actualChar;
	std::array<char, bufferSize> buffer_ {};

	inline void EnableClock() { RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN; };
	
	void ConfigureTX()
	{
		/*
		temp object, UB =/
		if (usart == USART2)
		{
			tx = GpioAlternate<GPIO_TypeDef, 3, AlternateFunction::AF7>(GPIOA);
		}*/

		//TX PA2
		GPIOA->MODER &= ~GPIO_MODER_MODE2_0;//11 after reset -- analog = 0b10;
		//AF7 needs to be 0111 ___ AFR[0] low register [1] high
		GPIOA->AFR[0] |= GPIO_AFRL_AFSEL2_0;//datasheet alternate function AF7
		GPIOA->AFR[0] |= GPIO_AFRL_AFSEL2_1;
		GPIOA->AFR[0] |= GPIO_AFRL_AFSEL2_2;
		//OTYPER = 0b0 - push-pull for reset state
		//GPIOA->OSPEEDR = 0b00 - very low speed by reset state
	}

	void ConfigureRX()
	{
		//RX PA3
		GPIOA->MODER &= ~GPIO_MODER_MODE3_0;
		GPIOA->AFR[0] |= GPIO_AFRL_AFSEL3_0;
		GPIOA->AFR[0] |= GPIO_AFRL_AFSEL3_1;
		GPIOA->AFR[0] |= GPIO_AFRL_AFSEL3_2;
		GPIOA->OTYPER |= GPIO_OTYPER_OT3; //why should it be open-drain
		//GPIOA->OSPEEDR = 0b00 - very low speed by reset state
	}
		//TO BE REFACTORED FOR UART1 also!!!


	void UartConfig()
	{
		ConfigureTX();
		ConfigureRX();
		//UART2 clock enable
		if (usart == USART2)
			RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;

		//USART Baud Rate Register BRR - speed of the USART1
		//UARTDIV (RM 40.8 USART baud rate register) = 4 000 000 / 115200 = 34,7
		//USART2 uses PCLK1 clock by default (reset state 00)
		//know from USART2SEL bits value
		usart->BRR = 4000000 / baudRate;

		//frame 8m1 -- 0b00 - for USART_CR1 (reset value)
		//PCE parity control enable: 0 - disabled (reset value), 1 - enabled
		//USART_CR2 STOP register - 00 - 1 stop bit; (RM USART_CR2) reset value
		//USART_CR1 UE (UART enable bit) - 0 -disabled (reset value): 1 - enabled
		usart->CR1 |= USART_CR1_UE; // UART enabled
		usart->CR1 |= USART_CR1_TE; //transmitter enabled - 0 disable, 1- enabled
		usart->CR1 |= USART_CR1_RE; //receiver enabled - 0 disable, 1- enabled
	};
	volatile bool messageReady_ = false;
	std::array<char, bufferSize>::iterator index_;


public:
	UART(const UART& source) = delete;
	UART(UART&& source) = delete;
	UART& operator=(const UART& source) = delete;
	UART& operator=(UART&& source) = delete;
	UART(Usart* const usart_) 
		: usart(usart_)
		, index_(buffer_.begin())
	{
		this->EnableClock();
		UartConfig();
	};

	void SendChar(const char ch)
	{
		//put data to transmit register
		//Transmit data register TDR
		usart->TDR = ch;
		while(!(USART2->ISR & USART_ISR_TC)) {} //just wait
		//interrupt status register ISR - what happens inside UART
		//bit TXE - transmit data register empty
		//0 - data is not transferred to the shift register
		//1 - data is transferred to the shift register
		//TC transmission complete - 0 (not complete) - 1 (complete)
	};
	void SendString(const char str[])
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
	};

	ERROR_CODE GetChar()
	{
		//_______________RECEIVE___________________
		//RXNE - read data register is not empty
		//if not empty - we can receive something
		//0 - data is not received
		//1 - received data is ready to be read

		//by one sign
		using enum ERROR_CODE;
		if (usart->ISR & USART_ISR_RXNE)
		{
			actualChar = usart->RDR;
			return OK;
		}
		return NOK;
	};
	ERROR_CODE GetString()
	{
		uint8_t i = 0;
		while (i < buffer_.size() - 1)
		{
			Timer receiverTimer(50);
			if (usart->ISR & USART_ISR_RXNE)
			{
				const char c = static_cast<char>(usart->RDR);
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
	};
	ERROR_CODE GetStringIT()
	{
		if (messageReady_)
		{
			messageReady_ = false;
			return ERROR_CODE::OK;
		}
		return ERROR_CODE::NOK;
	};
	inline std::array<char, bufferSize>GetBuffer() { return this->buffer_; }
	inline char GetActualChar() { return this->actualChar; }

	void ConfigureExtiReceive()
	{
		RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; //enable SYSCFG clock
		//9.2.6 System configuration controller SYSCFG
		SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI3; //0000
		SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI3_PA; //set bit for PA3 exti route to syscfg

		usart->CR1 |= USART_CR1_RXNEIE; //Enable RX interrupt
		NVIC_SetPriority(USART2_IRQn, 1); //set priority (for exti ,  priotity = 1
		NVIC_EnableIRQ(USART2_IRQn);//enable interrupt
		//enum from stm32l476xx.h (CMSIS file) - Interrupt number definition
	};
	void ClearBuffer()
	{
		index_ = buffer_.begin();
		buffer_.fill('\0');
	};
	void IRQ_Handler()
	{
		using enum ERROR_CODE;
		//read data register is not empty (1)
		if (usart->ISR & USART_ISR_RXNE)
		{
			const char received = usart->RDR; //Read data register
			if (received == '\n' || received == '\r' || index_ >= buffer_.end() - 1)
			{
				*index_ = '\0';
				messageReady_ = true;
				index_ = buffer_.begin();
			}
			else
			{
				*index_ = received;
				index_++;
			}
		}
	};

};
