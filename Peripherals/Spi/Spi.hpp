#pragma once
#include "ISpi.hpp"
#include "Config.hpp"

//////////////////////////////////////////////////
//////////////////////////////////////////////////
//			NEED TO ADD ALSO DMA				//
//////////////////////////////////////////////////
//////////////////////////////////////////////////

template<typename T>
concept SpiInterface = requires(T spi)
{
	{ spi.CR1 } -> std::convertible_to<volatile uint32_t>;
	{ spi.CR2 } -> std::convertible_to<volatile uint32_t>;
	{ spi.SR } -> std::convertible_to<volatile uint32_t>;
	{ spi.DR } -> std::convertible_to<volatile uint32_t>;
	{ spi.CRCPR } -> std::convertible_to<volatile uint32_t>;
	{ spi.RXCRCR } -> std::convertible_to<volatile uint32_t>;
	{ spi.TXCRCR } -> std::convertible_to<volatile uint32_t>;
};

template<SpiInterface SpiStruct, SpiMode spiMode_>
class Spi : public ISpi<Spi<SpiStruct, spiMode_>>
{
protected:
public:
	volatile SpiStruct* const spi = nullptr;
	static constexpr SpiMode spiMode = spiMode_;

	Spi(const Spi& source) = delete;
	Spi(Spi&& source) = delete;
	Spi& operator=(const Spi& source) = delete;
	Spi& operator=(Spi&& source) = delete;
	Spi() = delete;
	Spi(volatile SpiStruct* const spi_)
		: spi(spi_)
	{
		this->EnableClock();
		this->ConfigureSPI();
	}
	~Spi() = default;
};
