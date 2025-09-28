#pragma once
#include "ISpi.hpp"
#include "Config.hpp"

//////////////////////////////////////////////////
//////////////////////////////////////////////////
//			NEED TO ADD ALSO DMA				//
//////////////////////////////////////////////////
//////////////////////////////////////////////////

template<std::uintptr_t spiAddr_, SpiMode spiMode_>
class Spi : public ISpi<Spi<spiAddr_, spiMode_>>
{
protected:
public:
	static constexpr std::uintptr_t spiAddr = spiAddr_;
	static constexpr SpiMode spiMode = spiMode_;

	Spi(const Spi& source) = delete;
	Spi(Spi&& source) = delete;
	Spi& operator=(const Spi& source) = delete;
	Spi& operator=(Spi&& source) = delete;
	Spi()
	{
		this->EnableClock();
		this->ConfigureSPI();
	}
	~Spi() = default;
};
