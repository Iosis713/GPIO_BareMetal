#pragma once
#include "ISpiPins.hpp"

template<SpiSCK spiSCK_, SpiMOSI spiMISO_>
class SpiPinsHalfDuplexRX : public ISpiPins
{
protected:
public:
	SpiPinsHalfDuplexRX(const SpiPinsHalfDuplexRX& source) = delete;
	SpiPinsHalfDuplexRX(SpiPinsHalfDuplexRX&& source) = delete;
	SpiPinsHalfDuplexRX& operator=(const SpiPinsHalfDuplexRX& source) = delete;
	SpiPinsHalfDuplexRX& operator=(SpiPinsHalfDuplexRX&& source) = delete;
	SpiPinsHalfDuplexRX() = default;

	static constexpr auto sck = ConfigureSCK<spiSCK_>();
	static constexpr auto miso = ConfigureMOSI<spiMISO_>();
};
