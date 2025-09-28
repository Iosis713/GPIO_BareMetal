#pragma once;
#include "ISpiPins.hpp"

template<SpiSCK spiSCK_, SpiMOSI spiMOSI_>
class SpiPinsHalfDuplexTX : public ISpiPins
{
protected:
public:
	SpiPinsHalfDuplexTX(const SpiPinsHalfDuplexTX& source) = delete;
	SpiPinsHalfDuplexTX(SpiPinsHalfDuplexTX&& source) = delete;
	SpiPinsHalfDuplexTX& operator=(const SpiPinsHalfDuplexTX& source) = delete;
	SpiPinsHalfDuplexTX& operator=(SpiPinsHalfDuplexTX&& source) = delete;
	SpiPinsHalfDuplexTX() = default;

	static const inline auto sck = ConfigureSCK<spiSCK_>();
	static const inline auto mosi = ConfigureMOSI<spiMOSI_>();
};

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
