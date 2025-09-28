#pragma once
#include "ISpiPins.hpp"

template<SpiSCK spiSCK_, SpiMISO spiMISO_, SpiMOSI spiMOSI_>
class SpiPinsFullDuplex : public ISpiPins
{
protected:
public:
	SpiPinsFullDuplex(const SpiPinsFullDuplex& source) = delete;
	SpiPinsFullDuplex(SpiPinsFullDuplex&& source) = delete;
	SpiPinsFullDuplex& operator=(const SpiPinsFullDuplex& source) = delete;
	SpiPinsFullDuplex& operator=(SpiPinsFullDuplex&& source) = delete;
	SpiPinsFullDuplex() = default;

	static const inline auto sck = ConfigureSCK<spiSCK_>();
	static const inline auto miso = ConfigureMISO<spiMISO_>();
	static const inline auto mosi = ConfigureMOSI<spiMOSI_>();
};

