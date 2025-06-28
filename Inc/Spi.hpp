/*
 * Spi.hpp
 *
 *  Created on: Jun 23, 2025
 *      Author: bartoszlozinski
 */

#ifndef SPI_HPP_
#define SPI_HPP_

#include "Config.hpp"
#include "Gpio.hpp"

//just for spi 2 right now, according to:
//https://forbot.pl/blog/kurs-stm32l4-ekspander-portow-spi-quiz-id47763

//PC3 - MOSI
//PC2 - MISO
//PB

void SpiConfig();
void EnableSpiClocks();
void Spi2Transmit(const uint8_t data);
uint8_t Spi2Receive();

#endif /* SPI_HPP_ */
