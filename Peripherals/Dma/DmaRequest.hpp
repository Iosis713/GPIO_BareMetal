#pragma once
#include <cstdint>

enum class DMA1Request : uint8_t {
    ADC1_Request        = 0b0000,
    ADC2_Request        = 0b0000, // same request ID but different channel
    ADC3_Request        = 0b0000,
    FLT0        = 0b0000,
    FLT1        = 0b0000,
    FLT2        = 0b0000,
    FLT3        = 0b0000,

    SPI1_RX     = 0b0001,
    SPI1_TX     = 0b0001,
    SPI2_RX     = 0b0001,
    SPI2_TX     = 0b0001,
    SAI2_A      = 0b0001,
    SAI2_B      = 0b0001,

    USART3_TX   = 0b0010,
    USART3_RX   = 0b0010,
    USART1_TX   = 0b0010,
    USART1_RX   = 0b0010,
    USART2_RX   = 0b0010,
    USART2_TX   = 0b0010,

    I2C3_TX     = 0b0011,
    I2C3_RX     = 0b0011,
    I2C2_TX     = 0b0011,
    I2C2_RX     = 0b0011,
    I2C1_TX     = 0b0011,
    I2C1_RX     = 0b0011,
    TIM16_CH1   = 0b0011,

    TIM2_CH3    = 0b0100,
    TIM2_UP     = 0b0100,
    TIM2_CH1    = 0b0100,
    TIM16_UP    = 0b0100,
    TIM2_CH4    = 0b0100,
    TIM17_CH1   = 0b0100,
    TIM3_CH4    = 0b0100,
    TIM7_UP     = 0b0100,
    TIM3_CH1    = 0b0100,
    TIM17_CH1_2 = 0b0100, // second occurrence

    TIM3_CH3    = 0b0101,
    QUADSPI_Request     = 0b0101,
    TIM17_UP    = 0b0101,
    TIM3_UP     = 0b0101,
    DAC_CH2     = 0b0101,
    TIM3_TRIG   = 0b0101,
    TIM17_UP2   = 0b0101,
    TIM6_UP     = 0b0101,

    TIM4_CH1    = 0b0110,
    TIM4_CH2    = 0b0110,
    TIM4_CH3    = 0b0110,
    TIM4_UP     = 0b0110,
    DAC_CH1     = 0b0110,
    TIM15_CH1   = 0b0110,
    TIM1_CH4    = 0b0110,
    TIM15_UP    = 0b0110,

    TIM1_CH1    = 0b0111,
    TIM1_CH2    = 0b0111,
    TIM1_TRIG   = 0b0111,
    TIM1_UP     = 0b0111,
    TIM1_CH3    = 0b0111,
    TIM15_TRIG  = 0b0111,
    TIM1_COM    = 0b0111,
    TIM15_COM   = 0b0111
};

// DMA2 request IDs
enum class DMA2Request : uint8_t {
    I2C4_RX     = 0b0000,
    I2C4_TX     = 0b0000,
    ADC1_Request        = 0b0000,
    ADC2_Request        = 0b0000,
    ADC3_Request        = 0b0000,
    DCMI        = 0b0000,

    SAI1_A      = 0b0001,
    SAI1_B      = 0b0001,
    SAI2_A      = 0b0001,
    SAI2_B      = 0b0001,

    UART5_TX    = 0b0010,
    UART5_RX    = 0b0010,
    UART4_TX    = 0b0010,
    UART4_RX    = 0b0010,
    USART1_TX   = 0b0010,
    USART1_RX   = 0b0010,
    TIM6_UP     = 0b0010,
    TIM7_UP     = 0b0010,

    SPI3_RX     = 0b0011,
    SPI3_TX     = 0b0011,
    QUADSPI_Request     = 0b0011,
    DAC_CH1     = 0b0011,
    DAC_CH2     = 0b0011,
    LPUART1_RX  = 0b0011,
    LPUART1_TX  = 0b0011,

    SWPMI1_RX   = 0b0100,
    SWPMI1_TX   = 0b0100,
    SPI1_RX     = 0b0100,
    SPI1_TX     = 0b0100,
    DCMI_TX     = 0b0100,
    DCMI_RX     = 0b0100,
    TIM5_CH4    = 0b0100,
    TIM5_CH3    = 0b0100,

    TIM5_CH2    = 0b0101,
    TIM5_CH1    = 0b0101,
    I2C1_RX     = 0b0101,
    I2C1_TX     = 0b0101,
    TIM5_TRIG   = 0b0101,
    TIM5_UP     = 0b0101,

    AES_IN      = 0b0110,
    AES_OUT     = 0b0110,
    HASH_IN     = 0b0110,
    TIM8_CH4    = 0b0110,
    TIM8_CH3    = 0b0110,

    TIM8_TRIG   = 0b0111,
    SDMMC1_Request      = 0b0111,
    TIM8_CH1    = 0b0111,
    TIM8_CH2    = 0b0111,
    TIM8_UP     = 0b0111,
    TIM8_COM    = 0b0111
};
