#ifndef UNIT_TESTS
#define UNIT_TESTS
#endif

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "../Peripherals/Gpio/IGpio.hpp"
#include "../Peripherals/Gpio/GpioOutput.hpp"
#include "../Peripherals/Gpio/GpioInput.hpp"

struct FakeGpioRegisters 
{
    volatile uint32_t MODER = 0;
    volatile uint32_t OTYPER = 0;
    volatile uint32_t OSPEEDR = 0;
    volatile uint32_t PUPDR = 0;
    volatile uint32_t IDR = 0;
    volatile uint32_t ODR = 0;
    volatile uint32_t BSRR = 0;
    volatile uint32_t LCKR = 0;
    volatile uint32_t AFR[2] = {0, 0};
    volatile uint32_t BRR  = 0;
    volatile uint32_t ASCR = 0;
};

struct FakeRCC
{
    uint32_t AHB2ENR = 0;
};

inline FakeGpioRegisters fakeGPIORegisters;

template<GpioPort Port>
class FakeGpio : public IGpio<FakeGpio<Port>>
{
public:
    Port* const port = &fakeGPIORegisters;
    static constexpr uint8_t pin = 5;
};

class FakeIGpioFixture : public testing::Test
{
public:
    FakeGpio<FakeGpioRegisters> fakeGpio;
    void TearDown() override
    {
    fakeGPIORegisters.MODER = 0;
    fakeGPIORegisters.OTYPER = 0;
    fakeGPIORegisters.OSPEEDR = 0;
    fakeGPIORegisters.PUPDR = 0;
    fakeGPIORegisters.IDR = 0;
    fakeGPIORegisters.ODR = 0;
    fakeGPIORegisters.BSRR = 0;
    fakeGPIORegisters.LCKR = 0;
    fakeGPIORegisters.AFR[0] = 0;
    fakeGPIORegisters.AFR[1] = 0;
    fakeGPIORegisters.BRR  = 0;
    fakeGPIORegisters.ASCR = 0;
    }

    static constexpr uint32_t bitMask = 0b11;
};

TEST_F(FakeIGpioFixture, ConfigurePUPDRNone)
{
    fakeGpio.template ConfigurePUPDR<OptionsPUPDR::None>();
    ASSERT_EQ(fakeGpio.port->PUPDR & (static_cast<uint32_t>(OptionsPUPDR::None) << 2 * fakeGpio.pin), 0);
}

TEST_F(FakeIGpioFixture, ConfigurePUPDRPullUp)
{
    fakeGpio.template ConfigurePUPDR<OptionsPUPDR::PullUp>();
    static constexpr uint8_t bitShift = 2 * fakeGpio.pin;
    ASSERT_EQ((fakeGpio.port->PUPDR >> bitShift) & bitMask, static_cast<uint32_t>(OptionsPUPDR::PullUp));
};

TEST_F(FakeIGpioFixture, ConfigurePUPDRPullDown)
{
    fakeGpio.template ConfigurePUPDR<OptionsPUPDR::PullDown>();
    static constexpr uint8_t bitShift = 2 * fakeGpio.pin;
    ASSERT_EQ((fakeGpio.port->PUPDR >> bitShift) & bitMask , static_cast<uint32_t>(OptionsPUPDR::PullDown));
}

//MODER TESTS
TEST_F(FakeIGpioFixture, ConfigureMODERInput)
{
    fakeGpio.template ConfigureMODER<OptionsMODER::Input>();
    static constexpr uint8_t bitShift = 2 * fakeGpio.pin;
    ASSERT_EQ((fakeGpio.port->MODER >> bitShift) & bitMask, static_cast<uint32_t>(OptionsMODER::Input));
}

TEST_F(FakeIGpioFixture, ConfigureMODEROutput)
{
    fakeGpio.template ConfigureMODER<OptionsMODER::Output>();
    static constexpr uint8_t bitShift = 2 * fakeGpio.pin;
    ASSERT_EQ((fakeGpio.port->MODER >> bitShift) & bitMask, static_cast<uint32_t>(OptionsMODER::Output));
}

TEST_F(FakeIGpioFixture, ConfigureMODERAlternate)
{
    fakeGpio.template ConfigureMODER<OptionsMODER::Alternate>();
    static constexpr uint8_t bitShift = 2 * fakeGpio.pin;
    ASSERT_EQ((fakeGpio.port->MODER >> bitShift) & bitMask, static_cast<uint32_t>(OptionsMODER::Alternate));
}

TEST_F(FakeIGpioFixture, ConfigureMODERAnalog)
{
    fakeGpio.template ConfigureMODER<OptionsMODER::Analog>();
    static constexpr uint8_t bitShift = 2 * fakeGpio.pin;
    ASSERT_EQ((fakeGpio.port->MODER >> bitShift) & bitMask, static_cast<uint32_t>(OptionsMODER::Analog));
}

//OSPEEDR TESTS
TEST_F(FakeIGpioFixture, ConfigureOSPEEDRLowSpeed)
{
    fakeGpio.template ConfigureOSPEEDR<OptionsOSPEEDR::LowSpeed>();
    static constexpr uint8_t bitShift = 2 * fakeGpio.pin;
    ASSERT_EQ((fakeGpio.port->OSPEEDR >> bitShift) & bitMask, static_cast<uint32_t>(OptionsOSPEEDR::LowSpeed));
}

TEST_F(FakeIGpioFixture, ConfigureOSPEEDRMediumSpeed)
{
    fakeGpio.template ConfigureOSPEEDR<OptionsOSPEEDR::MediumSpeed>();
    static constexpr uint8_t bitShift = 2 * fakeGpio.pin;
    ASSERT_EQ((fakeGpio.port->OSPEEDR >> bitShift) & bitMask, static_cast<uint32_t>(OptionsOSPEEDR::MediumSpeed));
}

TEST_F(FakeIGpioFixture, ConfigureOSPEEDRHighSpeed)
{
    fakeGpio.template ConfigureOSPEEDR<OptionsOSPEEDR::HighSpeed>();
    static constexpr uint8_t bitShift = 2 * fakeGpio.pin;
    ASSERT_EQ((fakeGpio.port->OSPEEDR >> bitShift) & bitMask, static_cast<uint32_t>(OptionsOSPEEDR::HighSpeed));
}

TEST_F(FakeIGpioFixture, ConfigureOSPEEDRVeryHighSpeed)
{
    fakeGpio.template ConfigureOSPEEDR<OptionsOSPEEDR::VeryHighSpeed>();
    static constexpr uint8_t bitShift = 2 * fakeGpio.pin;
    ASSERT_EQ((fakeGpio.port->OSPEEDR >> bitShift) & bitMask, static_cast<uint32_t>(OptionsOSPEEDR::VeryHighSpeed));
}

//Alternate TESTS
TEST_F(FakeIGpioFixture, ConfigureAlternateFunctionTEST)
{
    using enum AlternateFunction;
    std::array<AlternateFunction, 16> alternateFunctions{ AF0, AF1, AF2, AF3, AF4, AF5, AF6, AF7, AF8, AF9, AF10, AF11, AF12, AF13, AF14, AF15 };
    static constexpr uint8_t bitShift = 4 * fakeGpio.pin;
    static constexpr uint8_t bitMask = 0b1111;

    for(const auto af : alternateFunctions)
    {
        fakeGpio.ConfigureAlternateFunction(af);
        ASSERT_EQ((fakeGpio.port->AFR[0] >> bitShift) & bitMask, static_cast<uint32_t>(af));
    }
}

////////////////////////////////////////////////////////
//                                                    //
//                   GPIO OUTPUT TESTS                //
//                                                    //
////////////////////////////////////////////////////////

/*
template<GpioPort Port>
class FakeGpioOutput : public GpioOutput<FakeGpio<Port>, 5>
{
public:
    Port* const port = &fakeGPIORegisters;
};
*/

class FakeGpioOutputFixture : public testing::Test
{
public:
    GpioOutput<FakeGpioRegisters, 5> fakeGpio{&fakeGPIORegisters};
    void TearDown() override
    {
    fakeGPIORegisters.MODER = 0;
    fakeGPIORegisters.OTYPER = 0;
    fakeGPIORegisters.OSPEEDR = 0;
    fakeGPIORegisters.PUPDR = 0;
    fakeGPIORegisters.IDR = 0;
    fakeGPIORegisters.ODR = 0;
    fakeGPIORegisters.BSRR = 0;
    fakeGPIORegisters.LCKR = 0;
    fakeGPIORegisters.AFR[0] = 0;
    fakeGPIORegisters.AFR[1] = 0;
    fakeGPIORegisters.BRR  = 0;
    fakeGPIORegisters.ASCR = 0;
    }

    static constexpr uint32_t bitMask = 0b11;
};

TEST_F(FakeGpioOutputFixture, IsPinSetNegativeTest)
{
    //0 by default
    ASSERT_FALSE(fakeGpio.IsPinSet());
}

TEST_F(FakeGpioOutputFixture, IsPinSetPositiveTest)
{
    //0 by default
    fakeGpio.port->ODR |= (0b1 << fakeGpio.pin);
    ASSERT_TRUE(fakeGpio.IsPinSet());
}

TEST_F(FakeGpioOutputFixture, SetTest)
{
    fakeGpio.Set();
    static constexpr uint32_t bsrrMask = 0b1;
    ASSERT_EQ((fakeGpio.port->BSRR >> fakeGpio.pin) & bsrrMask, bsrrMask);
}

TEST_F(FakeGpioOutputFixture, ClearTest)
{
    fakeGpio.Clear();
    static constexpr uint32_t bsrrMask = 0b1;
    static constexpr uint8_t bitShift = 16 + fakeGpio.pin;
    ASSERT_EQ((fakeGpio.port->BSRR >> bitShift) & bsrrMask, bsrrMask);
}

TEST_F(FakeGpioOutputFixture, ToggleTest)
{
    ASSERT_FALSE(fakeGpio.IsPinSet());
    fakeGpio.Toggle();
    ASSERT_TRUE(fakeGpio.IsPinSet());
}
