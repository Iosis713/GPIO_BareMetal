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

class FakeGpioFixture : public testing::Test
{
public:
    FakeGpio<FakeGpioRegisters> fakeGpio;
    void TearDown() override;
};

TEST_F(FakeGpioFixture, ConfigurePUPDRPullUp)
{
    //FakeGpio<FakeGpioRegisters> fakeGpio;
    //this->fakeGpio.template ConfigurePUPDR<OptionsPUPDR::PullUp>();
    ASSERT_TRUE(false);
};


int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

void FakeGpioFixture::TearDown()
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