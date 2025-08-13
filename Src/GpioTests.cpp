#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "../Inc/Gpio.hpp"

const GPIO_TypeDef fakePort{};

struct FakeGpioRegisters 
{
    uint32_t MODER = 0;
    uint32_t OTYPER = 0;
    uint32_t OSPEEDR = 0;
    uint32_t PUPDR = 0;
    uint32_t IDR = 0;
    uint32_t ODR = 0;
};

struct FakeRCC
{
    uint32_t AHB2ENR = 0;
};

inline FakeGpioRegisters fakeGPIOARegisters;


class FakeGpio : public IGpio<FakeGpio>
{
public:
    static constexpr std::uintptr_t portAddr = reinterprret_cast<std::uintptr_t>(&fakeGPIOARegisters);
    static constexpr uint8_t pin = 5;
};

TEST(GpioTest, EnableClockSetsAHB2ENR)
{
    FakeGpio fakeGpio;
    ASSERT_TRUE((RCC->AHB2ENR & RCC_AHB2ENR_GPIAEN) != 0);
};


int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}