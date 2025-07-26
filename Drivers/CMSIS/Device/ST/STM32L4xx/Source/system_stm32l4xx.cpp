#include "stm32l4xx.h"

extern "C" {

// Default system clock after reset is MSI at 4 MHz
uint32_t SystemCoreClock = 4000000U;

void SystemInit() {
    // You can leave this empty if you're not modifying clocks or peripherals here.
    // Optionally, disable watchdogs, etc.
	SCB->CPACR |= (0xF << 20); //enable FPU 20, 22
	SystemCoreClockUpdate();
}

void SystemCoreClockUpdate() {
    // No dynamic clock switching in this setup — just hardcode MSI at 4 MHz
    SystemCoreClock = 4000000U;
}

}
