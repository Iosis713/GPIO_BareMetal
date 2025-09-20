cmake_minimum_required(VERSION 3.28)
enable_language(ASM)
project(GPIO_BareMetal C CXX ASM)

set(MAP_FILE "${CMAKE_BINARY_DIR}/${PROJECT_NAME}.map")
set(LD_SCRIPT ${CMAKE_SOURCE_DIR}/STM32L476RGTX_FLASH.ld)
set(LINKER_FLAGS "-Wl,-Map=${MAP_FILE},--cref -T${LD_SCRIPT} -g -Wl,--gc-sections")
set(CMAKE_EXE_LINKER_FLAGS "${LINKER_FLAGS}")

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_TOOLCHAIN_FILE cmake/toolchain-arm-gcc.cmake)
include(cmake/compiler_flags.cmake)

add_subdirectory(Drivers)
add_compile_definitions(STM32L476xx)
include_directories(
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Include
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32L4xx/Include
)

set(C_SRCS
    Src/syscalls.c
    Src/sysmem.c
    Src/main.cpp
    Src/Timer.cpp
    Src/Spi.cpp
    Peripherals/Dma/DmaChannel.cpp
    Startup/startup_stm32l476rgtx.s
    Drivers/CMSIS/Device/ST/STM32L4xx/Source/system_stm32l4xx.cpp)

set(ASM_SRCS Startup/startup_stm32l476rgtx.s)
set(GLOBAL_DEFINES
    -DSTM32L476xx)

add_executable(${PROJECT_NAME} ${C_SRCS} ${ASM_SRCS})

target_compile_definitions(${PROJECT_NAME} PRIVATE
    STM32L476xx
)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}/Inc
    ${CMAKE_SOURCE_DIR}/Peripherals/Gpio
    # Add other include directories here
)
