cmake_minimum_required(VERSION 3.28)
project(GPIO_BareMetal C CXX ASM)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(GTest REQUIRED)
set(GMOCK_LIBRARIES /usr/lib/libgmock.a /usr/lib/libgmock_main.a)

add_compile_definitions(STM32L476xx)
set(INCLUDE_DIRS
    ../Drivers/CMSIS/Device/ST/STM32L4xx/Include
    ../Drivers/CMSIS/Include
    ../Drivers/.
)

set(GLOBAL_DEFINES
    -DSTM32L476xx
)

include_directories(
    ${CMAKE_SOURCE_DIR}/../Drivers/CMSIS/Include
    ${CMAKE_SOURCE_DIR}/../Drivers/CMSIS/Device/ST/STM32L4xx/Include
)

set(C_SRCS
    Src/syscalls.c
    Src/sysmem.c
    #Src/main.cpp
    #Src/Timer.cpp
    #Src/Spi.cpp
    Drivers/CMSIS/Device/ST/STM32L4xx/Source/system_stm32l4xx.cpp
    )

add_executable(${PROJECT_NAME}-gt ${C_SRC} GpioTests.cpp)
target_compile_definitions(${PROJECT_NAME}-gt PRIVATE UNNIT_TESTS)


target_include_directories(${PROJECT_NAME}-gt PRIVATE
    ${CMAKE_SOURCE_DIR}/../Inc
    ${CMAKE_SOURCE_DIR}/../Peripherals/Gpio
    # Add other include directories here
)

target_link_libraries(${PROJECT_NAME}-gt ${GTEST_LIBRARIES} ${GMOCK_LIBRARIES})
target_compile_options(${PROJECT_NAME}-gt PRIVATE -Wall -Werror -Wextra)
set_target_properties(${PROJECT_NAME}-gt PROPERTIES CXX_STANDARD 20 CXX_STANDARD_REQUIRED ON)
