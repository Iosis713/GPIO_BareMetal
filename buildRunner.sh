#!/bin/bash

if [ ! -d build ]; then
    mkdir build
fi

cd build
cmake .. -GNinja -DCMAKE_TOOLCHAIN_FILE=cmake/toolchain-arm-gcc.cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
ninja

cd ..
if [ ! -d buildTests ]; then
    mkdir buildTests
fi

cd buildTests
cmake ../UnitTests -G Ninja -DCMAKE_CXX_COMPILER=g++-13
ninja
cd ..
