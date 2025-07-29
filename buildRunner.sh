#!/bin/bash

if [ ! -d build ]; then
    mkdir build
fi

cd build
cmake .. -GNinja -DCMAKE_TOOLCHAIN_FILE=cmake/toolchain-arm-gcc.cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
ninja

cd ..

