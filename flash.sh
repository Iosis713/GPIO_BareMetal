#!/bin/bash

openocd -f interface/stlink.cfg -f target/stm32l4x.cfg   -c "program build/GPIO_BareMetal verify reset exit"

