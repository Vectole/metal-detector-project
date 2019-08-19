#!/bin/bash
#stm32flash -w ./target/thumbv7m-none-eabi/release/stm32-blink/stm32-project.bin -v -g 0x08000000 /dev/ttyUSB0
arm-none-eabi-objcopy -O binary target/thumbv7m-none-eabi/release/stm32-project stm32-project.bin
stm32flash -w stm32-project.bin -v -g 0x08000000 /dev/ttyUSB0
