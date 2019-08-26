#!/bin/bash
cargo build --release && 
arm-none-eabi-objcopy -O binary target/thumbv7m-none-eabi/release/stm32-project stm32-project.bin