#!/bin/bash
stm32flash -w stm32-project.bin -v -g 0x08000000 /dev/ttyUSB0
