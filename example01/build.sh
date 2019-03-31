#!/bin/bash

set -x

export APP=example01
export SOURCE=$APP.c
export TARGET=$APP.elf
export OPT="-Os"
export IFLAGS="-I ../cmsis"
export CFLAGS="-g -std=gnu99 -Wall $OPT -mcpu=cortex-m3 -mthumb -fsingle-precision-constant $IFLAGS"
export LDFLAGS="-Tstm32f103c8.ld"

arm-none-eabi-gcc -o $TARGET $CFLAGS $SOURCE $LDFLAGS -nostartfiles -nostdlib &&
\
arm-none-eabi-readelf -x .isr_vectors $TARGET >$APP.lss; \
arm-none-eabi-objdump -CS $TARGET >> $APP.lss; \
arm-none-eabi-size  $TARGET
