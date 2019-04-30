/*--------------------------------------------------------------------------------
 * main.h
 */
#pragma once

#include <stm32f103xb.h>
#include <cmsis_gcc.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <common.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/times.h>
#include <sys/unistd.h>
#include <inttypes.h>
#include <string.h>
#include "vectors.h"
#include "sysclock.h"

#define BAUD 115200
#define sizeofs(a) (int)(sizeof(a)/sizeof(a[0]))

/*---------------------------------------------------------------------
 function signatures
 */
extern "C" void _init(void);
extern "C" int main(void);
static char *ftoa(char buffer[], float number, int digits, int *cnt=0, bool bround=true);

/*---------------------------------------------------------------------
  globals
  */
static const uint32_t APB2_DIV = 1;
static const uint32_t APB1_DIV = (F_CPU>36000000) ? 2 : 1;

volatile unsigned tickcnt;            // SysTick_Handler msec counter
