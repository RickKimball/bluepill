/*---------------------------------------------------------------------
  main.cpp - blink pc13 using systick timer in c++ 

  (still the about the same size as the c version)
*/

#include <stm32f103xb.h>
#include <cmsis_gcc.h>
#include <common.h>
#include "utils.h"

extern "C" int main();

volatile unsigned tickcnt; // SysTick increments this

/*---------------------------------------------------------------------
  _init() - initialize board

  NOTE: called from __libc_init(). It must be a c routine
*/
extern "C" void _init(void);

void _init(void) {
  // turn on GPIOC clock
  RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;

  // PC13 set to opendrain
  GPIOC->CRH = (GPIOC->CRH & ~(0b1111<<20)) | (0b0001<<20);

  tickcnt=0xffffffff-5000; // initialize it 5 secs from roll over so we can test that
  SysTick_Config(F_CPU/1000);
}

int main()
{
  while(1) {
    // turn on led - pull to gnd
    GPIOC->BSRR = (1<<13) << 16;
    delay(50);
    
    // turn off led - let it float
    GPIOC->BSRR = (1<<13);
    delay(200);
    
  }
  return 0;
}

/* vim: set expandtab ts=2 sw=2: */
