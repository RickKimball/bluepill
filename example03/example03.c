/*
  vim: set expandtab ts=2 sw=2:

  example03.c - blink PC13 using SysTick clock
  
  Uses only stm32f103xb.h device header, no libc
  start up code. Self contained exception/vector
  table written in 'C'.

  $ make all
  generating example03.elf
  done!
     text    data     bss     dec     hex filename
      232       0       4     236      ec example03.elf


*/

#include <stm32f103xb.h>
#include <common.h>

int main() {
  RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;

  // PC13 set to opendrain
  GPIOC->CRH = (GPIOC->CRH & ~(0b1111<<20)) | (0b0001<<20);

  unsigned delay_until_tickcnt=tickcnt+100;

  while(1) {
    // turn on - pull to gnd
    GPIOC->ODR &= ~(1<<13);

    delay_until_tickcnt += 50;
    while( tickcnt < delay_until_tickcnt) { __WFE(); }
    
    // turn off - let it float
    GPIOC->ODR |= 1 << 13;

    delay_until_tickcnt += 450;
    while( tickcnt < delay_until_tickcnt) { __WFE(); }
    
  }
  return 0;
}
