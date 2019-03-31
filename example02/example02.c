// vim: set expandtab ts=2 sw=2:

// example02.c - minimal c code using stm32f103xb.h device header only 
// no stdlib, no gcc startup code, no gcc libs
// self contained VTOR table

#include <stm32f103xb.h>
#include <common.h>

typedef void (* const vector)(void);

void delay(unsigned a);
__attribute__((used, naked)) void _start(void);

extern unsigned _estack; // this comes from stm32f103c8.ld

__attribute((used, section(".isr_vectors")))
const vector excp_handlers[1+14] = 
{
  (void *)&_estack, _start
};

int main() {
  RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;

  // PC13 set to opendrain
  GPIOC->CRH = (GPIOC->CRH & ~(0b1111<<20)) | (0b0001<<20);

  while(1) {
    // turn on - pull to gnd
    GPIOC->ODR &= ~(1<<13);
    delay(100000);
    // turn off - let it float
    GPIOC->ODR |= 1 << 13;
    delay(400000);
  }
  return 0;
}

void delay(unsigned a) {
  volatile unsigned b=0;

  while(b++ < a);

  return;
} 

// _start - the reset exception handler

void _start(void) {
  // set stack pointer to top of stack memory
  //asm volatile ("ldr r0, =%0" :: "i" (&_estack));
  asm volatile ("ldr r0, =0x20005000" :: "i" (&_estack));
  asm volatile ("mov sp, r0");

#if 0
  /* you should really:
     memset bss to zeros,
     memcpy flash data to ram data
     call global constructors * even c has them
     then call main
  */

  // bss zero
  extern unsigned _sbss, _ebss;
  unsigned *dst;
  for(dst=&_sbss; dst < &_ebss;)
    *dst++ = 0;

  // data copy
  unsigned *src;
  extern unsigned _sidata, _sdata, _edata;
  for(dst=&_sdata,src=&_sidata; dst < &_edata; )
    *dst++ = *src++;
#endif

  main();

  while(1); // trap if main exits
}

