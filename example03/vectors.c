/*
  vim: set expandtab ts=2 sw=2:

  vectors.c - the exception/isr handler code and the vector table (VTOR)

*/

#include <stm32f103xb.h>

extern int main();

extern unsigned _estack;   // this comes from stm32f103c8.ld
volatile unsigned tickcnt; // SysTick counter available to main()

/*
 _init() - initialize board

 Setup the SysTick clock for 1 msec ticks

*/

void _init() {
  tickcnt=0; // as we aren't zeroing bss, we do this

  SysTick_Config(F_CPU/1000);
}

__attribute__((used, naked))
void Reset_Handler(void);

/*
 Reset_Handler() - the reset exception handler

 Sets stack pointer and optionally inits .bss and .data
 then calls main.

 */

void Reset_Handler(void) {

  // set stack pointer to top of stack memory
  asm volatile ("ldr r0, =%0" :: "i" (&_estack));
  asm volatile ("mov sp, r0");

#if 0
  /* in most code you should really:
     memset bss to zeros,
     memcpy flash data to ram data
     call global constructors * even c has them
     then call main
  */

  // bss zero
  extern unsigned __bss_start__, __bss_end__;
  unsigned *dst;
  for(dst=&__bss_start__; dst < &__bss_end__;)
    *dst++ = 0;

  // data copy
  unsigned *src;
  extern unsigned _sidata, _sdata, _edata;
  for(dst=&_sdata,src=&_sidata; dst < &_edata; )
    *dst++ = *src++;

#endif

  _init();
  main();

  while(1); // trap if main exits
}


/*
  SysTick_Handler() - SYSTICK exception handler.

  Increment a tick counter each time we get an interrupt
  from the SYSTICK.

  */

void SysTick_Handler(void) {
  ++tickcnt;
}

/*
  VTOR vector table - must begin at 0x08000000
  
  We just fill in the ones we are using

  [0] = initial stack address (grows down)
  [1] = reset exception handler address
  ..
  [15] = systick exception handler address
  [16] = first peripheral ISR handler address
  ...
  [..] = last .. we aren't using the peripheral ISRs yet
        so we don't even declare them

  */

typedef void (* const vector)(void); // const pointer to function(void) return void

__attribute((used, section(".isr_vectors")))
static const vector vectors[1+15] = 
{
  (void * const )&_estack,
  [1]=Reset_Handler,
  [15]=SysTick_Handler
};

