// vim: set expandtab ts=2 sw=2:

// start.c - the reset exception handler

#include <stm32f103xb.h>

typedef void (* const vector)(void);

extern unsigned _estack; // this comes from stm32f103c8.ld
extern int main();

volatile unsigned tickcnt;

/*
 _init() - normally we would setup the MCLK

*/
void _init() {
  tickcnt=0; // as we aren't zeroing bss we do this

  SysTick_Config(F_CPU/1000);
}

__attribute__((used, naked))
void Reset_Handler(void);

/*
 Reset_Handler() - the reset exception handler
 sets stack and optionally inits .bss and .data
 then calls main
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

 */

void SysTick_Handler(void) {
  ++tickcnt;
}

/*
 excpt_handler/vector table
 */

__attribute((used, section(".isr_vectors")))
static const vector excp_handlers[1+15] = 
{
  (void *)&_estack,
  [1]=Reset_Handler,
  [15]=SysTick_Handler
};

