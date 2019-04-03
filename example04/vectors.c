/*---------------------------------------------------------------------
  vectors.c - vector table (VTOR) and exception/isr handler code
*/
#include <stm32f103xb.h>
#include <common.h>

extern int main();
extern void __libc_init_array(void);


/*---------------------------------------------------------------------
 * ISR handlers
 */
__attribute__((used, naked))
void Reset_Handler(void);
void SysTick_Handler(void);

/*---------------------------------------------------------------------
 Reset_Handler() - the reset exception handler

 Sets stack pointer and optionally inits .bss and .data
 then calls main.
 */

unsigned _estack;   /* this comes from stm32f103c8.ld */

void Reset_Handler(void) {
  /* set stack pointer to top of stack memory */
  asm volatile ("ldr r0, =%0" :: "i" (&_estack));
  asm volatile ("mov sp, r0");

#if VECT_ADDR > 0x08000000
  SCB->VTOR = VECT_ADDR;
#endif

#ifndef NO_BSS_INIT
  /* bss zero */
  {
  unsigned *dst;
  extern unsigned __bss_start__, __bss_end__;

  for(dst=&__bss_start__; dst < &__bss_end__;) {
    *dst++ = 0;
  }
  }
#endif

#ifndef NO_DATA_INIT
  /* data copy */
  {
  unsigned *dst;
  extern unsigned _sidata, _sdata, _edata;
  unsigned *src;

  for(dst=&_sdata,src=&_sidata; dst < &_edata; ) {
    *dst++ = *src++;
  }
  }
#endif

  __libc_init_array();
  
  main();

  while(1); /* trap if main exits */
}

/*---------------------------------------------------------------------
  SysTick_Handler() - SYSTICK exception handler.

  Increment a tick counter each time we get an interrupt
  from the SYSTICK.
  */

void SysTick_Handler(void) {
  extern volatile unsigned tickcnt;

  ++tickcnt;
}

typedef void (* const vector)(void); /* const pointer to function(void) return void */
#define BootRAM 0xF108F85F

/*---------------------------------------------------------------------
  VTOR vector table - must begin at 0x08000000 || 0x20000000
  
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

#if VECT_ADDR == 0x08000000
#define VTOR_MIN (1+15)
#else
#define VTOR_MIN (1+15+51)
#endif

__attribute((used, section(".isr_vector")))
static const vector vector_table [ VTOR_MIN ] = 
{
  (void * const )&_estack,
  [ 1]=Reset_Handler,
  [15]=SysTick_Handler,
#if VECT_ADDR != 0x08000000
  [66]=(void * const)BootRAM /* inline instruction used when booting from RAM */
#endif
};

/* vim: set expandtab ts=2 sw=2: */

