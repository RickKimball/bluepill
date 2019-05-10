#include "stm32f030x8.h"

typedef void (* const vector)(void);

extern unsigned _estack; // this comes from stm32f030r8.ld
__attribute__((used, naked)) void setup(void);

static void loop();

__attribute((section(".isr_vectors")))
const vector vector_table[] = 
{
  (void *)&_estack, setup
#if 0
  ,[15]=loop
#endif
};

void setup(void) {
  RCC->AHBENR  = 0x00000014 | RCC_AHBENR_GPIOAEN;
  GPIOA->MODER = 0x28000000 | 0b01 << 10;

#if 0
  SysTick->LOAD = (8000000/2)-1;
  SysTick->VAL   = 0UL;                                             /* Load the SysTick Counter Value */
  SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk |
                   SysTick_CTRL_TICKINT_Msk   |
                   SysTick_CTRL_ENABLE_Msk;                         /* Enable SysTick IRQ and SysTick Timer */
#else
  loop();
#endif

  while(1);
}

#if 0 
void loop() {
  GPIOA->ODR ^= 1 << 5;
}
#else
void loop() {
  unsigned x;
  unsigned mask=0;

  while(1) {
    GPIOA->ODR = mask;
    mask ^= 1 << 5;
    
    x = 1 << 18;
    do {
      __NOP();
    } while(--x);
  }
}
#endif

// vim: set expandtab ts=2 sw=2:
