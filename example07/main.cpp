/*---------------------------------------------------------------------
  main.cpp - example07 cooperative multi-tasking USART1/Blink sample

  Assumes a USB dongle connected to PA9 for 115200,8,n,1
*/
#include <stm32f103xb.h>
#include <cmsis_gcc.h>
#include <stdio.h>
#include <unistd.h>
#include <common.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/times.h>
#include <sys/unistd.h>
#include <inttypes.h>
#include <string.h>
#include <Scheduler.h>
#include "vectors.h"
#include "sysclock.h"

#define BAUD 115200

/*---------------------------------------------------------------------
 function signatures
 */
extern "C" void _init(void);
extern "C" int main(void);
static void delay_until(unsigned & tick_start, const unsigned msecs);
static void blink_task();
static void loop();

/*---------------------------------------------------------------------
  globals
  */
static const uint32_t APB2_DIV = 1;
static const uint32_t APB1_DIV = (F_CPU>36000000) ? 2 : 1;

volatile unsigned tickcnt;            // SysTick_Handler msec counter

/*---------------------------------------------------------------------
 Reset_Handler() - the reset exception handler

 Sets stack pointer and optionally inits .bss and .data
 then calls main.
 */

void Reset_Handler(void) {
  /* set stack pointer to top of stack memory */
  asm volatile ("ldr r0, =%0" :: "i" (&_estack));
  asm volatile ("mov sp, r0");

  /* setup SYSCLK early */
  sysclock_set<F_CPU>();

#if VECT_ADDR > 0x08000000
  SCB->VTOR = VECT_ADDR;
#endif

  /* Copy the .data segment initializers from flash to SRAM */
  __asm__ volatile(
  "data_init:\n"
  " ldr   r0,=_sidata\n" /* start address of initialized data in flash */
  " ldr   r1,=_sdata\n" /* start address of data in SRAM */
  " ldr   r2,=_edata\n" /* end address of data in SRAM */
  "1:\n"
  " cmp   r1,r2\n"
  " ittt  lo\n"         /* exec following 3 instructions if r1 < r2 */
  " ldrlo r3,[r0],#4\n" /* post increment r0 after ldr */
  " strlo r3,[r1],#4\n" /* post increment r1 after str */
  " blo.n 1b\n"         /* try again */
  );

  /* Zero fill the .bss segment, assumes aligned on 4 bytes */
  __asm__ volatile(
  "bss_init:\n"
  " ldr   r0, =_sbss\n" /* start address of non initialized data in SRAM */
  " ldr   r1, =_ebss\n" /* end address of non initialized data in SRAM */
  " movs  r2, #0\n"     /* word sized zero constant */
  "2:\n"
  " cmp   r0,r1\n"
  " itt   ne\n"         /* exec following 2 instructions if r0 < r1 */
  " strne r2,[r0],#4\n" /* post increment r0 after storing zero */
  " bne.n 2b\n"         /* try again */
  );

#if 0
  /* pattern fill ram from _ebss to top of stack _estack w/(0xbaadf00d) */
  /* optional, useful for gdb to detect which parts of memory are used  */
  __asm__ volatile(
  "color_ram_init:\n"
  " ldr   r0, =end\n"     /* start address of heap in SRAM */
  " ldr   r1, =_estack\n" /* top of stack in SRAM */
  " mov   r2, #0xf00d\n"  /* pattern constant */
  " movt  r2, #0xbaad\n"  /* 0xbaadf00d */
  "2:\n"
  " cmp   r0,r1\n"
  " itt   ne\n"         /* exec following 2 instructions if r0 < r1 */
  " strne r2,[r0],#4\n" /* post increment r0 after storing zero */
  " bne.n 2b\n"         /* try again */
  );
#endif

  /* _init() board, then call any global c/c++ constructors from newlib */
  __libc_init_array();
  
  main();

  while(1); /* trap if main exits */
}

/*---------------------------------------------------------------------
  _init() - initialize board

  NOTE: called from __libc_init() before main().
        It must be a "C" routine.
*/
void _init(void) {
  RCC->APB2ENR |= 0                                       /* turn on clocks */
               | RCC_APB2ENR_AFIOEN                       /* Alternate IO   */
               | RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPCEN  /* GPIO A/C       */
               | RCC_APB2ENR_USART1EN                     /* USART1         */
               ;

  /*---------------------------------------------------------------------- */
  // PA8 configured as Master Clock Out (0b10 ALT FUNC PP | 0b11 50MHz max output)
  GPIOA->CRH = (GPIOC->CRH & ~(0b1111 << ((8-8)*4))) | (0b1011 << ((8-8)*4));
  MODIFY_REG(RCC->CFGR, RCC_CFGR_MCOSEL,
    ((F_CPU > 36000000) ? RCC_CFGR_MCOSEL_PLL_DIV2 : RCC_CFGR_MCOSEL_SYSCLK));

  /*---------------------------------------------------------------------- */
  //  PC13 set to opendrain max 2MHz
  MODIFY_REG(GPIOC->CRH,(0b1111 << ((13-8)*4)),(0b0110 << ((13-8)*4)));

  /*---------------------------------------------------------------------- */
  //  PA9/PA10 configured as USART1
  SET_BIT(RCC->APB2RSTR, RCC_APB2RSTR_USART1RST);
  CLEAR_BIT(RCC->APB2RSTR, RCC_APB2RSTR_USART1RST);
  SET_BIT(RCC->APB2ENR,RCC_APB2ENR_AFIOEN | RCC_APB2ENR_IOPAEN | RCC_APB2ENR_USART1EN);

  GPIOA->CRH = (GPIOA->CRH & ~(0b11111111 << (9-8)*4))
                                     | (0b01001011 << (9-8)*4); // PA10-RX1, PA9-TX1
  USART1->BRR = ((F_CPU) / BAUD) / APB2_DIV;                    // BUS SPEED is MCLK/1
  USART1->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;     // enable TX, RX, USART

  /*---------------------------------------------------------------------- */
  // enable SWD
  MODIFY_REG(AFIO->MAPR, AFIO_MAPR_SWJ_CFG, AFIO_MAPR_SWJ_CFG_JTAGDISABLE);

  /*---------------------------------------------------------------------- */
  // enable msec Systick ISR
  tickcnt=-500; // test tickcnt wraparound, start .5 seconds before rollover
  SysTick_Config(F_CPU/1000U);

#ifdef ENABLE_PROFILE_PIN
  // PA4 OUTPUT_PP, 50MHz toggle pin to profile yield time overhead (0b0011)
  MODIFY_REG(GPIOA->CRL,GPIO_CRL_CNF4,GPIO_CRL_MODE4);
#endif
}

/*---------------------------------------------------------------------
  int main() - 
  */
int main(void) {

  Scheduler.startLoop(blink_task);

  loop();

  return 0;
}

/*---------------------------------------------------------------------
 blink_task() - toggle LED_BUILTIN on/off
 */
void blink_task() {
  static unsigned start_tick=tickcnt;

  while(1) {
    // turn on led - pull to gnd
    GPIOC->BSRR = (1<<13) << 16;
    delay_until(start_tick, 50);

    // turn off led - let it float
    GPIOC->BSRR = (1<<13);
    delay_until(start_tick, 450);
  }
}

/*---------------------------------------------------------------------
 loop() - main tasks loop, spew tickcnt
 */
void loop() {
  static const unsigned print_delay_ms=500;
  static unsigned start_tick=tickcnt;

  while(1) {
#if 0 /* minimal code size to illustrate multi tasking */
    write(1,".",1);
#else
    iprintf("tickcnt=%d\n",start_tick);
#endif
    delay_until(start_tick, print_delay_ms); // wait, then update start_tick
  }
}

/*---------------------------------------------------------------------
 delay_until(tick_start, msec) - delay for milliseconds

 once delay is met, tick_start is updated for the caller.
 */
void delay_until(unsigned & tick_start, const unsigned msecs) {
  while( (tickcnt-tick_start) < msecs) {
    yield();
  }
  tick_start += msecs;
  return;
} 

/*---------------------------------------------------------------------
  SysTick_Handler() - SYSTICK exception handler.

  Increment a 1 msec tick counter each time we get an interrupt
  from the SYSTICK.
  */

void SysTick_Handler(void) {
  ++tickcnt;
}

/*---------------------------------------------------------------------
 * _write() - implement usart output
 */
extern "C"
int _write(int file, char *ptr, int len) {
  switch(file) {
  case 1:   /* stdout */
  case 2:   /* stderr */
    for (int n=0; n < len; ++n) {
      while (!(USART1->SR & USART_SR_TXE)) {
        yield();
      }
      USART1->DR = (*ptr++ & (uint16_t)0xFF);
    }
    return len;
  default:
    errno = EBADF;
    return -1;
  }
}

/* vim: set expandtab ts=2 sw=2: */
