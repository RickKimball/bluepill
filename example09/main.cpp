/*---------------------------------------------------------------------
  main.cpp - example09 - using float point and math functions newlib
*/
#include "main.h"
#include <stdlib.h>
#include <math.h>

/*---------------------------------------------------------------------
  Reset_Handler() - the reset exception handler

  Sets stack pointer and inits .bss and .data, optionally "color" memory,
  call global constructors, and finally calls main.
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

#ifdef COLOR_RAM
  /* pattern fill heap ram from _end to _end_heap w/(0xcdcdcdcd) */
  /* optional, useful for gdb to detect which parts of memory are used  */
  __asm__ volatile(
  "color_heap_init:\n"
  " ldr   r0, =end\n"       /* start address of heap in SRAM */
  " ldr   r1, =_end_heap\n" /* end address of available heap SRAM */
  " mov   r2, #0xcdcd\n"    /* pattern constant */
  " movt  r2, #0xcdcd\n"    /* 0xbaadf00d */
  "2:\n"
  " cmp   r0,r1\n"
  " itt   ne\n"         /* exec following 2 instructions if r0 < r1 */
  " strne r2,[r0],#4\n" /* post increment r0 after storing zero */
  " bne.n 2b\n"         /* try again */
  );
#endif

#ifdef COLOR_RAM
  /* pattern fill ram from start of ram to top of stack _estack w/(0xcccccccc) */
  /* optional, useful for gdb to detect which parts of memory are used  */
  __asm__ volatile(
  "color_stack_init:\n"
  " ldr   r0, =_sstack\n"     /* start address of heap in SRAM */
  " ldr   r1, =_estack\n"     /* top of stack in SRAM */
  " mov   r2, #0xcccc\n"      /* pattern constant */
  " movt  r2, #0xcccc\n"      /* 0xcccccccc */
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

  NOTE: called from __libc_init() before main(). It must be a "C" routine.
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
  //  PC13 set to opendrain max 2MHz and set it high
  MODIFY_REG(GPIOC->CRH,(0b1111 << ((13-8)*4)),(0b0110 << ((13-8)*4)));
  GPIOC->BSRR = 1 << 13;

  /*---------------------------------------------------------------------- */
  //  PA9/PA10 configured as USART1
  SET_BIT(RCC->APB2RSTR, RCC_APB2RSTR_USART1RST);
  CLEAR_BIT(RCC->APB2RSTR, RCC_APB2RSTR_USART1RST);
  SET_BIT(RCC->APB2ENR,RCC_APB2ENR_AFIOEN | RCC_APB2ENR_IOPAEN | RCC_APB2ENR_USART1EN);

  GPIOA->CRH = (GPIOA->CRH & ~(0b11111111 << (9-8)*4))
                                    | (0b01001011 << (9-8)*4);  // PA10-RX1, PA9-TX1
  USART1->BRR = ((F_CPU) / BAUD) / APB2_DIV;                    // BUS SPEED is MCLK/1
  USART1->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;     // enable TX, RX, USART

  /*---------------------------------------------------------------------- */
  // enable SWD debugging
  MODIFY_REG(AFIO->MAPR, AFIO_MAPR_SWJ_CFG, AFIO_MAPR_SWJ_CFG_JTAGDISABLE);

  /*---------------------------------------------------------------------- */
  // enable msec Systick ISR
  tickcnt=-500; // test tickcnt wraparound, start @ .5 seconds before rollover
  SysTick_Config(F_CPU/1000U);

#ifdef ENABLE_PROFILE_PIN
  // PA4 OUTPUT_PP, 50MHz toggle pin to profile yield time overhead (0b0011)
  MODIFY_REG(GPIOA->CRL,GPIO_CRL_CNF4,GPIO_CRL_MODE4);
#endif
}

/*----------------------------------------------------------------------
 */
static int print(const char *s) {
  int l = strlen(s);
  return write(1,s,l);
}

/*----------------------------------------------------------------------
 */
static int println(const char *s) {
  int rc=print(s);
  write(1,"\n",1);

  return rc+1;
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"

/*----------------------------------------------------------------------
 print(f,digits) - float, digits after the decimal
 */
static int print(float f, int digits) {
  char buffer[__FLT32_MAX_10_EXP__+2+1];
  int cnt;

  write(1,ftoa(buffer,f,digits,&cnt),cnt);

  return cnt;
}

#pragma GCC diagnostic pop

/*----------------------------------------------------------------------
 print(c,repeat) - print c multiple tiles
 */
static int print(char c, int repeat) {
  for (int i=0; i < repeat; ++i) putchar(c);
  return repeat;
}

/*----------------------------------------------------------------------
  ftoa() - convert float to char * string
    buffer - char array for conversion 
    number - float to be converted
    digits - number of digits after the decimal point
    cnt    - return len of converted string
    bround - round the output
  */
static char *ftoa(char buffer[], float number, int digits, int *cnt, bool bround) {
   // user assumes responsibility for correct buffer size for required digits
  char *p=buffer;
  /* constants */
  const float divf_10 = 1.0f / 10.0f;
  const float mulf_10 = 10.f;

  if ( digits >__FLT32_DECIMAL_DIG__ ) {
    digits = __FLT32_DECIMAL_DIG__;
  }

  // Handle negative numbers
  if (number < 0.0f) {
    *p++ = '-';
    number = -number;
  }

  if ( bround ) {
    // Round correctly so that print(1.999, 2) prints as "2.00"
    float rounding = 0.5f;

    for (int i = 0; i < digits; ++i) {
      rounding *= divf_10; // rounding /= 10.f
    }

    number += rounding;
  }

  // Extract the integer part of the number and print it
  unsigned int_part = (unsigned) number;
  int rc=sprintf(p,"%d",int_part);
  p += rc;

  // Print the decimal point, but only if there are digits beyond
  if (digits > 0) {
    *p++ = '.';
  }

  // Extract digits from the remainder one at a time
  float remainder = number - int_part;
  int remd =0;
  int d=digits;

  while (d-- > 0) {
    remainder *= mulf_10;
    unsigned digit = unsigned(remainder);
    remd *= 10;
    remd += digit;
    remainder = remainder - digit;
  }
  rc=sprintf(p,"%0*d",digits,remd);
  p += rc;

  if (cnt) {
    *cnt = p-buffer;
  }

  return buffer;
}

/*----------------------------------------------------------------------
 */
static float deg2rad(float deg) {
  return (deg * M_PI) / 180.0f;
}

/*---------------------------------------------------------------------
  int main() -
  */
int main(void) {
  static const int sin_tbl_size=(60/2);
  const float angle_incr = (360.0/(float)(sin_tbl_size));

  setvbuf(stdout, NULL, _IONBF, 0);

  int sin_array[sin_tbl_size], x=0;

  println("Sine Wave");
  for (float angle=0.0f; x < sin_tbl_size; angle += angle_incr, ++x)  {
    float f = sinf(deg2rad(angle))*(255.0f/256.0f);
    sin_array[x]=(int)(f*(float)sin_tbl_size);
   
    char b[2][32];
    printf("x=%2d s=%3d, angle=%5s, f=%s\n",
            x, sin_array[x], ftoa(b[0],angle,2), ftoa(b[1],f,4));
  }

  for (x=0; x < sin_tbl_size; ++x) {
      printf("%2d %3d ", x, sin_array[x % sin_tbl_size]);
      print((char)' ',(sin_array[x % sin_tbl_size]+sin_tbl_size));
      println("*");
  }

  return 0;
}

/*---------------------------------------------------------------------
  SysTick_Handler() - SYSTICK exception handler.

    Increment a 1 msec tick counter each time we get an interrupt
    from the SYSTICK.
  */

void SysTick_Handler(void) {
  ++tickcnt;
}

void yield() {

}

/*---------------------------------------------------------------------
  _read() - implement usart input w/yield
  */
extern "C"
int _read(int file, char *ptr, int len) {
  switch(file) {
  case 0:   /* stdin */
    for (int n=0; n < len; ++n) {
      while (!(USART1->SR & USART_SR_RXNE)) {
        yield();
      }

      *(unsigned *)ptr++ = (USART1->DR & 0xff);
    }
    return len;
    // break;

  default:
    errno = EBADF;
    return -1;
  }
}

/*---------------------------------------------------------------------
  _write() - implement usart output w/yield
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
