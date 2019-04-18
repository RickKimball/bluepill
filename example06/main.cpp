/*---------------------------------------------------------------------
  main.cpp - FreeRTOS USART1 example

  Assumes a USB dongle connected to PA9 for 115200,8,n,1
*/

#include <stm32f103xb.h>
#include <cmsis_gcc.h>
#include <common.h>
#include "vectors.h"
#include "FreeRTOS.h"
#include "task.h"
#include "sysclock.h"
#include <stdio.h>
#include <unistd.h>

#define STACK_SIZE 128
#define BAUD 115200

extern "C" void vApplicationStackOverflowHook(xTaskHandle *, signed portCHAR *);
extern "C" void vApplicationIdleHook(void);

static void task1(void *);
static void task2(void *);

static const uint32_t APB2_DIV = 1;
static const uint32_t APB1_DIV = (F_CPU>36000000) ? 2 : 1;

/*---------------------------------------------------------------------
  _init() - initialize board

  NOTE: called from __libc_init() before main().
        It must be a "C" routine.
*/

extern "C" void _init(void) {
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
  RCC->APB2RSTR |= RCC_APB2RSTR_USART1RST_Msk;
  RCC->APB2RSTR &= ~RCC_APB2RSTR_USART1RST_Msk;
  RCC->APB2ENR |= (RCC_APB2ENR_AFIOEN | RCC_APB2ENR_IOPAEN | RCC_APB2ENR_USART1EN);

  GPIOA->CRH = (GPIOA->CRH & ~(0b11111111 << (9-8)*4))
                                     | (0b01001011 << (9-8)*4); // PA10-RX1, PA9-TX1
  USART1->BRR = ((F_CPU) / BAUD) / APB2_DIV;                    // BUS SPEED is MCLK/1
  USART1->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;     // enable TX, RX, USART

  /*---------------------------------------------------------------------- */
  // enable SWD
  MODIFY_REG(AFIO->MAPR, AFIO_MAPR_SWJ_CFG, AFIO_MAPR_SWJ_CFG_JTAGDISABLE);
}

/*---------------------------------------------------------------------
  int main() - what you'd expect
  */

extern "C" int main() {

  xTaskCreate(task1, "LED0", STACK_SIZE, (void *)13, configMAX_PRIORITIES-1, 0);
  xTaskCreate(task2, "UART", STACK_SIZE, (void *)115200, configMAX_PRIORITIES-1, 0);
  vTaskStartScheduler();

  while(1);

  return 0;
}

/*---------------------------------------------------------------------
 task1() - blink PC13
 */
static void task1(void *) {
  while(1) {
    // turn on led - pull to gnd
    GPIOC->BSRR = (1<<13) << 16;
    vTaskDelay(pdMS_TO_TICKS(100));

    // turn off led - let it float
    GPIOC->BSRR = (1<<13);
    vTaskDelay(pdMS_TO_TICKS(400));
  }
}

/*---------------------------------------------------------------------
 task2() - spew uart data on PA9/PA10
 */
static void task2(void * user) {
  int baud = (unsigned)user;
  TickType_t freq=pdMS_TO_TICKS(100);
  TickType_t xLastWakeTime=xTaskGetTickCount();
  static const char msg[]="FreeRTOS usart example BAUD=%d\n";

  iprintf(msg,baud);
  while(1) {
    iprintf("xTaskGetTickCount = %10lu\n", xLastWakeTime);
    vTaskDelayUntil(&xLastWakeTime, freq);
  }
}

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

#ifndef NO_DATA_INIT
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
#endif

#ifndef NO_BSS_INIT
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
#endif

  /* init board, call any global constructors */
  __libc_init_array();
  
  main();

  while(1); /* trap if main exits */
}


void vApplicationStackOverflowHook(xTaskHandle *pxTask,signed portCHAR *pcTaskName) {
  (void)pxTask; (void)pcTaskName;
  while(1);
}

void vApplicationIdleHook(void) {
  __WFE();
}

/* vim: set expandtab ts=2 sw=2: */
