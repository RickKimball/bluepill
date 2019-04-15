/*---------------------------------------------------------------------
  main.cpp - FreeRTOS two task blinky 

  Assumes an external LED connected to PC14 in addtion to builtin led
*/

#include <stm32f103xb.h>
#include <cmsis_gcc.h>
#include <common.h>
#include "vectors.h"
#include "FreeRTOS.h"
#include "task.h"

#define STACK_SIZE 128

extern "C" void vApplicationStackOverflowHook(xTaskHandle *, signed portCHAR *);
static void task1(void *);
static void task2(void *);

/*---------------------------------------------------------------------
  _init() - initialize board

  NOTE: called from __libc_init(). It must be a "C" routine.
*/

extern "C" void _init(void) {
  // turn on GPIOC clock
  RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;

  // PC13 and PC14 set to opendrain / 2MHz max
  GPIOC->CRH = (GPIOC->CRH & ~(0b1111<<20 | 0b1111<<24)) |
               (0b0110<<20 | 0b0110<<24);
}

/*---------------------------------------------------------------------
  int main() - what you'd expect
  */

extern "C" int main() {
  xTaskCreate(task1, "LED0", STACK_SIZE, (void *)13, configMAX_PRIORITIES-1, 0);
  xTaskCreate(task2, "LED1", STACK_SIZE, (void *)14, configMAX_PRIORITIES-1, 0);
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
 task2() - blink PC14
 */
static void task2(void *) {
  while(1) {
    // turn on led - pull to gnd
    GPIOC->BSRR = (1<<14) << 16;
    vTaskDelay(pdMS_TO_TICKS(100));

    // turn off led - let it float
    GPIOC->BSRR = (1<<14);
    vTaskDelay(pdMS_TO_TICKS(1400));
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

  __libc_init_array();
  
  main();

  while(1); /* trap if main exits */
}


void
vApplicationStackOverflowHook(xTaskHandle *pxTask,signed portCHAR *pcTaskName) {
        (void)pxTask;
        (void)pcTaskName;
        while(1);
}

/* vim: set expandtab ts=2 sw=2: */
