/*---------------------------------------------------------------------
  vectors.c - vector table (VTOR) and exception/isr handler code
*/
#include <stm32f103xb.h>
#include <common.h>
#include "vectors.h"

__attribute__((naked))
void NMI_Handler() {
  while(1) {
  __NOP();
  }
}

__attribute__((naked))
void HardFault_Handler() {
  while(1) {
  __NOP();
  }
}

__attribute__((naked))
void MemManage_Handler() {
  while(1) {
  __NOP();
  }
}

__attribute__((naked))
void BusFault_Handler() {
  while(1) {
  __NOP();
  }
}

__attribute__((naked))
void UsageFault_Handler() {
  while(1) {
  __NOP();
  }
}

__attribute__((naked))
void SVCall_Handler() {
  while(1) {
  __NOP();
  }
}

__attribute__((naked))
void DebugMonitor_Handler() {
  while(1) {
  __NOP();
  }
}

__attribute__((naked))
void PendSV_Handler() {
  while(1) {
  __NOP();
  }
}

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
  [ 0]=(void * const )&_estack,
  [ 1]=Reset_Handler,
  [ 2]=NMI_Handler,
  [ 3]=HardFault_Handler,
  [ 4]=MemManage_Handler,
  [ 5]=BusFault_Handler,
  [ 6]=UsageFault_Handler,
  [11]=SVCall_Handler,
  [12]=DebugMonitor_Handler,
  [14]=PendSV_Handler,
  [15]=SysTick_Handler,
#if VECT_ADDR != 0x08000000
  [66]=(void * const)BootRAM /* inline instruction used when booting from RAM */
#endif
};

/* vim: set expandtab ts=2 sw=2: */
