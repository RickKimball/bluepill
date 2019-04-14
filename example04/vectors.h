/*---------------------------------------------------------------------
 * vector.h ISR handlers
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* addresses from stm32f103c8.ld */
unsigned _estack;   
extern unsigned __bss_start__, __bss_end__;
extern unsigned _sidata, _sdata, _edata;

/* make sure these all have C signatures */
void __libc_init_array(void);
__attribute__((used, naked)) void Reset_Handler(void);
void SysTick_Handler(void);

#ifdef __cplusplus
}
#endif

/* vim: set ts=2 sw=2 expandtab: */
