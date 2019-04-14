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
extern void __libc_init_array(void);
extern void Reset_Handler(void) __attribute__((used, naked));
extern void vPortSVCHandler( void ) __attribute__ (( naked ));
extern void xPortPendSVHandler( void ) __attribute__ (( naked ));
extern void xPortSysTickHandler( void );


#ifdef __cplusplus
}
#endif

/* vim: set ts=2 sw=2 expandtab: */
