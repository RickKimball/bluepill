/*----------------------------------------------------------------------
 utils.h
 */

#pragma once

extern volatile unsigned tickcnt;

/* systick delay_ms */
void delay_ms(unsigned msecs) {
  const unsigned t0=tickcnt;

  while( (tickcnt-t0) < msecs) { __WFE(); }

  return;
} 

/* cycle count delay from delay.S */
#ifdef __cplusplus
extern "C" void delay(unsigned msec);
#else
extern void delay(unsigned msec);
#endif

/* vim: set ts=2 sw=2 expandtab: */
