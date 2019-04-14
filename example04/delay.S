/*----------------------------------------------------------------------
 void delay(unsigned msecs) - cycle count delay for 8MHz clock
 */

#define _LOOP_CNT_PER_MSEC F_CPU/1000/4         @ each loop is 4 instruction cycles

        .cpu    cortex-m3
        .syntax unified
        .thumb

        .text
        .global delay
        .type   delay,%function
delay:
        .align  4
        mov     r12,#_LOOP_CNT_PER_MSEC
        mul     r0,r12                          @ multiply msecs * (cycles per msec/4)
0:
        adds    r0,r0,#-1                       @ 4 cycle count down loop
        bcc.n   1f
        b.n     0b
1:
        bx      lr

/* vim: set ts=8 sw=8 expandtab: */
