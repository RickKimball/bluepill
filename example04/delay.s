/*----------------------------------------------------------------------
 void delay(unsigned msec) - cycle count delay for 8MHz clock
 */
        .syntax unified
	.thumb
	.cpu cortex-m3
 
        .text
	.global delay
        .type delay,%function
delay:
	mov     r12,#1600 		@ F_CPU/5  (1/1000)/(1/8000000)
	mul     r0, r12 
0:
        adds    r0, r0, #-1
        bcc     1f
        b       0b
1:
        bx      lr

