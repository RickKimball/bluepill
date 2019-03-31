// minimal c code using stm32f103xb.h device header only 
// no stdlib, no gcc startup code
// self contained VTOR table

#include <stm32f103xb.h>

typedef void (* const vector)(void);

void delay(unsigned a);
__attribute__((used, naked)) void _start(void);

extern unsigned _estack;

__attribute((used, section(".isr_vectors")))
const vector isr_handlers[1+14] = 
{
  (void *)&_estack, _start
};

int main(){
  RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;

  // make PC13 opendrain
  GPIOC->CRH = (GPIOC->CRH & ~(0b1111 << 20)) | 0b1 << 20;

  while(1){
    GPIOC->ODR &= ~(1<<13); // let it float
    delay(50000);
    GPIOC->ODR |= 1<<13;    // pull to ground
    delay(300000);
  }
  return 0;
}

void delay(unsigned a){
  volatile unsigned b=0;

  while(b++ < a);

  return;
} 

//__attribute((naked,used))
void _start() {
  // set stack pointer to top of stack
  asm volatile ("ldr r1,=0x20005000");
  asm volatile ("mov sp,r1");

  /* you should really:
     memset bss to zeros,
     memcpy flash data to ram data
     call global constructors * even c has them
     then call main
  */
    
  main();

  while(1); // trap if main exits
}

