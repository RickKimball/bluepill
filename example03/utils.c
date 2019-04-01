/*
 vim: set expandtab ts=2 sw=2:

 utils.c - utility routines 

 */

void delay(unsigned a) {
  volatile unsigned b=0;

  while(b++ < a);

  return;
} 
