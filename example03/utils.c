// vim: set expandtab ts=2 sw=2:

void delay(unsigned a) {
  volatile unsigned b=0;

  while(b++ < a);

  return;
} 
