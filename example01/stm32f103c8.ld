ENTRY(_start)

PROVIDE ( _estack = 0x20005000 );

MEMORY
{
   flash(wrx) : ORIGIN = 0x08000000, LENGTH = 64K
   sram(rx)   : ORIGIN = 0x20000000, LENGTH = 20K
}

SECTIONS {
   .isr_vectors : { KEEP(*(.isr_vectors*)) . = ALIGN(4); } > flash
   .text   : { . = ALIGN(4); *(.text*)  } > flash
   .rodata : { . = ALIGN(4); *(.rodata*) } > flash
   .data   : { . = ALIGN(4); *(.*data*) } > sram AT > flash
   .bss    : { . = ALIGN(4); *(.bss*)   } > sram
}

