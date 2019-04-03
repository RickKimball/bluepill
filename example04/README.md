### example04 - blinky using multifiles

This verison of the makefile can handle multiple files. They can be c, c++, .s and .S assumber files.  I links using g++. For this example the top level file is main.cpp

Features:

* Make
* main in 'C++'
* vector table in 'C'
* STM32CubeMX stm32f103c8.ld
* SysTick interrupt handler 1 msecond tickcnt
* PC13 led blink

This example compiles and links 3 files 

  * **main.cpp** - loop and main
  * **utils.c** - contains the delay() routine that isn't actually used
  * **vector.c** - contains the vector table (VTOR) and exception handlers in 'C'

To build the .elf file:

  *  $ make all

To build .elf and generate .lss (mixed c/asm source), .hex (intel hex file), .bin (binary load at 0x08000000), and .sym (text listing of external symbols sorted by address):

 * $ make ALL

To upload using openocd and stlink:

  * $ make install

To clean up:

 * $ make clean

To debug with arm-none-eabi-gdb/openocd and stlink:

 * $ make debug # starts openocd and an xterm console gui gdb session

To get compiler verbose output:

* $ make V=1 clean
* $ make VERBOSE=1 clean
* $ make V=1 ALL


