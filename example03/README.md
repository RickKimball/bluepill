
### example03 - PC13 blinky light app using SysTick

This example compiles and links 3 files 

  * **vector.c ** - contains the exception handlers and vector table
  * **utils.c** - contains the delay() routine that isn't actually used
  * **example03.c** - contains the main() 

To build .elf file:

  *  $ make all

To build .elf and generate .lss (mixed c/asm source), .hex (intel hex file), .bin (binary load at 0x08000000), and .sym (text listing of external symbols sorted by address):

 * $ make ALL

To upload using openocd and stlink:

  * $ make upload

To clean up:

 * $ make clean

To debug with arm-none-eabi-gdb/openocd and stlink:

 * $ make debug # this actually uploads then starts a xterm console gui gdb session
