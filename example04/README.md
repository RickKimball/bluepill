### example04 - blinky using multifiles

This verison of the makefile can handle multiple files. They can be c, c++, .s and .S files.  It links using g++. For this example the top level file is main.cpp.

Note: This configuration makes a lot of assumptions, mostly that you are running linux or OSX. It expects that arm-none-eabi-gcc and openocd are installed and in your $PATH.

### Features:

* flexible Makefile automatically compiles and links, with autodependencies
* main in 'C++'
* vector table in 'C'
* STM32CubeMX stm32f103c8.ld with c++ support
* SysTick interrupt handler 1 msecond tickcnt
* sample assembler code for cycle count delay
* PC13 led blink

### Source Files
This example compiles and links 3 files 

  * **main.cpp** - loop and main
  * **delay.s** - contains cycle count delay() assumes 8MHz HSI
  * **vector.c** - contains the vector table (VTOR) and exception handlers in 'C'

### make target goals:

* **all** - builds all the files and produces a main.elf and main.hex
* **ALL** - builds all the files and produces a main.elf, main.hex, main.bin and all listing files
* **clean** - deletes all generated files
* **debug** - launches openocd and an xterm running arm-none-eabi-gdb with main.elf
* **erase** - uses openocd to do a mass erase
* **install** - uses openocd to load main.elf or main.hex onto flash or ram
* **kill** - sometimes openocd gets stuck, uses killall to force it to exit
* **openocd** - launches openocd and connects via an xterm running telnet interface
* **ram** - build and run the code all from RAM, assumes you move the boot0 and boot1 jumpers to '1'
* **size** - shows the size of main.elf

You can also invoke make with the V=1 enviroment variable to see the commands and command arguments

`$ make V=1 clean ALL install
`

See the makefile for other things you can easily override.