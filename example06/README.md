### example06 - blinky & serial output using FreeRTOS

A minimal FreeRTOS configuration to run two tasks one blinking PC13 led and a second using USART1 to output the current TaskTickCount. This is setup to use cooperative-multitasking to make the code smaller. Each task runs until it yields.  We use newlib and the heap_3.c memory manager to allow the use of malloc and iprintf().

The application code uses register access only, no HAL, LL, no libopencm3, no Arduino. The structures and defines in the stm32f103xb.h CMSIS device header are used to configure and manipulate the peripherals and clocks.

This verison of the makefile can handle multiple files. They can be c, c++, .s and .S files.  It links using g++. For this example the top level file is main.cpp.

Note: This configuration makes a lot of assumptions, mostly that you are running linux or OSX. It expects that arm-none-eabi-gcc and openocd are installed and in your $PATH.

Note: FreeRTOS has its own separate license see: FreeRTOS/License/license.txt

### Features:

* FreeRTOS 10.2.0 configuration 
* flexible Makefile automatically compiles and links, with autodependencies
* main in 'C++'
* vector table in 'C'
* STM32CubeMX stm32f103c8.ld with c++ support
* interrupt handler configured for FreeRTOS
* PC13/PC14 led blink
* uses newlib for malloc()/free() routines
* overrides _write() to implement console output
* using iprintf grows our code by about 3k, still fairly small size:
```
   generating main.elf
   text	   data	    bss	    dec	    hex	filename
   6344	    104	    252	   6700	   1a2c	main.elf

```
### Source Files
This example compiles and links files 

  * **main.cpp** - main, task1, task2
  * **os.c** - implements _write() that sends output to PA9.
  * **vector.c** - contains the vector table (VTOR) and exception handlers in 'C'
  * **FreeRTOS/\*.c** - contains FreeRTOS configured to use newlib malloc()/free()

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

You can invoke make with the V=1 enviroment variable for verbose command output:

`$ make V=1 clean ALL install
`

See the makefile for other things you can easily override.
