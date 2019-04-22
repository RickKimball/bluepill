### example08 - blinky & serial output using Scheduler from Arduino

TDB: add more features to this.  Talk about how this is using a new and simplified Makefile.
TBD: Talk about the armv7m-vectstate.gdb macro feature
TBD: Talk about compiling with the latest arm-eabi-gcc version 8.3.0
TBD: Talk about and code a UART ISR handler for input.

... this is all leftover from example07 but still applicable ...
I know my mantra all along has been no external class frameworks. I'm going to modify my rule this one time.

Arduino provides a cooperative multitasking Scheduler library that is actually very portable. It only works with cortex-m3 and cortex-m0 chips. Thankfully, the bluepill is a cortex-m3.  The Scheduler class code includes the Ardunio.h header, although it doesn't actually use any Arduino code or structures. We provide a fake Arduino.h that includes 'inttypes.h' and 'malloc.h'. The Scheduler class provides a very small cooperative multi tasking configuration perfect for our minimal code size philosophy.

This example creates a minimal configuration to run two tasks; one blinking the PC13 led and a second using USART1 to output the millisecond tickcnt. Much like the example06 that used FreeRTOS, each task runs until it yields.  We use newlib to provide implemenatations of malloc() and iprintf().  In reality, task creation only calls malloc one time and never releases the memory for the task.  There is one rule; task loops must yield periodically or other tasks won't run. This is accomplished by calling delay() or yield().  The code in main.cpp illustrates this technique. 

The application code uses register access only, no HAL, LL, no libopencm3. The structures and defines in the stm32f103xb.h CMSIS device header are used to configure and manipulate the peripherals and clocks.  As previously stated, this code does use the Arduino Scheduler. Look at the code, it really has no Arduino dependecies. I consider it safe to use and it adds little overhead to our setup but brings great flexibility.

This verison of the makefile can handle multiple files. They can be c, c++, .s and .S files.  It links using g++. For this example the top level file is main.cpp.

Note: This configuration makes a lot of assumptions, mostly that you are running linux or OSX. It expects that arm-none-eabi-gcc and openocd are installed and in your $PATH.

Note: The Arduino Scheduler class has its own separate license see: ../3rd_party/Scheduler code for details.

Note: Make sure you checkout the submodule in 3rd_party. The easiest way is to clone recursively:
```
$ git clone --recursive https://github.com/RickKimball/bluepill
```

### Features:

* Arduino cooperative task scheduler added as a github submodule. This is much smaller and less complex than FreeRTOS. It only handles the task switching, no helper atomic classes.
* flexible Makefile automatically compiles and links, with autodependencies
* main in 'C++'
* vector table in 'C'
* STM32CubeMX stm32f103c8.ld with c++ support
* SysTickh interrupt handler configured to create a msec timer tick.
* PC13 led blink
* uses newlib for malloc()/free() routines
* overrides _write() to implement console output
* using a minimal print, just a '.', results in multi tasking code that is small:
```
$ make
   text    data     bss     dec     hex filename
   1484     104      44    1632     660 main.elf
```
* using iprintf grows our code by about 3k, still much smaller than using FreeRTOS:
```
$ make
   text    data     bss     dec     hex filename
   4864     104      44    5012    1394 main.elf
```
### Source Files
This example compiles and links files 

  * **main.cpp** - main, blink_task, loop, delay, Reset_Handler, SysTick_Handler
  * **sysclock.h** - routines for setting SYSCLK called from Reset_Handler
  * **vector.c** - contains the vector_table (VTOR) in 'C'
  * **Scheduler/Scheduler.cpp** - contains Arduino cooperative multi tasker code

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
