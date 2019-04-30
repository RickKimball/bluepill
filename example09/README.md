### example09 - using float with newlib printf routines

With this example, we step back from using multitasking and just focus on using newlib nano.specs and POSIX style programming. 

One of the downsides of using nano.specs is that it eliminates floating point printf support in favor of a small size executable.  To use printf and floats you can solve this in 2 ways. You can override the newlib function that converts floats to strings, in the process you add a huge increase in the size of your code. Or take a simpler approach and implement a simple ftoa function to print floats and print it as a string. The code reduction size is significant 42k for the full newlib vs 12k for our ftoa with nano.specs.

This example provides an example of using a ftoa function and calls to the math lib to produce a sin wave printout.  It is a nice example of POSIX style programming used with an MCU.
```
Sine Wave
x= 0 s=  0, angle= 0.00, f=0.0000
x= 1 s=  6, angle=12.00, f=0.2071
x= 2 s= 12, angle=24.00, f=0.4051
x= 3 s= 17, angle=36.00, f=0.5855
x= 4 s= 22, angle=48.00, f=0.7402
x= 5 s= 25, angle=60.00, f=0.8626
x= 6 s= 28, angle=72.00, f=0.9473
x= 7 s= 29, angle=84.00, f=0.9906
x= 8 s= 29, angle=96.00, f=0.9906
x= 9 s= 28, angle=108.00, f=0.9473
x=10 s= 25, angle=120.00, f=0.8626
x=11 s= 22, angle=132.00, f=0.7402
x=12 s= 17, angle=144.00, f=0.5855
x=13 s= 12, angle=156.00, f=0.4051
x=14 s=  6, angle=168.00, f=0.2071
x=15 s=  0, angle=180.00, f=-0.0000
x=16 s= -6, angle=192.00, f=-0.2071
x=17 s=-12, angle=204.00, f=-0.4051
x=18 s=-17, angle=216.00, f=-0.5855
x=19 s=-22, angle=228.00, f=-0.7402
x=20 s=-25, angle=240.00, f=-0.8626
x=21 s=-28, angle=252.00, f=-0.9473
x=22 s=-29, angle=264.00, f=-0.9906
x=23 s=-29, angle=276.00, f=-0.9906
x=24 s=-28, angle=288.00, f=-0.9473
x=25 s=-25, angle=300.00, f=-0.8626
x=26 s=-22, angle=312.00, f=-0.7402
x=27 s=-17, angle=324.00, f=-0.5855
x=28 s=-12, angle=336.00, f=-0.4051
x=29 s= -6, angle=348.00, f=-0.2071
 0   0                               *
 1   6                                     *
 2  12                                           *
 3  17                                                *
 4  22                                                     *
 5  25                                                        *
 6  28                                                           *
 7  29                                                            *
 8  29                                                            *
 9  28                                                           *
10  25                                                        *
11  22                                                     *
12  17                                                *
13  12                                           *
14   6                                     *
15   0                               *
16  -6                         *
17 -12                   *
18 -17              *
19 -22         *
20 -25      *
21 -28   *
22 -29  *
23 -29  *
24 -28   *
25 -25      *
26 -22         *
27 -17              *
28 -12                   *
29  -6                         *

```
The application code uses register access only, no HAL, LL, no libopencm3. The structures and defines in the stm32f103xb.h CMSIS device header are used to configure and manipulate the peripherals and clocks.  This code does use the nano.specs newlib libc library.  This adds significant code overhead but it shows how you can treat an mcu with a more traditional POSIX style api for reading, writing and printing.

This verison of the makefile can handle multiple files. They can be c, c++, .s and .S files.  It links using g++. For this example the top level file is main.cpp.

Note: This configuration makes a lot of assumptions, mostly that you are running linux or OSX. It expects that arm-none-eabi-gcc and openocd are installed and in your $PATH.

Note: Make sure you checkout the submodule in 3rd_party. The easiest way is to clone recursively:
```
$ git clone --recursive https://github.com/RickKimball/bluepill
```

### Features:

* flexible Makefile automatically compiles and links, with autodependencies
* main in 'C++'
* vector table in 'C'
* STM32CubeMX stm32f103c8.ld with c++ support
* SysTick interrupt handler configured to create a msec timer tick.
* uses newlib for malloc()/free(), printf routines
* overrides _read() and _write() to implement console input/output
* using printf and command line processing has grown our code, still fairly small:
```
$ make
TARGET=firmware finished
   text	   data	    bss	    dec	    hex	filename
  12528	    100	     20	  12648	   3168	build/src/firmware.elf
```
### Source Files
This example compiles and links files 

  * **main.cpp** - main, Reset_Handler, SysTick_Handler
  * **sysclock.h** - routines for setting SYSCLK called from Reset_Handler
  * **vector.c** - contains the vector_table (VTOR) in 'C'

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
