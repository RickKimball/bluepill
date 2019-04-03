#-----------------------------------------------------------------------
# Makefile - somewhat simple makefile for the bluepill running on HSI
#
# The idea of this makefile is to use as many defaults from
# the make default rules as we can. It knows what to do with
# CC, CFLAGS, LDFLAGS etc... so use them by just populating
# them and letting the default transforms happen. We just
# need to fill in the values.
#
# NOTE: Executed commands are silent by default. To see verbose
#        command output and its args run:
#
#  $ make V=1 all
#

#--- override from the commandline make ---
COM_EXTRA    ?= -UNO_BSS_INIT -UNO_DATA_INIT
CPUFLAGS     ?= -mcpu=cortex-m3 -mthumb
DEBUGF       ?= -g
F_CPU        ?= 8000000
INCLUDES     ?= -I ../cmsis -I ../include
LDFILE       ?= stm32f103c8.ld
LDSCRIPT     ?= -T $(LDFILE)
LINKER_GCC   ?= arm-none-eabi-g++
OPTIMIZE     ?= -Os
STDCC        ?= gnu11
STDCXX       ?= gnu++14
TARGET       ?= main
VECT_ADDR    ?= 0x08000000

#--- assumes the compiler can be found in $PATH  ----------------------
AS=arm-none-eabi-gcc
CC=arm-none-eabi-gcc
CXX=arm-none-eabi-g++
OBJCOPY=arm-none-eabi-objcopy
OBJDUMP=arm-none-eabi-objdump
OBJNM=arm-none-eabi-nm
OBJREADELF=arm-none-eabi-readelf
OBJSIZE=arm-none-eabi-size
LINK.cc=$(CXX)

#--- gather the files to compile --------------------------------------
SRCS  = $(filter %.c %.cpp %.s %.S, $(wildcard *.*))
OBJ0 = $(SRCS:.c=.o)
OBJ1 = $(SRCS:.cpp=.o)
OBJ2 = $(SRCS:.s=.o)
OBJ3 = $(SRCS:.S=.o)
OBJX = $(OBJ0) $(OBJ1) $(OBJ2) $(OBJ3)
OBJS  = $(filter %.o, $(OBJX))
DEPS  = $(OBJS:.o=.d)

BIN := $(TARGET).bin
ELF := $(TARGET).elf
HEX := $(TARGET).hex
LSS := $(TARGET).lss
MAP := $(TARGET).map
SYM := $(TARGET).sym

ASFLAGS = -c -Wall -g $(CPUFLAGS) $(INCLUDES) -DF_CPU=$(F_CPU) -x assembler-with-cpp

COM_FLAGS += -Wall $(DEBUGF) $(OPTIMIZE) $(CPUFLAGS)
COM_FLAGS += -DF_CPU=$(F_CPU) -DVECT_ADDR=$(VECT_ADDR)
COM_FLAGS += -ffunction-sections -fwrapv
COM_FLAGS += -fsingle-precision-constant -Wdouble-promotion
COM_FLAGS += $(INCLUDES)
COM_FLAGS += $(COM_EXTRA)

CFLAGS = -std=$(STDCC) $(COM_FLAGS)

CXXFLAGS  = -std=$(STDCXX) $(COM_FLAGS)
CXXFLAGS += -fno-rtti -fno-exceptions
CXXFLAGS += -fno-threadsafe-statics -fno-use-cxa-atexit -felide-constructors

# add nano flags to compiler too, see: 
#  https://launchpadlibrarian.net/287100883/readme.txt

NANO_FLAGS=--specs=nano.specs --specs=nosys.specs
CFLAGS += $(NANO_FLAGS)

LDFLAGS += $(CPUFLAGS) -Wl,--gc-section -Wl,-Map=$(MAP)
LDFLAGS += $(LDSCRIPT)
LDFLAGS += -nostartfiles $(NANO_FLAGS)

# 
OPENOCD_ARGS = -f interface/stlink.cfg 
OPENOCD_ARGS += -c 'transport select hla_swd'
OPENOCD_ARGS += -f target/stm32f1x.cfg

#--- real goals ------------------------------------------------------

all: $(OBJS) $(ELF) $(HEX) size

$(ELF): $(OBJS)
	@echo "generating $@"
	$(LINKER_GCC) $(LDFLAGS) $^ -o $@ 

$(BIN): $(ELF)
	$(OBJCOPY) -O binary $< $@

$(HEX): $(ELF)
	$(OBJCOPY) -O ihex $< $@

$(LSS): $(ELF)
	@echo "generating $(BIN) $(HEX) $(LSS) $(SYM)"
	$(OBJREADELF) -x .isr_vector $< > $@
	$(OBJDUMP) -CS $< >> $@
	$(OBJREADELF) -x .rodata -x .data $< >> $@
	$(OBJREADELF) -t $< >> $@
	$(OBJNM) -n -S $< > $(SYM)

#--- PHONY goals -------------------------------------------------------
.PHONY: ALL clean erase debug install kill openocd ram size

ALL: $(OBJS) $(ELF) $(BIN) $(HEX) $(LSS) size

clean:
	@echo "cleaning generated files"
	rm -f $(ELF) $(BIN) $(HEX) $(LSS) $(MAP) $(SYM) $(OBJS) $(DEPS)

debug:
	printf "define exit\nmon reset run\nmon shutdown\nquit\nend\n" > /tmp/usercmds
	openocd $(OPENOCD_ARGS) & \
         xterm -e arm-none-eabi-gdb -ex "target remote :3333" -x /tmp/usercmds -ex 'layout split' $(ELF)

erase: $(HEX)
	openocd $(OPENOCD_ARGS) \
             -c 'init' -c 'halt' -c 'stm32f1x mass_erase 0' -c 'reset'  -c 'shutdown'

install: $(HEX)
ifeq ($(VECT_ADDR),0x20000000)
	openocd $(OPENOCD_ARGS) \
                -c 'init' -c 'halt' -c "load_image $(HEX)" -c 'reset' -c 'exit'
else
	openocd $(OPENOCD_ARGS) -c "program $(HEX) verify reset exit"
endif
	openocd $(OPENOCD_ARGS)  -c 'init;shutdown'

kill:
	killall openocd

openocd: $(HEX)
	openocd $(OPENOCD_ARGS) & xterm -e telnet localhost 4444


size: $(ELF)
	@echo "done!"
	$(OBJSIZE) $(ELF)

#----------------------------------------------------------------------
# build and install into RAM only
# set BOOT switches both to 1 and press the reset button
# then run 'make erase' once to clear flash
ram:
	env LDFILE=ram.ld VECT_ADDR=0x20000000 make clean ALL install

##--- augmented default rules, handle auto creation of depends files --

%.d: %.c Makefile
	@echo "recreating depend file $@"
	$(CC) $(CFLAGS) -MM -MT"$@ $(@:.d=.o)" -MF $@ $<

%.d: %.cpp Makefile
	@echo "recreating depend file $@"
	$(CXX) $(CXXFLAGS) -MM -MT"$@ $(@:.d=.o)" -MF $@ $<

ifneq ($(MAKECMDGOALS),clean)
-include $(DEPS)
endif

ifdef VERBOSE
V ?= 1
endif

$(V).SILENT:

# vim: set ts=8 sw=8 noexpandtab:

