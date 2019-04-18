#-----------------------------------------------------------------------
# Makefile - somewhat simple makefile for the bluepill running on HSI
#
# The idea of this makefile is to use as many defaults from
# the make default rules as we can. It knows what to do with
# CC, CFLAGS, LDFLAGS etc... so use them by just populating
# them and letting the default transforms happen. We just
# need to fill in the values.
#

# NOTE: Executed commands are silent by default.
#       To see verbose compile command output run:
#  $ make V=1 all

#--- override these from the make commandline ---
GCC_PATH      ?=              # you need to supply trailing '/' so ... '/usr/bin/'
PREFIX        ?= arm-none-eabi-
DEBUGSYM_FLAG ?= -g
F_CPU         ?= 24000000
INCLUDES      ?= . ../cmsis ../include FreeRTOS FreeRTOS/include FreeRTOS/portable/GCC/ARM_CM3
LDFILE        ?= stm32f103c8.ld
LDSCRIPT      ?= -T $(LDFILE)
LINKER_GCC    ?= $(PREFIX)g++
OPTIMIZE      ?= -Os
STDCC         ?= gnu11
STDCXX        ?= gnu++14
TARGET        ?= main
TARGET_ARCH   ?= -mcpu=cortex-m3 -mthumb
TARGET_MACH   ?= $(TARGET_ARCH)
USER_CPPFLAGS ?= -U NO_DATA_INIT -U NO_BSS_INIT
VECT_ADDR     ?= 0x08000000

#--- assumes the compiler can be found in $PATH  ----------------------
AS         	:= $(GCC_PATH)$(PREFIX)gcc -c -x assembler
CC         := $(GCC_PATH)$(PREFIX)gcc
CXX        := $(GCC_PATH)$(PREFIX)g++
LD         := $(GCC_PATH)$(PREFIX)ld
AR         := $(GCC_PATH)$(PREFIX)ar
OBJCOPY    := $(GCC_PATH)$(PREFIX)objcopy
OBJDUMP    := $(GCC_PATH)$(PREFIX)objdump
OBJNM      := $(GCC_PATH)$(PREFIX)nm
OBJREADELF := $(GCC_PATH)$(PREFIX)readelf
OBJSIZE    := $(GCC_PATH)$(PREFIX)size
LINK.cc    := $(CXX)

#--- gather the files to compile --------------------------------------
SRCS  = $(filter %.c %.cpp %.s %.S, $(wildcard *.*))
SRCS += $(filter %.c %.cpp %.s %.S, $(wildcard FreeRTOS/*.*))
SRCS += FreeRTOS/portable/GCC/ARM_CM3/port.c
SRCS += FreeRTOS/portable/MemMang/heap_3.c
OBJ0 := $(SRCS:.c=.o)
OBJ1 := $(SRCS:.cpp=.o)
OBJ2 := $(SRCS:.s=.o)
OBJ3 := $(SRCS:.S=.o)
OBJX := $(OBJ0) $(OBJ1) $(OBJ2) $(OBJ3)
OBJS := $(filter %.o, $(OBJX))
DEPS := $(OBJS:.o=.d)
INCS := $(addprefix -I,$(INCLUDES))
#INCS := $(foreach d, $(INCLUDES), -I"$d")

BIN := $(TARGET).bin
ELF := $(TARGET).elf
HEX := $(TARGET).hex
LSS := $(TARGET).lss
MAP := $(TARGET).map
SYM := $(TARGET).sym

# add nano flags to compiler too, see: 
#  https://launchpadlibrarian.net/287100883/readme.txt
NANO_FLAGS=--specs=nano.specs --specs=nosys.specs

COMMON_FLAGS += $(NANO_FLAGS)
COMMON_FLAGS += -Wall $(DEBUGSYM_FLAG) $(OPTIMIZE) $(TARGET_ARCH)
COMMON_FLAGS += -fwrapv -ffunction-sections
COMMON_FLAGS += -fsingle-precision-constant -Wdouble-promotion
COMMON_FLAGS += -fno-asynchronous-unwind-tables -fomit-frame-pointer
 
CPPFLAGS += -DF_CPU=$(F_CPU) -DVECT_ADDR=$(VECT_ADDR)
CPPFLAGS += $(INCS)
CPPFLAGS += $(USER_CPPFLAGS)

ASFLAGS   = -Wall $(DEBUGSYM_FLAG) 
 
CFLAGS    = -std=$(STDCC) $(COMMON_FLAGS)
CXXFLAGS  = -std=$(STDCXX) $(COMMON_FLAGS)
CXXFLAGS += -fno-rtti -fno-exceptions
CXXFLAGS += -fno-threadsafe-statics -fno-use-cxa-atexit -felide-constructors

LDFLAGS  = $(TARGET_ARCH) -Wl,--gc-section -Wl,-Map=$(MAP)
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
.PHONY: ALL clean clobber erase debug install kill listing openocd putty ram size version

ALL: $(OBJS) $(ELF) $(BIN) $(HEX) listing

clean: clobber
	@echo "cleaning generated listing files"
	rm -f $(LSS) $(MAP) $(SYM)

clobber:
	@echo "cleaning generated executable files"
	rm -f $(DEPS) $(OBJS) $(ELF) $(BIN) $(HEX) openocd.log
	rm -rf .vscode/ipch

erase: $(HEX)
	openocd $(OPENOCD_ARGS) \
             -c 'init' -c 'halt' -c 'stm32f1x mass_erase 0' -c 'reset'  -c 'shutdown'

debug:
	printf "define exit\nmon reset run\nmon shutdown\nquit\nend\n" > /tmp/usercmds
	xterm -e $(GCC_PATH)$(PREFIX)gdb \
		-ex "target remote | openocd -c 'gdb_port pipe; log_output openocd.log' $(OPENOCD_ARGS)" \
		-x /tmp/usercmds \
		-ex 'layout split' $(ELF)
	rm /tmp/usercmds

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

listing: $(LSS) size

openocd: $(HEX)
	openocd $(OPENOCD_ARGS) & xterm -e telnet localhost 4444

putty:
	putty /dev/ttyUSB0 -serial -sercfg 115200,8,n,1 &

#----------------------------------------------------------------------
# build and install into RAM only
# set BOOT switches both to 1 and press the reset button
# then run 'make erase' once to clear flash
ram:
	env LDFILE=ram.ld VECT_ADDR=0x20000000 make clean ALL install

size: $(ELF)
	@echo "done!"
	$(OBJSIZE) $(ELF)

version:
	( $(CC) --version && ( $(CC) $(CFLAGS) $(CPPFLAGS) -dM -E - < /dev/null | sort )) | less

##--- augmented default rules, handle auto creation of depends files --

%.d: %.c Makefile
	@echo "recreating depend file $@"
	$(CC) $(CFLAGS) $(CPPFLAGS) -MM -MT"$@ $(@:.d=.o)" -MF $@ $<

%.d: %.cpp Makefile
	@echo "recreating depend file $@"
	$(CXX) $(CXXFLAGS) $(CPPFLAGS) -MM -MT"$@ $(@:.d=.o)" -MF $@ $<

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(MAKECMDGOALS),clobber)
-include $(DEPS)
endif
endif

$(V).SILENT:

# vim: set ts=8 sw=8 noexpandtab:

