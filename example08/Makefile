#--------------------------------------------------------------------------------
# Makefile - example08 cooperative multi-tasking with UART and Blink
#
#     features cleaner Makefile with BUILD_DIR output, also demostrates
#     "coloring" memory to help debugging stack and heap issues.
#     added armv7m-vecstate.gdb macros.
#     Compiled using the latest linaro compiler arm-eabi-gcc v8.3.0
#--------------------------------------------------------------------------------
# using the (arm-eabi) binary download from here:
# https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-a/downloads
#--------------------------------------------------------------------------------
COLOR_RAM   ?= -UCOLOR_RAM
DEBUG_FLAGS ?= -Wall -g
F_CPU       ?= 24000000L
GCC_PATH    ?= ~/gcc-arm-8.3-2019.03-x86_64-arm-eabi/bin/
PREFIX      ?= arm-eabi-
LDFILE      ?= stm32f103cb.ld
OPTIMIZE    ?= -Os
STDC        ?= gnu11
STDCXX      ?= gnu++17
TARGET      ?= firmware
TTY         ?= /dev/ttyUSB0
VECT_ADDR   ?= 0x08000000
YYMAXFILE   ?= 3

#--------------------------------------------------------------------------------
AS_SRC     = \
 	$(filter %.S, $(wildcard *.*))

C_SRC      = \
 	$(filter %.c, $(wildcard *.*))

CXX_SRC    = \
 	$(filter %.cpp, process.cpp $(filter-out process.cpp, $(wildcard *.*))) \
 	$(filter %.cpp, $(wildcard ../3rd_party/Scheduler/src/*.*))

INCLUDES   = \
 	../include \
 	../cmsis \
 	../3rd_party \
 	../3rd_party/Scheduler/src

BLD_DIR   := build
BUILD_DIR := $(BLD_DIR)/src

#--------------------------------------------------------------------------------
AS         = $(GCC_PATH)$(PREFIX)gcc -c -x assembler-with-cpp
CC         = $(GCC_PATH)$(PREFIX)gcc
CXX        = $(GCC_PATH)$(PREFIX)g++
GDB        = $(GCC_PATH)$(PREFIX)gdb
OBJCOPY    = $(GCC_PATH)$(PREFIX)objcopy
OBJDUMP    = $(GCC_PATH)$(PREFIX)objdump
OBJNM      = $(GCC_PATH)$(PREFIX)nm
OBJREADELF = $(GCC_PATH)$(PREFIX)readelf
SIZE       = $(GCC_PATH)$(PREFIX)size

#--------------------------------------------------------------------------------
CPPFLAGS = \
 -DF_CPU=$(F_CPU) -DVECT_ADDR=$(VECT_ADDR) $(COLOR_RAM)\
 $(INCLUDES:%=-I"%")

COMMON_CFLAGS = \
 $(OPTIMIZE) \
 $(DEBUG_FLAGS) \
 --specs=nano.specs --specs=nosys.specs \
 -ffunction-sections \
 -fwrapv \
 -fsingle-precision-constant -Wdouble-promotion \
 -fno-asynchronous-unwind-tables -fomit-frame-pointer \
 -MD -MP -MF $(BUILD_DIR)/$(@F:.o=.d) 
 
ASFLAGS = $(DEBUG_FLAGS)

CFLAGS = -std=$(STDC) $(COMMON_CFLAGS)

CXXFLAGS = \
 -std=$(STDCXX) $(COMMON_CFLAGS) \
 -fno-rtti -fno-exceptions -felide-constructors \
 -fno-threadsafe-statics -fno-use-cxa-atexit 

LDFLAGS = -mcpu=cortex-m3 -mthumb  -Wl,--gc-section \
 -Wl,-Map=$(MAP) \
 -T $(LDFILE) -nostartfiles --specs=nano.specs --specs=nosys.specs

 RE2CFLAGS ?= -W --no-generation-date

TARGET_ARCH = -mcpu=cortex-m3 -mthumb

#--------------------------------------------------------------------------------
ASOBJS  = $(AS_SRC:%.S=$(BUILD_DIR)/%.o)
COBJS   = $(C_SRC:%.c=$(BUILD_DIR)/%.o)
CXXOBJS = $(CXX_SRC:%.cpp=$(BUILD_DIR)/%.o)
DEPS    = $(C_SRC:%.c=$(BUILD_DIR)/%.d) $(CXX_SRC:%.cpp=$(BUILD_DIR)/%.d)

BIN := $(BUILD_DIR)/$(TARGET).bin
ELF := $(BUILD_DIR)/$(TARGET).elf
HEX := $(BUILD_DIR)/$(TARGET).hex
LSS := $(BUILD_DIR)/$(TARGET).lss
MAP := $(BUILD_DIR)/$(TARGET).map
SYM := $(BUILD_DIR)/$(TARGET).sym

#--------------------------------------------------------------------------------
OPENOCD_ARGS  = -f interface/stlink.cfg 
OPENOCD_ARGS += -c 'transport select hla_swd'
OPENOCD_ARGS += -f target/stm32f1x.cfg
OPENOCD_LOG   = $(BUILD_DIR)/openocd.log

#--------------------------------------------------------------------------------
.PHONY: ALL all clean clobber erase debug install kill listing openocd putty ram version

all: $(ELF)
	@echo TARGET=$(TARGET) finished
	$(SIZE) $<

$(ELF): $(COBJS) $(CXXOBJS) $(ASOBJS)
	@echo linking $@ ...
	$(CXX) $(TARGET_ARCH) $(LDFLAGS) $^ -o $@

$(ASOBJS): $(BUILD_DIR)/%.o : %.S
	@echo assembling $< ...
	mkdir -p $(@D)
	$(AS) $(TARGET_ARCH) $(ASFLAGS) $(CPPFLAGS) -c -o $@ $<

$(COBJS): $(BUILD_DIR)/%.o : %.c
	@echo compiling $< ...
	mkdir -p $(@D)
	$(CC) $(TARGET_ARCH) $(CFLAGS) $(CPPFLAGS) -c -o $@ $<

$(CXXOBJS): $(BUILD_DIR)/%.o : %.cpp
	@echo compiling $< ...
	mkdir -p $(@D)
	$(CXX) $(TARGET_ARCH) $(CXXFLAGS) $(CPPFLAGS) -c -o $@ $<

$(BIN): $(ELF)
	$(OBJCOPY) -O binary $< $@

$(HEX): $(ELF)
	$(OBJCOPY) -O ihex $< $@

$(LSS): $(ELF)
	@echo "generating $(BIN) $(HEX) $(LSS) $(SYM)"
	$(OBJREADELF) -x .isr_vector $<       > $@
	$(OBJDUMP) -CS $<                    >> $@
	$(OBJREADELF) -x .rodata -x .data $< >> $@
	$(OBJREADELF) -t $<                  >> $@
	$(OBJNM) -n -C -S $<                  > $(SYM)

%.cpp: %.re process.h
	@echo generating $@ from $<
	re2c $(RE2CFLAGS) -o $@ $<
	@echo "#pragma once"        > process_defines.h
	grep YYMAXFILL process.cpp >> process_defines.h

#--------------------------------------------------------------------------------
ALL: $(BIN) $(HEX) $(LSS) all

clean:
	rm -f $(ASOBJS) $(COBJS) $(CXXOBJS) \
			$(LSS) $(MAP) $(SYM) \
			$(HEX) $(ELF) $(BIN) \
			$(OPENOCD_LOG) $(DEPS)

clobber:
	rm -rf $(BLD_DIR) .vscode/ipch process.cpp

erase:
	openocd $(OPENOCD_ARGS) \
             -c 'init' -c 'halt' -c 'stm32f1x mass_erase 0' -c 'reset'  -c 'shutdown'

debug:
	xterm -geometry 95x62 -e $(GDB) \
		-ex "target remote \
                    | openocd -c 'gdb_port pipe; log_output $(OPENOCD_LOG)' $(OPENOCD_ARGS)" \
		-ex 'source armv7m-vecstate.gdb' \
		-ex 'layout split' -ex 'winheight cmd 25' \
		-ex 'mon reset halt' \
		$(ELF)

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

putty:
	putty $(TTY) -serial -sercfg 115200,8,n,1 &

#----------------------------------------------------------------------
# build and install into RAM only
# set BOOT0/BOOT1 jumpers both to 1 and press the reset button
# then run 'make erase' once to clear flash
# then run 'make ram'
ram:
	env LDFILE=ram.ld VECT_ADDR=0x20000000 make clean ALL install

version:
	( $(CC) --version && ( $(CC) $(CFLAGS) $(CPPFLAGS) -dM -E - < /dev/null | sort )) | less

$(V).SILENT:

.PRECIOUS: . $(AS_SRC) $(C_SRC) $(CXX_SRC)

.SECONDARY:

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(MAKECMDGOALS),clobber)
 -include $(DEPS)
endif
endif

# vim: set ts=4 sw=4 noexpandtab :
