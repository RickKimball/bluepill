#----------------------------------------------------------------------
# Simple Makefile, we let make defaults do all the work.
# We just create environment variables it needs.
#
# TARGET is the .elf file that is loaded on the board
#
# NOTE: executed commands are silent by default.
#       To see verbose commands and args:
#
#  $ make V=1 all

ifdef V
VERBOSE      ?= 1
endif

#--- overridable from the commandline make ---
CPUFLAGS     ?= -mcpu=cortex-m3 -mthumb
F_CPU        ?= 8000000
INCLUDE_DIRS ?= -I ../cmsis -I ../include
OPTIMIZE     ?= -Os
STD          ?= gnu11

AS=arm-none-eabi-gcc
CC=arm-none-eabi-gcc
OBJCOPY=arm-none-eabi-objcopy
OBJDUMP=arm-none-eabi-objdump
OBJNM=arm-none-eabi-nm
OBJREADELF=arm-none-eabi-readelf
OBJSIZE=arm-none-eabi-size

TARGET = example03
SRCS   = $(wildcard *.c)
DEPS   = $(SRCS:.c=.d)
OBJS   = $(SRCS:.c=.o)

BIN := $(TARGET).bin
ELF := $(TARGET).elf
HEX := $(TARGET).hex
LSS := $(TARGET).lss
SYM := $(TARGET).sym

ASFLAGS = -Wall -g $(CPUFLAGS) $(INCLUDE_DIRS) -x assembler
CFLAGS += -Wall -g $(OPTIMIZE) $(CPUFLAGS) -std=$(STD)
CFLAGS += -DF_CPU=$(F_CPU)
CFLAGS += -fwrapv
CFLAGS += -fsingle-precision-constant -Wdouble-promotion
CFLAGS += -ffunction-sections
CFLAGS += $(INCLUDE_DIRS)

LDSCRIPT = -T stm32f103c8.ld
LDFLAGS += -Wl,--gc-section
LDFLAGS += $(LDSCRIPT)
LDFLAGS += -nostartfiles --specs=nano.specs --specs=nosys.specs

OPENOCD_ARGS = -f interface/stlink.cfg 
OPENOCD_ARGS += -c 'transport select hla_swd'
OPENOCD_ARGS += -f target/stm32f1x.cfg

#------------------------------------------------------------------------------

all: $(OBJS) $(TARGET) $(HEX) size

$(TARGET): $(OBJS)

$(ELF): $(TARGET)
	@echo "generating $@"
	cp $< $@

$(BIN): $(ELF)
	$(OBJCOPY) -O binary $< $@

$(HEX): $(ELF)
	$(OBJCOPY) -O ihex $< $@

$(LSS): $(ELF)
	@echo "generating $(BIN) $(HEX) $(LSS) $(SYM)"
	$(OBJREADELF) -x .isr_vectors $< > $@
	$(OBJDUMP) -CS $< >> $@
	$(OBJREADELF) -x .rodata -x .data $< >> $@
	$(OBJREADELF) -t $< >> $@
	$(OBJNM) -n -S $< > $(SYM)

#-------- PHONY targets -------------------------------------------------------
.PHONY: ALL clean debug size upload

ALL: $(OBJS) $(TARGET) $(ELF) $(BIN) $(HEX) $(LSS) size

clean:
	@echo "cleaning generated files"
	rm -f $(TARGET) $(ELF) $(OBJS) $(DEPS) $(BIN) $(HEX) $(LSS) $(SYM)

debug: upload
	openocd $(OPENOCD_ARGS) & xterm -e arm-none-eabi-gdb -ex "target remote :3333" -ex 'layout split' $(ELF)
	sleep 1
	killall openocd

size: $(ELF)
	@echo "done!"
	$(OBJSIZE) $(ELF)

upload: $(HEX)
	openocd $(OPENOCD_ARGS) -c "program $(HEX) verify reset exit"

## augmented auto depends

%.d: %.c Makefile
	$(CC) -MM -MT"$@ $(@:.d=.o)" -MF$@ $(CFLAGS) $(INCLUDES) $<

ifneq ($(MAKECMDGOALS),clean)
-include $(DEPS)
endif

$(VERBOSE).SILENT:

