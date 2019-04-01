# Simple Makefile, we let make defaults do all the work
# We just create environment variables it uses.
#
# TARGET is the .elf file used to load on the board

# NOTE: executed commands are silent by default, use:
# $ make V=1 all
# to see command and arguments used

CPUFLAGS     ?= -mcpu=cortex-m3 -mthumb
F_CPU        ?= 8000000
INCLUDE_DIRS ?= -I ../cmsis -I ../include
OPTIMIZE     ?= -Os
STD          ?= gnu11

CC=arm-none-eabi-gcc
OBJCOPY=arm-none-eabi-objcopy
OBJDUMP=arm-none-eabi-objdump
OBJNM=arm-none-eabi-nm
OBJREADELF=arm-none-eabi-readelf
OBJSIZE=arm-none-eabi-size

TARGET = example03
SRCS   = example03.c utils.c vectors.c
DEPS   = $(SRCS:.c=.d)
OBJS   = $(SRCS:.c=.o)

BIN=$(TARGET).bin
ELF=$(TARGET).elf
HEX=$(TARGET).hex
LSS=$(TARGET).lss
SYM=$(TARGET).sym

CFLAGS+=-Wall -g $(OPTIMIZE) $(CPUFLAGS) -std=$(STD)
CFLAGS+=-DF_CPU=$(F_CPU)
CFLAGS+=-fsingle-precision-constant -Wdouble-promotion
CFLAGS+=-ffunction-sections
CFLAGS+=$(INCLUDE_DIRS)

LDSCRIPT=-T stm32f103c8.ld
LDFLAGS+=-Wl,--gc-section
LDFLAGS+=$(LDSCRIPT)
LDFLAGS+=-nostartfiles --specs=nano.specs --specs=nosys.specs

OPENOCD_ARGS=-f interface/stlink.cfg -c 'transport select hla_swd' -f target/stm32f1x.cfg

# the make command by default will create TARGET from TARGET.c
# using the default rules given the correct ENV names 

all: $(OBJS) $(TARGET) $(HEX) size Makefile stm32f103c8.ld

$(TARGET): $(OBJS)

$(ELF): $(TARGET)
	@echo "generating $(TARGET).elf"
	cp $(TARGET) $(ELF)

$(BIN): $(ELF)
	$(OBJCOPY) -O binary $(TARGET) $(BIN)

$(HEX): $(ELF)
	$(OBJCOPY) -O ihex $(TARGET) $(HEX)

$(LSS): $(ELF)
	@echo "generating $(TARGET).bin $(TARGET).hex $(TARGET).lss $(TARGET).sym"
	$(OBJREADELF) -x .isr_vectors -x .rodata -x .data $(TARGET) >$(LSS)
	$(OBJDUMP) -CS $(TARGET) >>$(LSS)
	$(OBJREADELF) -t $(TARGET) >>$(LSS)
	$(OBJNM) -n -S $(TARGET) >>$(SYM)

#-------- phony targets ---------------------------------------------------------
.PHONY: ALL clean debug size upload

ALL: $(OBJS) $(TARGET) $(ELF) $(BIN) $(HEX) $(LSS) size Makefile stm32f103c8.ld

clean:
	@echo "cleaning generated files"
	rm -f $(TARGET) $(ELF) $(OBJS) $(DEPS) $(BIN) $(HEX) $(LSS) $(SYM)

debug: upload
	openocd $(OPENOCD_ARGS) & xterm -e arm-none-eabi-gdb -ex "target remote :3333" -ex 'layout split' $(ELF)
	sleep 1
	killall openocd

size:
	@echo "done!"
	$(OBJSIZE) $(ELF)

upload: $(ELF)
	openocd $(OPENOCD_ARGS) -c "program $(HEX) verify reset exit"

## augmented auto depends

%.d: %.c Makefile
	$(CC) -MM -MT"$@ $(@:.d=.o)" -MF$@ $(CFLAGS) $(INCLUDES) $<

ifneq ($(MAKECMDGOALS),clean)
-include $(DEPS)
endif

$(V).SILENT:

