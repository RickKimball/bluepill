# Simple Makefile for a single file
# we let make do all the work with its defaults
# result TARGET is the .elf file

# silent by default, use make V=1 to be verbose

CC=arm-none-eabi-gcc
OBJCOPY=arm-none-eabi-objcopy
OBJDUMP=arm-none-eabi-objdump
OBJNM=arm-none-eabi-nm
OBJREADELF=arm-none-eabi-readelf
OBJSIZE=arm-none-eabi-size

SRCS=vectors.c utils.c example03.c
OBJS=$(SRCS:.c=.o)
DEPS=$(SRCS:.c=.d)
TARGET=example03

BIN=$(TARGET).bin
ELF=$(TARGET).elf
HEX=$(TARGET).hex
LSS=$(TARGET).lss
SYM=$(TARGET).sym

OPTIMIZE?=-Os
CPUFLAGS?=-mcpu=cortex-m3 -mthumb
INCLUDE_DIRS=-I ../cmsis -I ../include
F_CPU?=8000000

CFLAGS+=-Wall -g $(OPTIMIZE) $(CPUFLAGS)
CFLAGS+=-DF_CPU=$(F_CPU)
CFLAGS+=-fsingle-precision-constant -Wdouble-promotion
CFLAGS+=-ffunction-sections
CFLAGS+=$(INCLUDE_DIRS)

LDSCRIPT=-T stm32f103c8.ld
LDFLAGS+=-Wl,--gc-section
LDFLAGS+=$(LDSCRIPT)
LDFLAGS+=-nostartfiles --specs=nano.specs --specs=nosys.specs

OPENOCD_ARGS=-f interface/stlink.cfg -c 'transport select hla_swd' -f target/stm32f1x.cfg

# the make command by default will create example02 from example02.c
# using the default rules, we take advantage of that 
# by just using the ENV variables it expects
.PHONY: ALL clean debug size upload

all: $(OBJS) $(TARGET) $(ELF) size Makefile stm32f103c8.ld

$(TARGET): $(OBJS)


$(BIN): $(TARGET)
	$(OBJCOPY) -O binary $(TARGET) $(BIN)

$(ELF): $(TARGET)
	@echo "generating $(TARGET).elf"
	cp $(TARGET) $(ELF)

$(HEX): $(TARGET)
	$(OBJCOPY) -O ihex $(TARGET) $(HEX)

$(LSS):
	@echo "generating $(TARGET).bin $(TARGET).hex $(TARGET).lss $(TARGET).sym"
	$(OBJREADELF) -x .isr_vectors -x .rodata -x .data $(TARGET) >$(LSS)
	$(OBJDUMP) -CS $(TARGET) >>$(LSS)
	$(OBJREADELF) -t $(TARGET) >>$(LSS)
	$(OBJNM) -n -S $(TARGET) >>$(SYM)


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
	openocd $(OPENOCD_ARGS) -c "program $(ELF) verify reset exit"

## augmented auto depends

%.d:	%.c Makefile
	$(CC) -MM -MT"$@ $(@:.d=.o)" -MF$@ $(CFLAGS) $(INCLUDES) $<

ifneq ($(MAKECMDGOALS),clean)
-include $(DEPS)
endif

$(V).SILENT:

