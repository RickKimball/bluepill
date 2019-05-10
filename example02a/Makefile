# Simple Makefile
# we let make do all the work with its defaults
# result TARGET is the .elf file

CC=arm-none-eabi-gcc
OBJCOPY=arm-none-eabi-objcopy
OBJDUMP=arm-none-eabi-objdump
OBJNM=arm-none-eabi-nm
OBJREADELF=arm-none-eabi-readelf
OBJSIZE=arm-none-eabi-size

SRCS=$(wildcard *.c)
OBJS=$(SRCS:.c=.o)
DEPS=$(SRCS:.c=.d)
TARGET=$(SRCS:.c=)

BIN=$(TARGET).bin
ELF=$(TARGET).elf
HEX=$(TARGET).hex
LSS=$(TARGET).lss
SYM=$(TARGET).sym

OPTIMIZE?=-Os
CPUFLAGS?=-mcpu=cortex-m0 -mthumb
LDSCRIPT=-T stm32f030r8.ld
INCLUDE_DIRS=-I CMSIS 
OPENOCD_ARGS=-f interface/stlink.cfg -c 'transport select hla_swd' -f target/stm32f0x.cfg

CFLAGS+=-Wall -g $(OPTIMIZE) $(CPUFLAGS)
CFLAGS+=-fsingle-precision-constant -Wdouble-promotion
CFLAGS+=$(INCLUDE_DIRS)

LDFLAGS+=$(LDSCRIPT) -nostartfiles -nodefaultlibs

all: $(TARGET) $(BIN) $(ELF) $(HEX) $(LSS) size

$(BIN): $(TARGET)
	$(OBJCOPY) -O binary $(TARGET) $(BIN)

$(ELF): $(TARGET)
	cp $(TARGET) $(ELF)

$(HEX): $(TARGET)
	$(OBJCOPY) -O ihex $(TARGET) $(HEX)

$(LSS):
	$(OBJREADELF) -x .isr_vectors -x .rodata -x .data $(TARGET) >$(LSS)
	$(OBJDUMP) -CS $(TARGET) >>$(LSS)
	$(OBJREADELF) -t $(TARGET) >>$(LSS)
	$(OBJNM) -n -S $(TARGET) >>$(SYM)

.PHONY: clean debug size upload

clean:
	rm -f $(OBJS) $(DEPS) $(TARGET) $(BIN) $(ELF) $(HEX) $(LSS) $(SYM)

debug: upload
	openocd $(OPENOCD_ARGS) & xterm -e arm-none-eabi-gdb -ex "target remote :3333" -ex 'layout split' $(ELF)
	sleep 1
	killall openocd

size:
	$(OBJSIZE) $(ELF)

upload: $(ELF)
	openocd $(OPENOCD_ARGS) -c "program $(ELF) verify reset exit"

-include $(DEPS)
