CC=avr-gcc
LD=avr-gcc
MCU=atmega644p
F_CPU=16000000L
OPTLEVEL=3
CFLAGS=-mmcu=$(MCU) -std=gnu99 -O$(OPTLEVEL) -DF_CPU=$(F_CPU) -Wall -DUSE_SERIAL_STDIO -DDEBUGMSG
LDFLAGS=-mmcu=$(MCU) 
OBJCOPY=avr-objcopy
OBJDUMP=avr-objdump
FORMAT=ihex
OBJS=main.o serial-avr644p.o up6502.o

all: 6502 lst

6502: 6502.hex

lst: 6502.lst

clean:
	rm -rf *.hex *.elf *.o *.lst

%.hex: %.elf
	$(OBJCOPY) -O $(FORMAT) -R .eeprom $< $@

%.lst: %.elf
	$(OBJDUMP) -h -S $< > $@

6502.elf: $(OBJS)
	$(LD) $(LDFLAGS) $(OBJS) -o 6502.elf

.o: .c
	$(CC) $(CFLAGS) -c $<

burn: 6502
	avrdude -V -c usbasp -p $(MCU) -U flash:w:6502.hex 
fuses: 6502
	avrdude -V -c usbasp -p $(MCU) -U lfuse:w:0xff:m -U hfuse:w:0xdf:m

