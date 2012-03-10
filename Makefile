MCU = atmega328p
F_CPU = 16000000

FORMAT = ihex

CC = avr-gcc
FLAGS = -Wall -pedantic -Os -std=c99 -g -DF_CPU=$(F_CPU) -mmcu=$(MCU)
OBJCOPY = avr-objcopy
DFU_PRG = dfu-programmer
PORT ?= /dev/tty.usbmodem*
STTY = stty
AVRDUDE = avrdude
PROGRAMMER = arduino

SRC = $(wildcard *.c)
OBJ = $(subst .c$,.o,$(SRC))
UART = ./lib/uart

programme: $(FILE).hex
	$(STTY) -f $(PORT) hupcl
	$(AVRDUDE) -p $(MCU) -P $(PORT) -c $(PROGRAMMER) -U flash:w:$<

$(FILE).hex: $(FILE).elf
	$(OBJCOPY) -j .text -j .data -O $(FORMAT) $< $@

$(FILE).elf: $(OBJ) $(UART).o
	echo $(OBJ)
	$(CC) -g -mmcu=$(MCU) -o $@ $+

$(UART).o: $(UART).c
	$(CC) $(FLAGS) -c -o $@ $<

%.o: %.c
	$(CC) $(FLAGS) -c -o $@ $<

.PHONY: clear

clean:
	rm -f *.elf *.o *.hex
	cd lib; rm -f *.elf *.o
