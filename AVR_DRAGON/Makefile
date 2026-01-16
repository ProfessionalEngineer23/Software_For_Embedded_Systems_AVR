#My makefile for AVR_Debugging

#Flags
CC := gcc

#Rules

all:
	avr-gcc -mmcu=atmega328p -Og -ggdb -o blinky.elf blinky.c
	avr-objcopy -O ihex blinky.elf blinky.hex

flash:
	avrdude -c dragon_isp -p m328p -P usb -U flash:w:blinky.hex:i

fuse:
	avrdude -c dragon_isp -p m328p -U efuse:r:-:h -U hfuse:r:-:h -U lfuse:r:-:h

dwen:
	avrdude -c dragon_isp -p m328p -P usb -U hfuse:w:0x99:m

nodwen:
	avrdude -c dragon_isp -p m328p -P usb -U hfuse:w:0xd9:m

server:
	avarice -g -w -d -P atmega328p :3333

gdb:
	avr-gdb --tui blinky.elf

Clean:
	rm -f blinky.elf blinky.hex
