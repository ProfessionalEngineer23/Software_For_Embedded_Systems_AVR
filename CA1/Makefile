# --- config ---
MCU        := atmega328p
F_CPU      := 1000000UL
PROGRAMMER := stk500
PORT       := /dev/ttyACM0
BAUD       := 9600

# If you switch to usart.c later, just change this one line:
USART_SRC  := usart1.c

# Programs
PROGS := mainc mainb

# --- tools ---
CC      := avr-gcc
OBJCOPY := avr-objcopy
AVRDUDE := avrdude

# --- flags ---
CFLAGS  := -mmcu=$(MCU) -DF_CPU=$(F_CPU) -O2 -flto \
           -ffunction-sections -fdata-sections \
           -fno-split-wide-types -Wall -Wextra
LDFLAGS := -mmcu=$(MCU) -flto -Wl,--gc-sections

# --- per-program sources ---
mainc_SOURCES := mainc.c $(USART_SRC)
mainb_SOURCES := mainb.c $(USART_SRC)

# --- derived objects/binaries ---
mainc_OBJECTS := $(mainc_SOURCES:.c=.o)
mainb_OBJECTS := $(mainb_SOURCES:.c=.o)

mainc_ELF := mainc.elf
mainc_HEX := mainc.hex
mainb_ELF := mainb.elf
mainb_HEX := mainb.hex

# --- top-level targets ---
.PHONY: all clean term upload-mainc upload-mainb

# Build both by default
all: $(mainc_HEX) $(mainb_HEX)

# Upload each explicitly
upload-mainc: $(mainc_HEX)
	$(AVRDUDE) -c $(PROGRAMMER) -p $(MCU) -P $(PORT) -U flash:w:$<:i

upload-mainb: $(mainb_HEX)
	$(AVRDUDE) -c $(PROGRAMMER) -p $(MCU) -P $(PORT) -U flash:w:$<:i

# Open GTK Term
term:
	gtkterm -p $(PORT) -s $(BAUD)

# --- link rules ---
$(mainc_ELF): $(mainc_OBJECTS)
	$(CC) $(LDFLAGS) -o $@ $^

$(mainb_ELF): $(mainb_OBJECTS)
	$(CC) $(LDFLAGS) -o $@ $^

# --- hex from elf ---
$(mainc_HEX): $(mainc_ELF)
	$(OBJCOPY) -O ihex -R .eeprom $< $@

$(mainb_HEX): $(mainb_ELF)
	$(OBJCOPY) -O ihex -R .eeprom $< $@

# --- compile .c to .o ---
%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

# --- clean ---
clean:
	rm -f $(mainc_OBJECTS) $(mainb_OBJECTS) $(mainc_ELF) $(mainb_ELF) $(mainc_HEX) $(mainb_HEX)
