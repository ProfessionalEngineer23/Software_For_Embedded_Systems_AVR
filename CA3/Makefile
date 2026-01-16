# Makefile for Alarm_Bug2.c

# --------- Config ---------
MCU        = atmega328p
F_CPU      = 1000000UL
CC         = avr-gcc
OBJCOPY    = avr-objcopy
PROGRAMMER = dragon_isp
PORT       = usb

SRC        = Alarm_Bug2.c

# Normal build files
ELF        = Alarm_Bug2.elf
HEX        = Alarm_Bug2.hex

# Debug build files (with -DDEBUG)
ELF_DEBUG  = Alarm_Bug2_debug.elf
HEX_DEBUG  = Alarm_Bug2_debug.hex

ASM        = Alarm_Bug2.s

CFLAGS_COMMON = -mmcu=$(MCU) -DF_CPU=$(F_CPU) -Og -ggdb -fno-inline

.PHONY: all normal debug asm flash flash_debug clean \
        check_fuses enable_dwen disable_dwen \
        avarice avarice_example gdb

# --------- Default ---------
all: normal

# --------- Builds ---------

# Normal build (with delays)
normal: $(ELF) $(HEX)

$(ELF): $(SRC)
	@echo "Compiling normal build"
	@$(CC) $(CFLAGS_COMMON) -o $(ELF) $(SRC)
	@echo "Compilation completed"

$(HEX): $(ELF)
	@echo "Generating HEX file"
	@$(OBJCOPY) -O ihex $(ELF) $(HEX)
	@echo "HEX generation completed"

# Debug build (DEBUG defined, delays compiled out)
debug: $(ELF_DEBUG) $(HEX_DEBUG)

$(ELF_DEBUG): $(SRC)
	@echo "Compiling debug build with DEBUG flag"
	@$(CC) $(CFLAGS_COMMON) -DDEBUG -o $(ELF_DEBUG) $(SRC)
	@echo "Debug compilation completed"

$(HEX_DEBUG): $(ELF_DEBUG)
	@echo "Generating HEX file without EEPROM section"
	@$(OBJCOPY) -O ihex -R .eeprom $(ELF_DEBUG) $(HEX_DEBUG)
	@echo "HEX generation completed"

# Generate assembly from debug build
asm:
	@echo "Generating assembly listing"
	@$(CC) -mmcu=$(MCU) -DF_CPU=$(F_CPU) -DDEBUG -Og -fno-inline -S $(SRC) -o $(ASM)
	@echo "Assembly generation completed"

# --------- Programming / Fuses ---------

# Flash normal build
flash: $(HEX)
	@echo "Flashing normal program to device"
	@avrdude -c $(PROGRAMMER) -p m328p -P $(PORT) -U flash:w:$(HEX):i
	@echo "Flashing completed"

# Flash debug build (no delays)
flash_debug: $(HEX_DEBUG)
	@echo "Flashing debug program to device"
	@avrdude -c $(PROGRAMMER) -p m328p -P $(PORT) -U flash:w:$(HEX_DEBUG):i
	@echo "Flashing completed"

check_fuses:
	@echo "Reading fuses"
	@avrdude -c $(PROGRAMMER) -p m328p -U efuse:r:-:h -U hfuse:r:-:h -U lfuse:r:-:h
	@echo "Fuse read completed"

enable_dwen:
	@echo "Enabling DWEN on reset pin"
	@avrdude -c $(PROGRAMMER) -p m328p -P $(PORT) -U hfuse:w:0x99:m
	@echo "DWEN enabled"

disable_dwen:
	@echo "Disabling DWEN on reset pin"
	@avrdude -c $(PROGRAMMER) -p m328p -P $(PORT) -U hfuse:w:0xD9:m
	@echo "DWEN disabled"

# --------- Debug helpers ---------

# Start avarice with debug ELF
avarice: debug
	@echo "Starting Avarice server with debug ELF file"
	@avarice --dragon --debugwire -P $(MCU) -g -w -f $(ELF_DEBUG) :3333
	@echo "Avarice server finished"

avarice_example:
	@echo "Starting Avarice server classroom example"
	@avarice -g -w -d -P $(MCU) :3333
	@echo "Avarice server finished"

# Run gdb on debug ELF
gdb: debug
	@echo "Starting avr-gdb with debug ELF"
	@avr-gdb --tui $(ELF_DEBUG)
	@echo "GDB session finished"

# --------- Cleanup ---------

clean:
	@echo "Cleaning build files"
	@rm -f $(ELF) $(HEX) $(ELF_DEBUG) $(HEX_DEBUG) $(ASM)
	@echo "Clean completed"
