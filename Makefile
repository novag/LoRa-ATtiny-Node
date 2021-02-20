MCU=attiny85
AVR_FREQ=8000000
PROGRAMMER_MCU=attiny85

AVRDUDE_PROGRAMMERID=usbasp

#########################

CC := avr-g++
OBJCOPY := avr-objcopy
SIZE := avr-size
AVRDUDE := avrdude
SRCDIR := src
BUILDDIR := build
TARGETDIR := bin
TARGET := $(TARGETDIR)/out
 
SRCEXT := cpp S
SOURCES := $(foreach EXT,$(SRCEXT),$(shell find $(SRCDIR) -type f -name *.$(EXT)))
OBJECTS := $(patsubst $(SRCDIR)/%,$(BUILDDIR)/%,$(addsuffix .o,$(basename $(SOURCES))))
CFLAGS := -Os -flto -fdata-sections -ffunction-sections -mcall-prologues # -g -Wall
LDFLAGS := -Wl,--gc-sections
LIB := -L lib
INC := -I include

ifdef DEBUG
	CFLAGS += -DDEBUG
endif

hex: $(TARGET).hex

eeprom: $(TARGET)_eeprom.hex

$(TARGET).hex: $(TARGET).elf
	@echo " $(OBJCOPY) -O ihex -j .data -j .text $(TARGET).elf $(TARGET).hex"; $(OBJCOPY) -O ihex -j .data -j .text $(TARGET).elf $(TARGET).hex

$(TARGET)_eeprom.hex: $(TARGET).elf
	@echo " $(OBJCOPY) -O ihex -j .eeprom --change-section-lma .eeprom=1 $(TARGET).elf $(TARGET)_eeprom.hex"; $(OBJCOPY) -O ihex -j .eeprom --change-section-lma .eeprom=1 $(TARGET).elf $(TARGET)_eeprom.hex

$(TARGET).elf: $(OBJECTS)
	@echo " Linking..."
	@echo " $(CC) $(LDFLAGS) -mmcu=$(MCU) $^ -o $(TARGET).elf $(LIB)"; $(CC) $(LDFLAGS) -mmcu=$(MCU) $^ -o $(TARGET).elf $(LIB)

$(BUILDDIR)/%.o: $(SRCDIR)/%.c
	@mkdir -p $(BUILDDIR) $(TARGETDIR)
	@echo " $(CC) $(CFLAGS) -mmcu=$(MCU) -DF_CPU=$(AVR_FREQ) $(INC) -c -o $@ $<"; $(CC) $(CFLAGS) -mmcu=$(MCU) -DF_CPU=$(AVR_FREQ) $(INC) -c -o $@ $<

$(BUILDDIR)/%.o: $(SRCDIR)/%.cpp
	@mkdir -p $(BUILDDIR) $(TARGETDIR)
	@echo " $(CC) $(CFLAGS) -mmcu=$(MCU) -DF_CPU=$(AVR_FREQ) $(INC) -c -o $@ $<"; $(CC) $(CFLAGS) -mmcu=$(MCU) -DF_CPU=$(AVR_FREQ) $(INC) -c -o $@ $<

$(BUILDDIR)/%.o: $(SRCDIR)/%.S
	@mkdir -p $(BUILDDIR) $(TARGETDIR)
	@echo " $(CC) $(CFLAGS) -mmcu=$(MCU) -DF_CPU=$(AVR_FREQ) $(INC) -c -o $@ $<"; $(CC) $(CFLAGS) -mmcu=$(MCU) -DF_CPU=$(AVR_FREQ) $(INC) -c -o $@ $<

size:
	$(SIZE) --mcu=$(MCU) -C $(TARGET).elf

program: hex
	$(AVRDUDE) -c $(AVRDUDE_PROGRAMMERID) \
	 -p $(PROGRAMMER_MCU) -B 5 -e \
	 -U flash:w:$(TARGET).hex:a

write_eeprom: eeprom
	$(AVRDUDE) -c $(AVRDUDE_PROGRAMMERID) \
	 -p $(PROGRAMMER_MCU) -B 5 -e \
	 -U eeprom:w:$(TARGET)_eeprom.hex

clean:
	@echo " Cleaning..."
	@echo " rm -r $(BUILDDIR) $(TARGETDIR)"; rm -r $(BUILDDIR) $(TARGETDIR)

.PHONY: clean
