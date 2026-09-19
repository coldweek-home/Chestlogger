PROJECT := ChestLogger
MCU := attiny85
F_CPU := 1000000UL
CONFIG ?= Debug

ATMEL_TOOLCHAIN ?= C:/Program Files (x86)/Atmel/Studio/7.0/toolchain/avr8/avr8-gnu-toolchain
DFP_DIR ?= C:/Program Files (x86)/Atmel/Studio/7.0/Packs/atmel/ATtiny_DFP/1.10.348
AVRDUDE ?= avrdude

CC := "$(ATMEL_TOOLCHAIN)/bin/avr-gcc.exe"
OBJCOPY := "$(ATMEL_TOOLCHAIN)/bin/avr-objcopy.exe"
OBJDUMP := "$(ATMEL_TOOLCHAIN)/bin/avr-objdump.exe"
SIZE := "$(ATMEL_TOOLCHAIN)/bin/avr-size.exe"

BUILD_DIR := build/$(CONFIG)
SOURCES := main.c oledm_i2c.c sh1106.c ssd1306.c terminus8x16.c terminus8x16_var1.c text.c twi_attiny.c
OBJECTS := $(SOURCES:%.c=$(BUILD_DIR)/%.o)
DEPS := $(OBJECTS:.o=.d)
ELF := $(BUILD_DIR)/$(PROJECT).elf
HEX := $(BUILD_DIR)/$(PROJECT).hex

COMMON_CFLAGS := -x c -std=gnu99 -mmcu=$(MCU) -B "$(DFP_DIR)/gcc/dev/$(MCU)" -DF_CPU=$(F_CPU) -DOLED_SSD1306 -funsigned-char -funsigned-bitfields -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -I"$(DFP_DIR)/include"
ifeq ($(CONFIG),Release)
CONFIG_CFLAGS := -Os -DNDEBUG
else
CONFIG_CFLAGS := -Og -g2 -DDEBUG
endif
CFLAGS := $(COMMON_CFLAGS) $(CONFIG_CFLAGS)
LDFLAGS := -mmcu=$(MCU) -B "$(DFP_DIR)/gcc/dev/$(MCU)" -Wl,--gc-sections -Wl,-Map="$(BUILD_DIR)/$(PROJECT).map"

.PHONY: all clean flash size

all: $(HEX) $(BUILD_DIR)/$(PROJECT).eep $(BUILD_DIR)/$(PROJECT).lss $(BUILD_DIR)/$(PROJECT).srec size

$(BUILD_DIR):
	@if not exist "$(BUILD_DIR)" mkdir "$(BUILD_DIR)"

$(BUILD_DIR)/%.o: %.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -MD -MP -MF "$(@:.o=.d)" -c "$<" -o "$@"

$(ELF): $(OBJECTS)
	$(CC) $(LDFLAGS) -o "$@" $(OBJECTS) -lm

$(HEX): $(ELF)
	$(OBJCOPY) -O ihex -R .eeprom -R .fuse -R .lock -R .signature -R .user_signatures "$<" "$@"

$(BUILD_DIR)/$(PROJECT).eep: $(ELF)
	$(OBJCOPY) -j .eeprom --set-section-flags=.eeprom=alloc,load --change-section-lma .eeprom=0 --no-change-warnings -O ihex "$<" "$@" || exit 0

$(BUILD_DIR)/$(PROJECT).lss: $(ELF)
	$(OBJDUMP) -h -S "$<" > "$@"

$(BUILD_DIR)/$(PROJECT).srec: $(ELF)
	$(OBJCOPY) -O srec -R .eeprom -R .fuse -R .lock -R .signature -R .user_signatures "$<" "$@"

size: $(ELF)
	$(SIZE) "$<"

flash: $(HEX)
	$(AVRDUDE) -c usbasp -p t85 -U flash:w:"$(HEX)":i -B $(if $(filter Release,$(CONFIG)),2,4)

clean:
	@if exist "build" rmdir /s /q "build"

-include $(DEPS)