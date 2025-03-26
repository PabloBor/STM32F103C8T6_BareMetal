# Build tools
CC=arm-none-eabi-gcc
OBJCOPY=arm-none-eabi-objcopy

# Compiler and linker flags
CFLAGS=-mcpu=cortex-m3 -mthumb -O2
LDFLAGS=-Tstartup/stm32f103c8.ld -nostartfiles -Wl,--gc-sections -Xlinker -Map=$(BUILD_DIR)/firmware.map

# Directories
BUILD_DIR=build
SRC_DIR=src
STARTUP_DIR=startup

# Generated files
OBJ=$(BUILD_DIR)/startup.o $(BUILD_DIR)/main.o
ELF=$(BUILD_DIR)/firmware.elf
BIN=$(BUILD_DIR)/firmware.bin

all: $(BUILD_DIR) $(ELF) $(BIN)

$(BUILD_DIR):
	mkdir $(BUILD_DIR)

$(BUILD_DIR)/startup.o: $(STARTUP_DIR)/startup.s | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/main.o: main.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(ELF): $(OBJ)
	$(CC) $(LDFLAGS) $^ -o $@

$(BIN): $(ELF)
	$(OBJCOPY) -O binary $< $@

flash: $(BIN)
	openocd -f interface/stlink.cfg -f target/stm32f1x.cfg -c "program $(BIN) 0x08000000 verify reset exit"

clean:
	del /q $(BUILD_DIR)\*.*
	