# Jacob Chisholm
# Makefile for compiling and uploading bare metal code
# to STMF446 with BlackMagic Probe + GDB
CFLAGS  ?=  -W -Wall -Wextra -Werror -Wundef -Wshadow -Wdouble-promotion \
            -Wformat-truncation -fno-common -Wconversion \
            -g3 -Os -ffunction-sections -fdata-sections -I. \
            -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 $(EXTRA_CFLAGS)
LDFLAGS ?= -Tlink.ld -nostartfiles -nostdlib --specs nano.specs -lc -lgcc -Wl,--gc-sections -Wl,-Map=$@.map
SOURCES = main.c startup.c syscalls.c
COMPORT = COM5

build: firmware.elf

firmware.elf: $(SOURCES)
	arm-none-eabi-gcc $(SOURCES) $(CFLAGS) $(LDFLAGS) -o $@

firmware.bin: firmware.elf
	arm-none-eabi-objcopy -O binary $< $@

# must run "run" command or restart target to start program
flash:
	gdb -ex 'target extended-remote $(COMPORT)' \
	-ex 'set mem inaccessible-by-default off' \
	-ex 'monitor swdp' \
	-ex 'attach 1' \
	-ex 'load firmware.elf' \
	-q

clean:
	rm firmware.*