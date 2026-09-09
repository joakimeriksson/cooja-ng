# csim sniffer demo firmware for Renode (bare-metal Cortex-M4).
CC      = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
CFLAGS  = -mcpu=cortex-m4 -mthumb -Os -g -Wall -Wextra -ffreestanding \
          -fno-builtin -nostdlib
LDFLAGS = -T link.ld -Wl,--gc-sections -nostdlib
# libgcc only: the 64-bit divides in the timestamp printer need __aeabi_uldivmod.
LIBS    = -lgcc

all: sniffer.elf

sniffer.elf: sniffer.c startup.c link.ld ../csim_dev.h
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ sniffer.c startup.c $(LIBS)
	@arm-none-eabi-size $@

clean:
	rm -f sniffer.elf

.PHONY: all clean
