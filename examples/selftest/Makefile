# Makefile — Selbsttest-Programme für den LPC1115-Emulator (Linux/macOS).
#
#   make            # alle Tests bauen (.elf + .hex)
#   make t3_timer_irq.hex
#   make clean
#
# Voraussetzung: arm-none-eabi-gcc im PATH.

CROSS   ?= arm-none-eabi-
CC       = $(CROSS)gcc
OBJCOPY  = $(CROSS)objcopy
SIZE     = $(CROSS)size

CFLAGS   = -mcpu=cortex-m0 -mthumb -Os -g \
           -ffunction-sections -fdata-sections \
           -Wall -Wextra \
           -nostartfiles -Wl,--gc-sections -T lpc1115.ld

STARTUP  = startup_lpc1115.c
TESTS   := $(patsubst %.c,%,$(wildcard t*.c))
HEX     := $(addsuffix .hex,$(TESTS))

all: $(HEX)

%.elf: %.c $(STARTUP) lpc1115.ld lpc1115.h
	$(CC) $(CFLAGS) $(STARTUP) $< -o $@
	$(SIZE) $@

%.hex: %.elf
	$(OBJCOPY) -O ihex $< $@

clean:
	rm -f *.elf *.hex

.PHONY: all clean
.PRECIOUS: %.elf
