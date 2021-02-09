# Additional atsame build rules

# Setup the toolchain
CROSS_PREFIX=arm-none-eabi-

dirs-y += src/atsame src/generic
dirs-$(CONFIG_MACH_SAME51) += lib/same51/same51a/gcc
dirs-$(CONFIG_MACH_SAME54) += lib/same54/same54a/gcc

MCU := $(shell echo $(CONFIG_MCU) | tr a-z A-Z)

CFLAGS-$(CONFIG_MACH_SAME51) += -mcpu=cortex-m4 -Ilib/same51/same51a/include
CFLAGS-$(CONFIG_MACH_SAME51) += -mfpu=fpv4-sp-d16 -mfloat-abi=hard
CFLAGS-$(CONFIG_MACH_SAME54) += -mcpu=cortex-m4 -Ilib/same54/same54a/include
CFLAGS-$(CONFIG_MACH_SAME54) += -mfpu=fpv4-sp-d16 -mfloat-abi=hard

CFLAGS += $(CFLAGS-y) -D__$(MCU)__ -mthumb -Ilib/cmsis-core

CFLAGS_klipper.elf += --specs=nano.specs --specs=nosys.specs
CFLAGS_klipper.elf += -T $(OUT)src/generic/armcm_link.ld
$(OUT)klipper.elf: $(OUT)src/generic/armcm_link.ld

# Add source files
src-y += atsame/main.c atsame/gpio.c generic/crc16_ccitt.c
src-y += generic/armcm_boot.c generic/armcm_irq.c generic/armcm_reset.c
src-$(CONFIG_USBSERIAL) += atsame/usbserial.c atsame/chipid.c generic/usb_cdc.c
src-$(CONFIG_SERIAL) += atsame/serial.c generic/serial_irq.c
src-$(CONFIG_HAVE_GPIO_ADC) += atsame/adc.c
src-$(CONFIG_HAVE_GPIO_I2C) += atsame/i2c.c
src-$(CONFIG_HAVE_GPIO_SPI) += atsame/spi.c
src-$(CONFIG_HAVE_SERCOM) += atsame/sercom.c
src-$(CONFIG_HAVE_GPIO_HARD_PWM) += atsame/hard_pwm.c
src-$(CONFIG_MACH_SAME51) += atsame/same5_watchdog.c
src-$(CONFIG_MACH_SAME51) += atsame/same5_clock.c generic/armcm_timer.c
src-$(CONFIG_MACH_SAME54) += atsame/same5_watchdog.c
src-$(CONFIG_MACH_SAME54) += atsame/same5_clock.c generic/armcm_timer.c

# Build the additional hex and bin output files
target-y += $(OUT)klipper.bin $(OUT)klipper.elf.hex

$(OUT)klipper.bin: $(OUT)klipper.elf
	@echo "  Creating bin file $@"
	$(Q)$(OBJCOPY) -O binary $< $@

$(OUT)klipper.elf.hex: $(OUT)klipper.elf
	@echo "  Creating hex file $@"
	$(Q)$(OBJCOPY) -j .text -j .relocate -O ihex $< $@

# Flash rules
lib/bossac/bin/bossac:
	@echo "  Building bossac"
	$(Q)make -C lib/bossac bin/bossac

BOSSAC_OFFSET-$(CONFIG_MACH_SAME51) := 0x4000
BOSSAC_OFFSET-$(CONFIG_MACH_SAME54) := 0x4000
flash: $(OUT)klipper.bin lib/bossac/bin/bossac
	@echo "  Flashing $< to $(FLASH_DEVICE)"
	$(Q)$(PYTHON) ./scripts/flash_usb.py -t $(CONFIG_MCU) -d "$(FLASH_DEVICE)" -s "$(BOSSAC_OFFSET-y)" $(OUT)klipper.bin
