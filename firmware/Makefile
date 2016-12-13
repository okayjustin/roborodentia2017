BUILDDIR = build

DEVICE = libraries/CMSIS/Device/ST/STM32F4xx
CMSIS = libraries/CMSIS
LIBRARIES = libraries/STM32F4xx_HAL_Driver
UTILS = utils
DRIVERS = drivers
CONFIG = config
PERIPHERALS = peripherals
USB = peripherals/USB
SYSTEM = system

SOURCES += main.c

SOURCES += $(LIBRARIES)/source/stm32f4xx_hal_adc.c \
		   $(LIBRARIES)/source/stm32f4xx_hal_adc_ex.c \
		   $(LIBRARIES)/source/stm32f4xx_hal_cortex.c \
		   $(LIBRARIES)/source/stm32f4xx_hal_dac.c \
		   $(LIBRARIES)/source/stm32f4xx_hal_dac_ex.c \
		   $(LIBRARIES)/source/stm32f4xx_hal_dma.c \
		   $(LIBRARIES)/source/stm32f4xx_hal_gpio.c \
		   $(LIBRARIES)/source/stm32f4xx_hal_i2c.c \
		   $(LIBRARIES)/source/stm32f4xx_hal_pcd.c \
		   $(LIBRARIES)/source/stm32f4xx_hal_pcd_ex.c \
		   $(LIBRARIES)/source/stm32f4xx_hal_pwr_ex.c \
		   $(LIBRARIES)/source/stm32f4xx_hal_rcc.c \
		   $(LIBRARIES)/source/stm32f4xx_hal_spi.c \
		   $(LIBRARIES)/source/stm32f4xx_hal_tim.c \
		   $(LIBRARIES)/source/stm32f4xx_hal_tim_ex.c \
		   $(LIBRARIES)/source/stm32f4xx_hal_uart.c \
		   $(LIBRARIES)/source/stm32f4xx_hal_usart.c \
		   $(LIBRARIES)/source/stm32f4xx_ll_usb.c \
		   $(LIBRARIES)/source/stm32f4xx_hal.c

SOURCES += $(SYSTEM)/startup_stm32f446xx.s \
		   $(SYSTEM)/stm32f4xx_hal_msp.c \
		   $(SYSTEM)/stm32f4xx_it.c \
		   $(SYSTEM)/system_stm32f4xx.c \
		   $(SYSTEM)/stm32f4xx_nucleo_144.c 


#SOURCES += $(PERIPHERALS)/adc.c \
		   $(PERIPHERALS)/dac.c \
		   $(PERIPHERALS)/dma.c \
		   $(PERIPHERALS)/gpio.c \
		   $(PERIPHERALS)/i2c.c \
		   $(PERIPHERALS)/spi.c \
		   $(PERIPHERALS)/tim.c \
		   $(PERIPHERALS)/usart.c \
		   $(PERIPHERALS)/usb_otg.c

#SOURCES += $(USB)/usbd_cdc.c \
		   $(USB)/usbd_conf.c \
		   $(USB)/usbd_core.c \
		   $(USB)/usbd_ctlreq.c \
		   $(USB)/usbd_desc.c \
		   $(USB)/usbd_ioreq.c

#SOURCES += $(DRIVERS)/comms.c \
		   $(DRIVERS)/console.c \
		   $(DRIVERS)/fnr. \
		   $(DRIVERS)/kill.c \
		   $(DRIVERS)/led.c \
		   $(DRIVERS)/motorControl.c \
		   $(DRIVERS)/pwradc.c \
		   $(DRIVERS)/sevenSeg.c \
		   $(DRIVERS)/speedDAC.c \
		   $(DRIVERS)/steering.c \
		   $(DRIVERS)/encoder.c

#SOURCES += $(UTILS)/buffer8.c \
		   $(UTILS)/newlib_hooks.c \
		   $(UTILS)/timerCallback.c \
		   $(UTILS)/characterMapping.c \
		   $(UTILS)/crc8.c \
		   $(UTILS)/doubleBuffer.c

OBJECTS = $(addprefix $(BUILDDIR)/, $(addsuffix .o, $(basename $(SOURCES))))

INCLUDES += -I$(DEVICE)/include \
			-I$(LIBRARIES)/include \
			-I$(CMSIS)/include \
			-I$(USB)/include \
			-I$(SYSTEM) \
			-I$(CONFIG)\
			-I$(DRIVERS)\
			-I$(UTILS)\
			-I$(PERIPHERALS)\
			-I$(USB)\
			-I.

ELF = $(BUILDDIR)/program.elf
HEX = $(BUILDDIR)/program.hex
BIN = $(BUILDDIR)/program.bin
MAP = $(BUILDDIR)/program.map

CC = arm-none-eabi-gcc
LD = arm-none-eabi-gcc
AR = arm-none-eabi-ar
OBJCOPY = arm-none-eabi-objcopy

WARNINGS = -Wall -Werror -Wno-unused-function

# Removed -DUSE_STDPERIPH_DRIVER    
CFLAGS  = -O0 -g \
   -mcpu=cortex-m4 -mthumb -mlittle-endian \
   -mfloat-abi=hard -mfpu=fpv4-sp-d16 \
   $(INCLUDES) \
   -DSTM32F446xx -DUSE_USB_FS \
   $(WARNINGS)

LDSCRIPT = buildTools/STM32F446ZE_FLASH.ld
LDFLAGS += -T$(LDSCRIPT) -mthumb -mcpu=cortex-m4 -nostdlib -LbuildTools

LDLIBS += -lc -lm -lnosys

FLASH_SCRIPT = buildTools/flash.jlink

$(BIN): $(ELF)
	$(OBJCOPY) -O binary $< $@

$(HEX): $(ELF)
	$(OBJCOPY) -O ihex $< $@

$(ELF): $(OBJECTS)
	$(LD) $(LDFLAGS) -Wl,-Map=$(MAP) -o $@ $(OBJECTS) $(LDLIBS)
	arm-none-eabi-size $@

$(BUILDDIR)/%.o: %.c
	mkdir -p $(dir $@)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILDDIR)/%.o: %.s
	mkdir -p $(dir $@)
	$(CC) -c $(CFLAGS) $< -o $@

flash: $(BIN)
	JLinkExe -Device STM32F446RE -CommanderScript $(FLASH_SCRIPT)

clean:
	rm -rf build
