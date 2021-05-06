# GCC makefile for ENNOID-BMS-SS firmware

SRCS  = ./Main/main.c
SRCS += ./Modules/src/modCAN.c
SRCS += ./Modules/src/modCommands.c
SRCS += ./Modules/src/modConfig.c
SRCS += ./Modules/src/modDelay.c
SRCS += ./Modules/src/modDisplay.c
SRCS += ./Modules/src/modEffect.c
SRCS += ./Modules/src/modFlash.c
SRCS += ./Modules/src/modOperationalState.c
SRCS += ./Modules/src/modPowerElectronics.c
SRCS += ./Modules/src/modPowerState.c
SRCS += ./Modules/src/modStateOfCharge.c
SRCS += ./Modules/src/modTerminal.c
SRCS += ./Modules/src/modUART.c

SRCS += ./Drivers/SWDrivers/Src/driverSWStorageManager.c
SRCS += ./Drivers/SWDrivers/Src/driverSWSSD1306.c
SRCS += ./Drivers/SWDrivers/Src/driverSWLTC6804.c
SRCS += ./Drivers/SWDrivers/Src/driverSWISL28022.c
SRCS += ./Drivers/SWDrivers/Src/driverSWADC128D818.c
SRCS += ./Drivers/SWDrivers/Src/driverSWUART2.c
SRCS += ./Drivers/SWDrivers/Src/driverSWSHT21.c
SRCS += ./Drivers/SWDrivers/Src/driverSWHTC1080.c

SRCS += ./Drivers/HWDrivers/Src/driverHWADC.c
SRCS += ./Drivers/HWDrivers/Src/driverHWEEPROM.c
SRCS += ./Drivers/HWDrivers/Src/driverHWI2C1.c
SRCS += ./Drivers/HWDrivers/Src/driverHWI2C2.c
SRCS += ./Drivers/HWDrivers/Src/driverHWPowerState.c
SRCS += ./Drivers/HWDrivers/Src/driverHWSPI1.c
SRCS += ./Drivers/HWDrivers/Src/driverHWSwitches.c
SRCS += ./Drivers/HWDrivers/Src/driverHWStatus.c
SRCS += ./Drivers/HWDrivers/Src/driverHWUART2.c

SRCS += ./Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal.c
SRCS += ./Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_adc.c
SRCS += ./Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_adc_ex.c
SRCS += ./Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_can.c
SRCS += ./Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_cortex.c
SRCS += ./Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_dma.c
SRCS += ./Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_flash.c
SRCS += ./Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_flash_ex.c
SRCS += ./Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_gpio.c
SRCS += ./Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_iwdg.c
SRCS += ./Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_i2c.c
SRCS += ./Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_i2c_ex.c
SRCS += ./Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_rcc.c
SRCS += ./Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_rcc_ex.c
SRCS += ./Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_spi.c
SRCS += ./Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_uart.c
SRCS += ./Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_uart_ex.c

SRCS += ./Device/stm32f3xx_it.c
SRCS += ./Device/stm32f3xx_hal_msp.c
SRCS += ./Drivers/CMSIS/Device/ST/STM32F3xx/Source/Templates/system_stm32f3xx.c

SRCS += ./Libraries/Scr/libBuffer.c
SRCS += ./Libraries/Scr/libCRC.c
SRCS += ./Libraries/Scr/libLogos.c
SRCS += ./Libraries/Scr/libGLCDFont.c
SRCS += ./Libraries/Scr/libGraphics.c
SRCS += ./Libraries/Scr/libPacket.c
SRCS += ./Libraries/Scr/libRingbuffer.c

# TODO: place object files in separate directory
OBJS = $(SRCS:.c=.o)

INCLUDE  = -I ./Main
INCLUDE += -I ./Drivers/CMSIS/Device/ST/STM32F3xx/Include
INCLUDE += -I ./Drivers/CMSIS/Include
INCLUDE += -I ./Drivers/HWDrivers/Inc
INCLUDE += -I ./Drivers/STM32F3xx_HAL_Driver/Inc
INCLUDE += -I ./Drivers/SWDrivers/Inc
INCLUDE += -I ./Libraries/Inc
INCLUDE += -I ./Modules/Inc
INCLUDE += -I ./CubeMX/Inc

LINKER_SCRIPT = ./GCC/EBMS-linker.ld
# STARTUP = ./GCC/EBMS-startup.c
STARTUP = ./GCC/startup_stm32f303xc.s

CC = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
OBJDUMP = arm-none-eabi-objdump

CFLAGS  = -std=gnu99 -g -Os -Wall
CFLAGS += -mlittle-endian -mthumb -mthumb-interwork
CFLAGS += -mcpu=cortex-m4 -fsingle-precision-constant
CFLAGS += -Wdouble-promotion
CFLAGS += -specs=nano.specs -specs=nosys.specs
CFLAGS += -mfpu=fpv4-sp-d16 -mfloat-abi=hard -march=armv7e-m
CFLAGS += -ffunction-sections -fdata-sections
# define board for compiler
CFLAGS += -D STM32F303xC
CFLAGS += -D USE_HAL_DRIVER

LINKER_FLAGS = -Wl,-Map=main.map -Wl,--gc-sections

OPENOCD_FLAGS = -f interface/stlink-v2.cfg -f target/stm32f3x.cfg

all: main.elf main.bin main.hex main-St.txt main-d.txt main-h.txt main-nm.txt

%.o: %.c
	$(CC) $(CFLAGS) $(INCLUDE) -c $< -o $@ -lm

# $(OBJS): | build/

main.elf: $(OBJS) $(STARTUP)
	$(CC) -T$(LINKER_SCRIPT) $(CFLAGS) $(LINKER_FLAGS) $^ -o $@

main.bin: main.elf
	$(OBJCOPY) -O binary $< $@

main.hex: main.elf
	$(OBJCOPY) -O ihex $< $@

main-St.txt: main.elf
	$(OBJDUMP) -St $< > $@

main-h.txt: main.elf
	$(OBJDUMP) -h $< > $@

main-d.txt: main.elf
	$(OBJDUMP) -d $< > $@

main-nm.txt: main.elf
	arm-none-eabi-nm $< | sort > $@

upload-stlink: main.elf
	openocd $(OPENOCD_FLAGS) -c "program main.elf reset exit"

upload-bin-stlink: main.bin
	openocd $(OPENOCD_FLAGS) -c "program main.bin reset verify exit 0x08000000"

upload-hex-stlink: main.hex
	openocd $(OPENOCD_FLAGS) -c "program main.hex reset verify exit"

OPENOCD_JLINK_FLAGS = -f interface/jlink.cfg -f GCC/jlink.cfg -f target/stm32f3x.cfg
upload: main.elf
	openocd $(OPENOCD_JLINK_FLAGS) -c "init" -c "reset init" -c "flash write_image erase main.elf" -c "reset" -c "shutdown"

upload-hex: main.hex
	openocd $(OPENOCD_JLINK_FLAGS) -c "init" -c "reset init" -c "flash write_image erase main.hex" -c "reset" -c "shutdown"

upload-bin: main.bin
	openocd $(OPENOCD_JLINK_FLAGS) -c "program main.bin reset verify exit 0x08000000"

connect:
	openocd $(OPENOCD_FLAGS)

debug:
	arm-none-eabi-gdb --eval-command="target remote localhost:3333" main.elf

clean:
	rm main.elf
	rm main.bin
	rm main.hex
	rm Main/main.o
	rm Modules/Src/*.o
	rm Libraries/Scr/*.o
	rm Drivers/HWDrivers/Src/*.o
	rm Drivers/SWDrivers/Src/*.o
	rm Drivers/STM32F3xx_HAL_Driver/Src/*.o
	rm Drivers/CMSIS/Device/ST/STM32F3xx/Source/Templates/*.o
	rm GCC/*.o