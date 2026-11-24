# servo_test_leg: stand-alone servo test firmware (NUCLEO-F412ZG)
#
#   make            build build/servo_test_leg.elf / .hex / .bin
#   make flash      build, then program the board over ST-LINK (OpenOCD)
#   make restore    re-flash the original board firmware from ../firmware_backup
#   make clean

TARGET    := servo_test_leg
BUILD_DIR := build

# HAL + CMSIS are shared with the main firmware project (not copied here).
DRIVERS   ?= ../quadruped_control_mcu/Drivers
HAL_DIR   := $(DRIVERS)/STM32F4xx_HAL_Driver
# Servo pot calibration tables (calibration_ADC.h), also shared with the main project.
CALIB_DIR ?= ../quadruped_control_mcu/Core/Inc

HAL_MODULES := hal hal_cortex hal_rcc hal_rcc_ex hal_gpio hal_pwr hal_pwr_ex \
               hal_flash hal_flash_ex hal_dma hal_dma_ex hal_tim hal_tim_ex hal_uart \
               hal_adc hal_adc_ex

C_SOURCES   := Src/main.c Src/system_stm32f4xx.c \
               $(patsubst %,$(HAL_DIR)/Src/stm32f4xx_%.c,$(HAL_MODULES))
ASM_SOURCES := startup_stm32f412zx.s
LDSCRIPT    := STM32F412XX_FLASH.ld

PREFIX  := arm-none-eabi-
CC      := $(PREFIX)gcc
OBJCOPY := $(PREFIX)objcopy
SIZE    := $(PREFIX)size
OPENOCD ?= openocd
OPENOCD_CFG := -f interface/stlink.cfg -f target/stm32f4x.cfg
ORIGINAL_FW := ../firmware_backup/firmware_used_42496B.bin

MCU     := -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
DEFS    := -DUSE_HAL_DRIVER -DSTM32F412Zx
INCS    := -IInc -I$(HAL_DIR)/Inc -I$(DRIVERS)/CMSIS/Device/ST/STM32F4xx/Include -I$(DRIVERS)/CMSIS/Include \
           -I$(CALIB_DIR)
CFLAGS  := $(MCU) $(DEFS) $(INCS) -std=gnu11 -O2 -g -Wall -fdata-sections -ffunction-sections -MMD -MP
ASFLAGS := $(MCU) -x assembler-with-cpp -g
LDFLAGS := $(MCU) -T$(LDSCRIPT) -specs=nano.specs -Wl,--gc-sections \
           -Wl,-Map=$(BUILD_DIR)/$(TARGET).map -Wl,--print-memory-usage -Wl,--no-warn-rwx-segments

OBJECTS := $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o) $(ASM_SOURCES:.s=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(CC) -c $(ASFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) $(LDSCRIPT)
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SIZE) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf
	$(OBJCOPY) -O ihex $< $@

$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf
	$(OBJCOPY) -O binary -S $< $@

$(BUILD_DIR):
	mkdir -p $@

flash: $(BUILD_DIR)/$(TARGET).elf
	$(OPENOCD) $(OPENOCD_CFG) -c "program $< verify reset exit"

restore:
	$(OPENOCD) $(OPENOCD_CFG) -c "program $(ORIGINAL_FW) 0x08000000 verify reset exit"

clean:
	rm -rf $(BUILD_DIR)

-include $(wildcard $(BUILD_DIR)/*.d)

.PHONY: all flash restore clean
