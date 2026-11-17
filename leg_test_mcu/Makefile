##########################################################################################################################
# MOSAC leg test firmware (NUCLEO-F412ZG)
#   make                          build for the leg set in Core/Inc/leg_test_config.h
#   make TEST_LEG=LEG_BACK_LEFT   build for another leg (LEG_FRONT_RIGHT, LEG_FRONT_LEFT, LEG_BACK_RIGHT, LEG_BACK_LEFT)
#   make flash                    program the board through the ST-LINK (OpenOCD)
# HAL/CMSIS drivers, startup file, linker script and calibration tables come from ../quadruped_control_mcu
##########################################################################################################################

TARGET = leg_test
DEBUG = 1
OPT = -Og
BUILD_DIR = build

MAIN_FW = ../quadruped_control_mcu
DRIVERS = $(MAIN_FW)/Drivers

C_SOURCES =  \
Core/Src/main.c \
Core/Src/board.c \
Core/Src/joints.c \
Core/Src/calibration.c \
Core/Src/servo.c \
Core/Src/feedback.c \
Core/Src/motion.c \
Core/Src/sequence.c \
Core/Src/log.c \
Core/Src/stm32f4xx_it.c \
Core/Src/system_stm32f4xx.c \
$(DRIVERS)/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c \
$(DRIVERS)/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc.c \
$(DRIVERS)/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc_ex.c \
$(DRIVERS)/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c \
$(DRIVERS)/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c \
$(DRIVERS)/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c \
$(DRIVERS)/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_exti.c \
$(DRIVERS)/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c \
$(DRIVERS)/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c \
$(DRIVERS)/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c \
$(DRIVERS)/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c \
$(DRIVERS)/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c \
$(DRIVERS)/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c \
$(DRIVERS)/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c \
$(DRIVERS)/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c \
$(DRIVERS)/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c \
$(DRIVERS)/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c

ASM_SOURCES = $(MAIN_FW)/startup_stm32f412zx.s
LDSCRIPT = $(MAIN_FW)/STM32F412XX_FLASH.ld

PREFIX = arm-none-eabi-
ifdef GCC_PATH
CC = $(GCC_PATH)/$(PREFIX)gcc
AS = $(GCC_PATH)/$(PREFIX)gcc -x assembler-with-cpp
CP = $(GCC_PATH)/$(PREFIX)objcopy
SZ = $(GCC_PATH)/$(PREFIX)size
else
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size
endif
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S

MCU = -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard

C_DEFS = -DUSE_HAL_DRIVER -DSTM32F412Zx
ifdef TEST_LEG
C_DEFS += -DTEST_LEG=$(TEST_LEG)
endif

C_INCLUDES =  \
-ICore/Inc \
-I$(DRIVERS)/STM32F4xx_HAL_Driver/Inc \
-I$(DRIVERS)/STM32F4xx_HAL_Driver/Inc/Legacy \
-I$(DRIVERS)/CMSIS/Device/ST/STM32F4xx/Include \
-I$(DRIVERS)/CMSIS/Include

ASFLAGS = $(MCU) $(OPT) -Wall -fdata-sections -ffunction-sections
CFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections
ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
endif
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"

LIBS = -lc -lm -lnosys
LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections

all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin

OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

# Rebuild everything when the selected leg changes
$(BUILD_DIR)/leg.cfg: FORCE | $(BUILD_DIR)
	@echo "$(TEST_LEG)" | cmp -s - $@ || echo "$(TEST_LEG)" > $@

$(BUILD_DIR)/%.o: %.c Makefile $(BUILD_DIR)/leg.cfg | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@

$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@

$(BUILD_DIR):
	mkdir $@

flash: $(BUILD_DIR)/$(TARGET).elf
	openocd -f interface/stlink.cfg -f target/stm32f4x.cfg -c "program $< verify reset exit"

clean:
	-rm -fR $(BUILD_DIR)

FORCE:

.PHONY: all flash clean FORCE

-include $(wildcard $(BUILD_DIR)/*.d)
