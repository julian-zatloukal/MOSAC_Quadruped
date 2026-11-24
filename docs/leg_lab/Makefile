# leg_lab: single-leg test firmware (NUCLEO-F412ZG)
#
#   make                                  build with the settings in Src/config.h
#   make PROGRAM=PROGRAM_LIMITS TEST_LEG=LEG_BR    override program / leg (rebuilds)
#   make flash                            program the board over ST-LINK (OpenOCD); it then idles
#   make clean
#
# Any setting of Src/config.h can be overridden the same way, e.g. POTCAL_HORN_REMOVED=1.

TARGET    := leg_lab
BUILD_DIR := build

# HAL + CMSIS and the pot tables are shared with the main firmware project.
DRIVERS   ?= ../../quadruped_control_mcu/Drivers
HAL_DIR   := $(DRIVERS)/STM32F4xx_HAL_Driver
CALIB_DIR ?= ../../quadruped_control_mcu/Core/Inc

HAL_MODULES := hal hal_cortex hal_rcc hal_rcc_ex hal_gpio hal_pwr hal_pwr_ex \
               hal_flash hal_flash_ex hal_dma hal_dma_ex hal_tim hal_tim_ex hal_uart \
               hal_adc hal_adc_ex

APP_SOURCES := Src/main.c Src/robot.c Src/console.c Src/feedback.c Src/motion.c Src/guard.c \
               Src/prog_check.c Src/prog_potcal.c Src/prog_limits.c Src/hw_stm32.c
C_SOURCES   := $(APP_SOURCES) Src/system_stm32f4xx.c \
               $(patsubst %,$(HAL_DIR)/Src/stm32f4xx_%.c,$(HAL_MODULES))
ASM_SOURCES := startup_stm32f412zx.s
LDSCRIPT    := STM32F412XX_FLASH.ld

PREFIX  := arm-none-eabi-
CC      := $(PREFIX)gcc
OBJCOPY := $(PREFIX)objcopy
SIZE    := $(PREFIX)size
OPENOCD ?= openocd
OPENOCD_CFG := -f interface/stlink.cfg -f target/stm32f4x.cfg

# Settings passed on the command line become -D overrides of Src/config.h.
OVERRIDES := PROGRAM TEST_LEG CHECK_JOINTS POTCAL_JOINT POTCAL_HORN_REMOVED POTCAL_MOUNT_DEG \
             LIMITS_JOINTS LIMITS_REFINE LIMITS_MAP_FT LIMITS_MAP_FB MAP_FEMUR_STEP_DEG
CFG_DEFS  := $(foreach v,$(OVERRIDES),$(if $($(v)),-D$(v)=$($(v))))

MCU     := -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
DEFS    := -DUSE_HAL_DRIVER -DSTM32F412Zx $(CFG_DEFS)
INCS    := -IInc -ISrc -I$(HAL_DIR)/Inc -I$(DRIVERS)/CMSIS/Device/ST/STM32F4xx/Include -I$(DRIVERS)/CMSIS/Include \
           -I$(CALIB_DIR)
CFLAGS  := $(MCU) $(DEFS) $(INCS) -std=gnu11 -O2 -g -Wall -Wextra -Wno-unused-parameter -fdata-sections \
           -ffunction-sections -fsingle-precision-constant -MMD -MP
ASFLAGS := $(MCU) -x assembler-with-cpp -g
LDFLAGS := $(MCU) -T$(LDSCRIPT) -specs=nano.specs -Wl,--gc-sections \
           -Wl,-Map=$(BUILD_DIR)/$(TARGET).map -Wl,--print-memory-usage -Wl,--no-warn-rwx-segments

OBJECTS := $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o) $(ASM_SOURCES:.s=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin

# Rebuild everything when the overrides change.
$(BUILD_DIR)/.cfg: FORCE | $(BUILD_DIR)
	@echo '$(CFG_DEFS)' | cmp -s - $@ || echo '$(CFG_DEFS)' > $@

$(BUILD_DIR)/%.o: %.c Makefile $(BUILD_DIR)/.cfg | $(BUILD_DIR)
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

clean:
	rm -rf $(BUILD_DIR)

-include $(wildcard $(BUILD_DIR)/*.d)

.PHONY: all flash clean FORCE
