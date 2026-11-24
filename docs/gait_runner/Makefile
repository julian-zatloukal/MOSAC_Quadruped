# gait_runner: the Programmed_Yaw walk on the real robot (NUCLEO-F412ZG)
#
# Normally run through Docker:  ./build.sh [target] [SETTING=value ...]
#
#   make                         firmware: build/gait_runner.elf / .hex / .bin
#   make RUN_MODE=RUN_STAND      any scalar setting of Src/config.h, see OVERRIDES
#   make test                    C walk against the Python walk (needs python3 + numpy)
#   make report                  the board's dry run of the walk, on the host
#   make params                  regenerate Src/gait_params.h from the Webots JSON files
#   make flash                   program the board over ST-LINK (OpenOCD on the host, not in Docker)
#   make clean

TARGET    := gait_runner
BUILD_DIR := build

# HAL + CMSIS and the pot tables are shared with the main firmware project.
DRIVERS   ?= ../../quadruped_control_mcu/Drivers
HAL_DIR   := $(DRIVERS)/STM32F4xx_HAL_Driver
CALIB_DIR ?= ../../quadruped_control_mcu/Core/Inc

HAL_MODULES := hal hal_cortex hal_rcc hal_rcc_ex hal_gpio hal_pwr hal_pwr_ex \
               hal_flash hal_flash_ex hal_dma hal_dma_ex hal_tim hal_tim_ex hal_uart \
               hal_adc hal_adc_ex

CORE_SOURCES := Src/gait.c Src/pose.c Src/plan.c Src/robot.c      # no hardware access: also built on the host
APP_SOURCES  := Src/main.c Src/servo.c Src/pots.c Src/console.c Src/hw_stm32.c $(CORE_SOURCES)
C_SOURCES    := $(APP_SOURCES) Src/system_stm32f4xx.c \
                $(patsubst %,$(HAL_DIR)/Src/stm32f4xx_%.c,$(HAL_MODULES))
ASM_SOURCES  := startup_stm32f412zx.s
LDSCRIPT     := STM32F412XX_FLASH.ld

PREFIX  := arm-none-eabi-
CC      := $(PREFIX)gcc
OBJCOPY := $(PREFIX)objcopy
SIZE    := $(PREFIX)size
HOSTCC  ?= cc
PYTHON  ?= python3
OPENOCD ?= openocd
OPENOCD_CFG := -f interface/stlink.cfg -f target/stm32f4x.cfg

# Settings given on the command line become -D overrides of Src/config.h.
OVERRIDES := RUN_MODE SPEED_SCALE STRIDE WALK_CYCLES POWERUP_WIGGLE
CFG_DEFS  := $(foreach v,$(OVERRIDES),$(if $($(v)),-D$(v)=$($(v))))

MCU     := -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
DEFS    := -DUSE_HAL_DRIVER -DSTM32F412Zx $(CFG_DEFS)
INCS    := -IInc -ISrc -I$(HAL_DIR)/Inc -I$(DRIVERS)/CMSIS/Device/ST/STM32F4xx/Include -I$(DRIVERS)/CMSIS/Include \
           -I$(CALIB_DIR)
WARN    := -Wall -Wextra -Wno-unused-parameter
CFLAGS  := $(MCU) $(DEFS) $(INCS) -std=gnu11 -O2 -g $(WARN) -fdata-sections -ffunction-sections -MMD -MP
ASFLAGS := $(MCU) -x assembler-with-cpp -g
LDFLAGS := $(MCU) -T$(LDSCRIPT) -specs=nano.specs -specs=nosys.specs -Wl,--gc-sections -lm \
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

# ------------------------------------------------------------------ host builds

HOST_CFLAGS := -std=c11 -O2 $(WARN) -ISrc $(CFG_DEFS)

$(BUILD_DIR)/host/gait_host: tools/gait_host.c Src/gait.c Src/gait.h Src/gait_params.h $(BUILD_DIR)/.cfg
	@mkdir -p $(dir $@)
	$(HOSTCC) $(HOST_CFLAGS) tools/gait_host.c Src/gait.c -lm -o $@

$(BUILD_DIR)/host/plan_host: tools/plan_host.c $(CORE_SOURCES) Src/*.h $(BUILD_DIR)/.cfg
	@mkdir -p $(dir $@)
	$(HOSTCC) $(HOST_CFLAGS) tools/plan_host.c $(CORE_SOURCES) -lm -o $@

test: $(BUILD_DIR)/host/gait_host
	$(PYTHON) tools/gen_params.py --check
	$(PYTHON) tools/test_gait.py $<

report: $(BUILD_DIR)/host/plan_host
	@$<

params:
	$(PYTHON) tools/gen_params.py

flash: $(BUILD_DIR)/$(TARGET).elf
	$(OPENOCD) $(OPENOCD_CFG) -c "program $< verify reset exit"

clean:
	rm -rf $(BUILD_DIR)

-include $(wildcard $(BUILD_DIR)/*.d)

.PHONY: all test report params flash clean FORCE
