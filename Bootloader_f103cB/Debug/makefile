################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F1xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
Bootloader_f103cB.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
Bootloader_f103cB.list \

OBJCOPY_HEX += \
Bootloader_f103cB.hex \

OBJCOPY_BIN += \
Bootloader_f103cB.bin \


# All Target
all: Bootloader_f103cB.elf secondary-outputs

# Tool invocations
Bootloader_f103cB.elf: $(OBJS) $(USER_OBJS) C:\Users\ME\STM32CubeIDE\workspace_1.2.0\Bootloader_f103cB\STM32F103CBTX_FLASH.ld
	arm-none-eabi-gcc -o "Bootloader_f103cB.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"C:\Users\ME\STM32CubeIDE\workspace_1.2.0\Bootloader_f103cB\STM32F103CBTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="Bootloader_f103cB.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

Bootloader_f103cB.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "Bootloader_f103cB.list"
	@echo 'Finished building: $@'
	@echo ' '

Bootloader_f103cB.hex: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O ihex $(EXECUTABLES) "Bootloader_f103cB.hex"
	@echo 'Finished building: $@'
	@echo ' '

Bootloader_f103cB.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "Bootloader_f103cB.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_HEX) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
