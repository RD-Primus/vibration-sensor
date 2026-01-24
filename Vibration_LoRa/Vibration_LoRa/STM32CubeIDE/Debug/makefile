################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Utilities/subdir.mk
-include Middlewares/SubGHz_Phy/subdir.mk
-include Middlewares/LoRaWAN/subdir.mk
-include Drivers/STM32WLxx_HAL_Driver/subdir.mk
-include Drivers/CMSIS/subdir.mk
-include Drivers/BSP/Components/subdir.mk
-include Drivers/BSP/B-WL5M-SUBG1/subdir.mk
-include Application/User/Startup/subdir.mk
-include Application/User/NFC7/subdir.mk
-include Application/User/LoRaWAN/Target/subdir.mk
-include Application/User/LoRaWAN/App/subdir.mk
-include Application/User/Core/subdir.mk
-include Application/Primus/Library/Src/subdir.mk
-include Application/Primus/Interface/Src/subdir.mk
-include Application/Primus/Common/Library_STTS22H/subdir.mk
-include Application/Primus/Common/Library_ISM330DHCX/subdir.mk
-include Application/Primus/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := Vibration_LoRa
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
Vibration_LoRa.elf \

MAP_FILES += \
Vibration_LoRa.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
Vibration_LoRa.list \

OBJCOPY_HEX += \
Vibration_LoRa.hex \


# All Target
all: main-build

# Main-build Target
main-build: Vibration_LoRa.elf secondary-outputs

# Tool invocations
Vibration_LoRa.elf Vibration_LoRa.map: $(OBJS) $(USER_OBJS) D:\vibration_sensor\Vibration_LoRa\Vibration_LoRa\STM32CubeIDE\STM32WL5MOCHX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "Vibration_LoRa.elf" @"objects.list" $(USER_OBJS) -Wl,--start-group $(LIBS) -Wl,--end-group -mcpu=cortex-m4 -T"D:\vibration_sensor\Vibration_LoRa\Vibration_LoRa\STM32CubeIDE\STM32WL5MOCHX_FLASH.ld" --specs=nosys.specs -Wl,-Map="Vibration_LoRa.map" -Wl,--gc-sections -static -L"D:\vibration_sensor\Vibration_LoRa\Vibration_LoRa\STM32CubeIDE\Application\Primus\Common\Library" --specs=nano.specs -mfloat-abi=soft -mthumb -u _printf_float -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

Vibration_LoRa.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "Vibration_LoRa.list"
	@echo 'Finished building: $@'
	@echo ' '

Vibration_LoRa.hex: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O ihex $(EXECUTABLES) "Vibration_LoRa.hex"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) Vibration_LoRa.elf Vibration_LoRa.hex Vibration_LoRa.list Vibration_LoRa.map default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_HEX)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
