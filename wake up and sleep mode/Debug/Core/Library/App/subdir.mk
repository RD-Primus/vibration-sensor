################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Library/App/app_mems.c 

OBJS += \
./Core/Library/App/app_mems.o 

C_DEPS += \
./Core/Library/App/app_mems.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Library/App/%.o Core/Library/App/%.su Core/Library/App/%.cyclo: ../Core/Library/App/%.c Core/Library/App/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DCORE_CM4 -DUSE_HAL_DRIVER -DSTM32WL55xx -c -I../Core/Inc -I../MEMS/Target -I../Drivers/BSP/custom -I../Drivers/STM32WLxx_HAL_Driver/Inc -I../Drivers/STM32WLxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WLxx/Include -I../Drivers/CMSIS/Include -I../Drivers/BSP/Components/lsm6dso -I../Drivers/BSP/Components/lis2dw12 -I../Drivers/BSP/Components/lis2mdl -I../Drivers/BSP/Components/hts221 -I../Drivers/BSP/Components/lps22hh -I../Drivers/BSP/Components/stts751 -I../Drivers/BSP/IKS01A3 -I../Drivers/BSP/Components/Common -I"D:/vibration_sensor (1)/test_sleep_mode 1/Core/Library" -I"D:/vibration_sensor (1)/test_sleep_mode 1/Core/Library/App" -I../MEMS/App -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Library-2f-App

clean-Core-2f-Library-2f-App:
	-$(RM) ./Core/Library/App/app_mems.cyclo ./Core/Library/App/app_mems.d ./Core/Library/App/app_mems.o ./Core/Library/App/app_mems.su

.PHONY: clean-Core-2f-Library-2f-App

