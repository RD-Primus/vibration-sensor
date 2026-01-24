################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/custom_bus.c \
../Core/Src/main.c \
../Core/Src/stm32wlxx_hal_msp.c \
../Core/Src/stm32wlxx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c 

OBJS += \
./Core/Src/custom_bus.o \
./Core/Src/main.o \
./Core/Src/stm32wlxx_hal_msp.o \
./Core/Src/stm32wlxx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o 

C_DEPS += \
./Core/Src/custom_bus.d \
./Core/Src/main.d \
./Core/Src/stm32wlxx_hal_msp.d \
./Core/Src/stm32wlxx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DCORE_CM4 -DUSE_HAL_DRIVER -DSTM32WL55xx -c -I"C:/Users/User/STM32Cube/Repository/Packs/STMicroelectronics/X-CUBE-MEMS1/11.2.0/Middlewares/ST/STM32_MotionTL2_Library/Inc" -I"C:/Users/User/STM32Cube/Repository/Packs/STMicroelectronics/X-CUBE-MEMS1/11.2.0/Middlewares/ST/STM32_MotionEC_Library/Inc" -I../MEMS/App -I../MEMS/Target -I../Core/Inc -I../../Drivers/BSP/custom -I../../Drivers/BSP/Components/lsm6dso -I../../Drivers/BSP/Components/lis2dw12 -I../../Drivers/BSP/Components/lis2mdl -I../../Drivers/BSP/Components/hts221 -I../../Drivers/BSP/Components/lps22hh -I../../Drivers/BSP/Components/stts751 -I../../Drivers/BSP/IKS01A3 -I../../Drivers/BSP/Components/Common -I../../Drivers/STM32WLxx_HAL_Driver/Inc -I../../Drivers/STM32WLxx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32WLxx/Include -I../../Drivers/CMSIS/Include -Oz -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/custom_bus.cyclo ./Core/Src/custom_bus.d ./Core/Src/custom_bus.o ./Core/Src/custom_bus.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/stm32wlxx_hal_msp.cyclo ./Core/Src/stm32wlxx_hal_msp.d ./Core/Src/stm32wlxx_hal_msp.o ./Core/Src/stm32wlxx_hal_msp.su ./Core/Src/stm32wlxx_it.cyclo ./Core/Src/stm32wlxx_it.d ./Core/Src/stm32wlxx_it.o ./Core/Src/stm32wlxx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su

.PHONY: clean-Core-2f-Src

