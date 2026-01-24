################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../MEMS/App/app_mems.c 

OBJS += \
./MEMS/App/app_mems.o 

C_DEPS += \
./MEMS/App/app_mems.d 


# Each subdirectory must supply rules for building sources it contributes
MEMS/App/%.o MEMS/App/%.su MEMS/App/%.cyclo: ../MEMS/App/%.c MEMS/App/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DCORE_CM4 -DUSE_HAL_DRIVER -DSTM32WL55xx -c -I"C:/Users/User/STM32Cube/Repository/Packs/STMicroelectronics/X-CUBE-MEMS1/11.2.0/Middlewares/ST/STM32_MotionTL2_Library/Inc" -I"C:/Users/User/STM32Cube/Repository/Packs/STMicroelectronics/X-CUBE-MEMS1/11.2.0/Middlewares/ST/STM32_MotionEC_Library/Inc" -I../MEMS/App -I../MEMS/Target -I../Core/Inc -I../../Drivers/BSP/custom -I../../Drivers/BSP/Components/lsm6dso -I../../Drivers/BSP/Components/lis2dw12 -I../../Drivers/BSP/Components/lis2mdl -I../../Drivers/BSP/Components/hts221 -I../../Drivers/BSP/Components/lps22hh -I../../Drivers/BSP/Components/stts751 -I../../Drivers/BSP/IKS01A3 -I../../Drivers/BSP/Components/Common -I../../Drivers/STM32WLxx_HAL_Driver/Inc -I../../Drivers/STM32WLxx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32WLxx/Include -I../../Drivers/CMSIS/Include -Oz -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-MEMS-2f-App

clean-MEMS-2f-App:
	-$(RM) ./MEMS/App/app_mems.cyclo ./MEMS/App/app_mems.d ./MEMS/App/app_mems.o ./MEMS/App/app_mems.su

.PHONY: clean-MEMS-2f-App

