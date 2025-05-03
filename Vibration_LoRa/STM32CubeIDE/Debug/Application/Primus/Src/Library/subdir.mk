################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Application/Primus/Src/Library/Sensor.c 

OBJS += \
./Application/Primus/Src/Library/Sensor.o 

C_DEPS += \
./Application/Primus/Src/Library/Sensor.d 


# Each subdirectory must supply rules for building sources it contributes
Application/Primus/Src/Library/%.o Application/Primus/Src/Library/%.su Application/Primus/Src/Library/%.cyclo: ../Application/Primus/Src/Library/%.c Application/Primus/Src/Library/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DCORE_CM4 -DUSE_HAL_DRIVER -DSTM32WL5Mxx -c -I../../Core/Inc -I../../LoRaWAN/App -I../../LoRaWAN/Target -I../../X-CUBE-MEMS1/Target -I../../NFC7 -I../../NFC7/Target -I../../Drivers/BSP/B-WL5M-SUBG1 -I../../Drivers/STM32WLxx_HAL_Driver/Inc -I../../Drivers/STM32WLxx_HAL_Driver/Inc/Legacy -I../../Utilities/trace/adv_trace -I../../Utilities/misc -I../../Utilities/sequencer -I../../Utilities/timer -I../../Utilities/lpm/tiny_lpm -I../../Middlewares/Third_Party/LoRaWAN/LmHandler/Packages -I../../Drivers/CMSIS/Device/ST/STM32WLxx/Include -I../../Middlewares/Third_Party/LoRaWAN/Crypto -I../../Middlewares/Third_Party/LoRaWAN/Mac/Region -I../../Middlewares/Third_Party/LoRaWAN/Mac -I../../Middlewares/Third_Party/LoRaWAN/LmHandler -I../../Middlewares/Third_Party/LoRaWAN/Utilities -I../../Middlewares/Third_Party/SubGHz_Phy -I../../Middlewares/Third_Party/SubGHz_Phy/stm32_radio_driver -I../../Drivers/CMSIS/Include -I../../Drivers/BSP/Components/st25dvxxkc -I../../Drivers/BSP/Components/stts22h -I../../Drivers/BSP/Components/ism330dhcx -I"D:/STM32/STM32CubeIDE/workspace/Vibration_LoRa/STM32CubeIDE/Application/Primus/Inc" -I"D:/STM32/STM32CubeIDE/workspace/Vibration_LoRa/STM32CubeIDE/Application/Primus/Inc/Library" -I"D:/STM32/STM32CubeIDE/workspace/Vibration_LoRa/STM32CubeIDE/Application/Primus/Common/Library_ISM330DHCX" -I"D:/STM32/STM32CubeIDE/workspace/Vibration_LoRa/STM32CubeIDE/Application/Primus/Common/Library_STTS22H" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Application-2f-Primus-2f-Src-2f-Library

clean-Application-2f-Primus-2f-Src-2f-Library:
	-$(RM) ./Application/Primus/Src/Library/Sensor.cyclo ./Application/Primus/Src/Library/Sensor.d ./Application/Primus/Src/Library/Sensor.o ./Application/Primus/Src/Library/Sensor.su

.PHONY: clean-Application-2f-Primus-2f-Src-2f-Library

