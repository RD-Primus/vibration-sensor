################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Application/Primus/Interface/Src/Alarm_interface.c \
../Application/Primus/Interface/Src/LoRa_Support.c \
../Application/Primus/Interface/Src/NFC_07A1_Interface.c \
../Application/Primus/Interface/Src/Parameter.c \
../Application/Primus/Interface/Src/Sensor.c 

OBJS += \
./Application/Primus/Interface/Src/Alarm_interface.o \
./Application/Primus/Interface/Src/LoRa_Support.o \
./Application/Primus/Interface/Src/NFC_07A1_Interface.o \
./Application/Primus/Interface/Src/Parameter.o \
./Application/Primus/Interface/Src/Sensor.o 

C_DEPS += \
./Application/Primus/Interface/Src/Alarm_interface.d \
./Application/Primus/Interface/Src/LoRa_Support.d \
./Application/Primus/Interface/Src/NFC_07A1_Interface.d \
./Application/Primus/Interface/Src/Parameter.d \
./Application/Primus/Interface/Src/Sensor.d 


# Each subdirectory must supply rules for building sources it contributes
Application/Primus/Interface/Src/%.o Application/Primus/Interface/Src/%.su Application/Primus/Interface/Src/%.cyclo: ../Application/Primus/Interface/Src/%.c Application/Primus/Interface/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DCORE_CM4 -DUSE_HAL_DRIVER -DSTM32WL5Mxx -c -I../../Core/Inc -I../../LoRaWAN/App -I../../LoRaWAN/Target -I../../X-CUBE-MEMS1/Target -I../../NFC7 -I../../NFC7/Target -I../../Drivers/BSP/B-WL5M-SUBG1 -I../../Drivers/STM32WLxx_HAL_Driver/Inc -I../../Drivers/STM32WLxx_HAL_Driver/Inc/Legacy -I../../Utilities/trace/adv_trace -I../../Utilities/misc -I../../Utilities/sequencer -I../../Utilities/timer -I../../Utilities/lpm/tiny_lpm -I../../Middlewares/Third_Party/LoRaWAN/LmHandler/Packages -I../../Drivers/CMSIS/Device/ST/STM32WLxx/Include -I../../Middlewares/Third_Party/LoRaWAN/Crypto -I../../Middlewares/Third_Party/LoRaWAN/Mac/Region -I../../Middlewares/Third_Party/LoRaWAN/Mac -I../../Middlewares/Third_Party/LoRaWAN/LmHandler -I../../Middlewares/Third_Party/LoRaWAN/Utilities -I../../Middlewares/Third_Party/SubGHz_Phy -I../../Middlewares/Third_Party/SubGHz_Phy/stm32_radio_driver -I../../Drivers/CMSIS/Include -I../../Drivers/BSP/Components/st25dvxxkc -I../../Drivers/BSP/Components/stts22h -I../../Drivers/BSP/Components/ism330dhcx -I"D:/vibration test123/Vibration_LoRa/Vibration_LoRa/STM32CubeIDE/Application/Primus/Common/Library" -I"D:/vibration test123/Vibration_LoRa/Vibration_LoRa/STM32CubeIDE/Application/Primus/Common/Library_ISM330DHCX" -I"D:/vibration test123/Vibration_LoRa/Vibration_LoRa/STM32CubeIDE/Application/Primus/Common/Library_STTS22H" -I"D:/vibration test123/Vibration_LoRa/Vibration_LoRa/STM32CubeIDE/Application/Primus" -I"D:/vibration test123/Vibration_LoRa/Vibration_LoRa/STM32CubeIDE/Application/Primus/Interface/Inc" -I"D:/vibration test123/Vibration_LoRa/Vibration_LoRa/STM32CubeIDE/Application/Primus/Library/Inc" -Oz -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Application-2f-Primus-2f-Interface-2f-Src

clean-Application-2f-Primus-2f-Interface-2f-Src:
	-$(RM) ./Application/Primus/Interface/Src/Alarm_interface.cyclo ./Application/Primus/Interface/Src/Alarm_interface.d ./Application/Primus/Interface/Src/Alarm_interface.o ./Application/Primus/Interface/Src/Alarm_interface.su ./Application/Primus/Interface/Src/LoRa_Support.cyclo ./Application/Primus/Interface/Src/LoRa_Support.d ./Application/Primus/Interface/Src/LoRa_Support.o ./Application/Primus/Interface/Src/LoRa_Support.su ./Application/Primus/Interface/Src/NFC_07A1_Interface.cyclo ./Application/Primus/Interface/Src/NFC_07A1_Interface.d ./Application/Primus/Interface/Src/NFC_07A1_Interface.o ./Application/Primus/Interface/Src/NFC_07A1_Interface.su ./Application/Primus/Interface/Src/Parameter.cyclo ./Application/Primus/Interface/Src/Parameter.d ./Application/Primus/Interface/Src/Parameter.o ./Application/Primus/Interface/Src/Parameter.su ./Application/Primus/Interface/Src/Sensor.cyclo ./Application/Primus/Interface/Src/Sensor.d ./Application/Primus/Interface/Src/Sensor.o ./Application/Primus/Interface/Src/Sensor.su

.PHONY: clean-Application-2f-Primus-2f-Interface-2f-Src

