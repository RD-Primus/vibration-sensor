################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
D:/vibration\ test123/Vibration_LoRa/Vibration_LoRa/LoRaWAN/Target/radio_board_if.c 

OBJS += \
./Application/User/LoRaWAN/Target/radio_board_if.o 

C_DEPS += \
./Application/User/LoRaWAN/Target/radio_board_if.d 


# Each subdirectory must supply rules for building sources it contributes
Application/User/LoRaWAN/Target/radio_board_if.o: D:/vibration\ test123/Vibration_LoRa/Vibration_LoRa/LoRaWAN/Target/radio_board_if.c Application/User/LoRaWAN/Target/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DCORE_CM4 -DUSE_HAL_DRIVER -DSTM32WL5Mxx -c -I../../Core/Inc -I../../LoRaWAN/App -I../../LoRaWAN/Target -I../../X-CUBE-MEMS1/Target -I../../NFC7 -I../../NFC7/Target -I../../Drivers/BSP/B-WL5M-SUBG1 -I../../Drivers/STM32WLxx_HAL_Driver/Inc -I../../Drivers/STM32WLxx_HAL_Driver/Inc/Legacy -I../../Utilities/trace/adv_trace -I../../Utilities/misc -I../../Utilities/sequencer -I../../Utilities/timer -I../../Utilities/lpm/tiny_lpm -I../../Middlewares/Third_Party/LoRaWAN/LmHandler/Packages -I../../Drivers/CMSIS/Device/ST/STM32WLxx/Include -I../../Middlewares/Third_Party/LoRaWAN/Crypto -I../../Middlewares/Third_Party/LoRaWAN/Mac/Region -I../../Middlewares/Third_Party/LoRaWAN/Mac -I../../Middlewares/Third_Party/LoRaWAN/LmHandler -I../../Middlewares/Third_Party/LoRaWAN/Utilities -I../../Middlewares/Third_Party/SubGHz_Phy -I../../Middlewares/Third_Party/SubGHz_Phy/stm32_radio_driver -I../../Drivers/CMSIS/Include -I../../Drivers/BSP/Components/st25dvxxkc -I../../Drivers/BSP/Components/stts22h -I../../Drivers/BSP/Components/ism330dhcx -I"D:/vibration test123/Vibration_LoRa/Vibration_LoRa/STM32CubeIDE/Application/Primus/Common/Library" -I"D:/vibration test123/Vibration_LoRa/Vibration_LoRa/STM32CubeIDE/Application/Primus/Common/Library_ISM330DHCX" -I"D:/vibration test123/Vibration_LoRa/Vibration_LoRa/STM32CubeIDE/Application/Primus/Common/Library_STTS22H" -I"D:/vibration test123/Vibration_LoRa/Vibration_LoRa/STM32CubeIDE/Application/Primus" -I"D:/vibration test123/Vibration_LoRa/Vibration_LoRa/STM32CubeIDE/Application/Primus/Interface/Inc" -I"D:/vibration test123/Vibration_LoRa/Vibration_LoRa/STM32CubeIDE/Application/Primus/Library/Inc" -Oz -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"Application/User/LoRaWAN/Target/radio_board_if.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Application-2f-User-2f-LoRaWAN-2f-Target

clean-Application-2f-User-2f-LoRaWAN-2f-Target:
	-$(RM) ./Application/User/LoRaWAN/Target/radio_board_if.cyclo ./Application/User/LoRaWAN/Target/radio_board_if.d ./Application/User/LoRaWAN/Target/radio_board_if.o ./Application/User/LoRaWAN/Target/radio_board_if.su

.PHONY: clean-Application-2f-User-2f-LoRaWAN-2f-Target

