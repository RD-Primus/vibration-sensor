################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
D:/vibration\ test123/Vibration_LoRa/Vibration_LoRa/Drivers/BSP/B-WL5M-SUBG1/b_wl5m_subg1_bus.c 

OBJS += \
./Drivers/BSP/B-WL5M-SUBG1/b_wl5m_subg1_bus.o 

C_DEPS += \
./Drivers/BSP/B-WL5M-SUBG1/b_wl5m_subg1_bus.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/B-WL5M-SUBG1/b_wl5m_subg1_bus.o: D:/vibration\ test123/Vibration_LoRa/Vibration_LoRa/Drivers/BSP/B-WL5M-SUBG1/b_wl5m_subg1_bus.c Drivers/BSP/B-WL5M-SUBG1/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DCORE_CM4 -DUSE_HAL_DRIVER -DSTM32WL5Mxx -c -I../../Core/Inc -I../../LoRaWAN/App -I../../LoRaWAN/Target -I../../X-CUBE-MEMS1/Target -I../../NFC7 -I../../NFC7/Target -I../../Drivers/BSP/B-WL5M-SUBG1 -I../../Drivers/STM32WLxx_HAL_Driver/Inc -I../../Drivers/STM32WLxx_HAL_Driver/Inc/Legacy -I../../Utilities/trace/adv_trace -I../../Utilities/misc -I../../Utilities/sequencer -I../../Utilities/timer -I../../Utilities/lpm/tiny_lpm -I../../Middlewares/Third_Party/LoRaWAN/LmHandler/Packages -I../../Drivers/CMSIS/Device/ST/STM32WLxx/Include -I../../Middlewares/Third_Party/LoRaWAN/Crypto -I../../Middlewares/Third_Party/LoRaWAN/Mac/Region -I../../Middlewares/Third_Party/LoRaWAN/Mac -I../../Middlewares/Third_Party/LoRaWAN/LmHandler -I../../Middlewares/Third_Party/LoRaWAN/Utilities -I../../Middlewares/Third_Party/SubGHz_Phy -I../../Middlewares/Third_Party/SubGHz_Phy/stm32_radio_driver -I../../Drivers/CMSIS/Include -I../../Drivers/BSP/Components/st25dvxxkc -I../../Drivers/BSP/Components/stts22h -I../../Drivers/BSP/Components/ism330dhcx -I"D:/vibration test123/Vibration_LoRa/Vibration_LoRa/STM32CubeIDE/Application/Primus/Common/Library" -I"D:/vibration test123/Vibration_LoRa/Vibration_LoRa/STM32CubeIDE/Application/Primus/Common/Library_ISM330DHCX" -I"D:/vibration test123/Vibration_LoRa/Vibration_LoRa/STM32CubeIDE/Application/Primus/Common/Library_STTS22H" -I"D:/vibration test123/Vibration_LoRa/Vibration_LoRa/STM32CubeIDE/Application/Primus" -I"D:/vibration test123/Vibration_LoRa/Vibration_LoRa/STM32CubeIDE/Application/Primus/Interface/Inc" -I"D:/vibration test123/Vibration_LoRa/Vibration_LoRa/STM32CubeIDE/Application/Primus/Library/Inc" -Oz -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"Drivers/BSP/B-WL5M-SUBG1/b_wl5m_subg1_bus.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-BSP-2f-B-2d-WL5M-2d-SUBG1

clean-Drivers-2f-BSP-2f-B-2d-WL5M-2d-SUBG1:
	-$(RM) ./Drivers/BSP/B-WL5M-SUBG1/b_wl5m_subg1_bus.cyclo ./Drivers/BSP/B-WL5M-SUBG1/b_wl5m_subg1_bus.d ./Drivers/BSP/B-WL5M-SUBG1/b_wl5m_subg1_bus.o ./Drivers/BSP/B-WL5M-SUBG1/b_wl5m_subg1_bus.su

.PHONY: clean-Drivers-2f-BSP-2f-B-2d-WL5M-2d-SUBG1

