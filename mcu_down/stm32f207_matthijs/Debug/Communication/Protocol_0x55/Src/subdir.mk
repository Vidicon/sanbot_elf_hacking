################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Communication/Protocol_0x55/Src/protocol_0x55.c 

OBJS += \
./Communication/Protocol_0x55/Src/protocol_0x55.o 

C_DEPS += \
./Communication/Protocol_0x55/Src/protocol_0x55.d 


# Each subdirectory must supply rules for building sources it contributes
Communication/Protocol_0x55/Src/protocol_0x55.o: ../Communication/Protocol_0x55/Src/protocol_0x55.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F207xx -DDEBUG -c -I../Core/Inc -I../Drivers/STM32F2xx_HAL_Driver/Inc -I../Drivers/STM32F2xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F2xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I.././Communication/Protocol_0x55/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Communication/Protocol_0x55/Src/protocol_0x55.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

