################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/ST/STM32_USB_Device_Library/Class/CustomHID/Src/usbd_customhid.c 

OBJS += \
./Middlewares/ST/STM32_USB_Device_Library/Class/CustomHID/Src/usbd_customhid.o 

C_DEPS += \
./Middlewares/ST/STM32_USB_Device_Library/Class/CustomHID/Src/usbd_customhid.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/ST/STM32_USB_Device_Library/Class/CustomHID/Src/%.o: ../Middlewares/ST/STM32_USB_Device_Library/Class/CustomHID/Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m7 -mthumb -mfloat-abi=hard -mfpu=fpv5-d16 '-D__weak=__attribute__((weak))' '-D__packed="__attribute__((__packed__))"' -DUSE_HAL_DRIVER -DSTM32H753xx -I"/home/kbisland/MotherboardFirmware/Inc" -I"/home/kbisland/MotherboardFirmware/Drivers/STM32H7xx_HAL_Driver/Inc" -I"/home/kbisland/MotherboardFirmware/Drivers/STM32H7xx_HAL_Driver/Inc/Legacy" -I"/home/kbisland/MotherboardFirmware/Middlewares/ST/STM32_USB_Device_Library/Core/Inc" -I"/home/kbisland/MotherboardFirmware/Middlewares/ST/STM32_USB_Device_Library/Class/CustomHID/Inc" -I"/home/kbisland/MotherboardFirmware/Drivers/CMSIS/Device/ST/STM32H7xx/Include" -I"/home/kbisland/MotherboardFirmware/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


