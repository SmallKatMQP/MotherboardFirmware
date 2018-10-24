################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Src/SmartServoAPI/SmartServo.cpp 

OBJS += \
./Src/SmartServoAPI/SmartServo.o 

CPP_DEPS += \
./Src/SmartServoAPI/SmartServo.d 


# Each subdirectory must supply rules for building sources it contributes
Src/SmartServoAPI/%.o: ../Src/SmartServoAPI/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: MCU G++ Compiler'
	@echo $(PWD)
	arm-none-eabi-g++ -mcpu=cortex-m7 -mthumb -mfloat-abi=hard -mfpu=fpv5-d16 '-D__weak=__attribute__((weak))' '-D__packed="__attribute__((__packed__))"' -DUSE_HAL_DRIVER -DSTM32H753xx -I"/home/kbisland/MotherboardFirmware/Inc" -I"/home/kbisland/MotherboardFirmware/Drivers/STM32H7xx_HAL_Driver/Inc" -I"/home/kbisland/MotherboardFirmware/Drivers/STM32H7xx_HAL_Driver/Inc/Legacy" -I"/home/kbisland/MotherboardFirmware/Middlewares/ST/STM32_USB_Device_Library/Core/Inc" -I"/home/kbisland/MotherboardFirmware/Middlewares/ST/STM32_USB_Device_Library/Class/CustomHID/Inc" -I"/home/kbisland/MotherboardFirmware/Drivers/CMSIS/Device/ST/STM32H7xx/Include" -I"/home/kbisland/MotherboardFirmware/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fno-exceptions -fno-rtti -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


