################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/stm32h7xx_hal_msp.c \
../Src/system_stm32h7xx.c \
../Src/usb_device.c \
../Src/usbd_conf.c \
../Src/usbd_desc.c 

CPP_SRCS += \
../Src/main.cpp \
../Src/stm32h7xx_it.cpp \
../Src/usbd_custom_hid_if.cpp 

OBJS += \
./Src/main.o \
./Src/stm32h7xx_hal_msp.o \
./Src/stm32h7xx_it.o \
./Src/system_stm32h7xx.o \
./Src/usb_device.o \
./Src/usbd_conf.o \
./Src/usbd_custom_hid_if.o \
./Src/usbd_desc.o 

C_DEPS += \
./Src/stm32h7xx_hal_msp.d \
./Src/system_stm32h7xx.d \
./Src/usb_device.d \
./Src/usbd_conf.d \
./Src/usbd_desc.d 

CPP_DEPS += \
./Src/main.d \
./Src/stm32h7xx_it.d \
./Src/usbd_custom_hid_if.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: MCU G++ Compiler'
	@echo $(PWD)
	arm-none-eabi-g++ -mcpu=cortex-m7 -mthumb -mfloat-abi=hard -mfpu=fpv5-d16 '-D__weak=__attribute__((weak))' '-D__packed="__attribute__((__packed__))"' -DUSE_HAL_DRIVER -DSTM32H753xx -I"/home/kbisland/MotherboardFirmware/Inc" -I"/home/kbisland/MotherboardFirmware/Drivers/STM32H7xx_HAL_Driver/Inc" -I"/home/kbisland/MotherboardFirmware/Drivers/STM32H7xx_HAL_Driver/Inc/Legacy" -I"/home/kbisland/MotherboardFirmware/Middlewares/ST/STM32_USB_Device_Library/Core/Inc" -I"/home/kbisland/MotherboardFirmware/Middlewares/ST/STM32_USB_Device_Library/Class/CustomHID/Inc" -I"/home/kbisland/MotherboardFirmware/Drivers/CMSIS/Device/ST/STM32H7xx/Include" -I"/home/kbisland/MotherboardFirmware/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fno-exceptions -fno-rtti -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m7 -mthumb -mfloat-abi=hard -mfpu=fpv5-d16 '-D__weak=__attribute__((weak))' '-D__packed="__attribute__((__packed__))"' -DUSE_HAL_DRIVER -DSTM32H753xx -I"/home/kbisland/MotherboardFirmware/Inc" -I"/home/kbisland/MotherboardFirmware/Drivers/STM32H7xx_HAL_Driver/Inc" -I"/home/kbisland/MotherboardFirmware/Drivers/STM32H7xx_HAL_Driver/Inc/Legacy" -I"/home/kbisland/MotherboardFirmware/Middlewares/ST/STM32_USB_Device_Library/Core/Inc" -I"/home/kbisland/MotherboardFirmware/Middlewares/ST/STM32_USB_Device_Library/Class/CustomHID/Inc" -I"/home/kbisland/MotherboardFirmware/Drivers/CMSIS/Device/ST/STM32H7xx/Include" -I"/home/kbisland/MotherboardFirmware/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


