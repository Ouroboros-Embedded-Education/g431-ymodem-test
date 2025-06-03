################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../libs/ymodem/ymodem.c 

OBJS += \
./libs/ymodem/ymodem.o 

C_DEPS += \
./libs/ymodem/ymodem.d 


# Each subdirectory must supply rules for building sources it contributes
libs/ymodem/%.o libs/ymodem/%.su libs/ymodem/%.cyclo: ../libs/ymodem/%.c libs/ymodem/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I../USB_Device/App -I../USB_Device/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I"/media/pablo/Seagate Portable Drive/Backup Notebook/Ouroboros Embedded Education/YouTube/YT - Firmware/ymodem-test/libs/ymodem" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-libs-2f-ymodem

clean-libs-2f-ymodem:
	-$(RM) ./libs/ymodem/ymodem.cyclo ./libs/ymodem/ymodem.d ./libs/ymodem/ymodem.o ./libs/ymodem/ymodem.su

.PHONY: clean-libs-2f-ymodem

