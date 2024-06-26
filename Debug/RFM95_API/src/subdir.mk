################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../RFM95_API/src/RFM95.c 

OBJS += \
./RFM95_API/src/RFM95.o 

C_DEPS += \
./RFM95_API/src/RFM95.d 


# Each subdirectory must supply rules for building sources it contributes
RFM95_API/src/%.o RFM95_API/src/%.su RFM95_API/src/%.cyclo: ../RFM95_API/src/%.c RFM95_API/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/huang/STM32CubeIDE/workspace_1.14.0/F411CE_RFM95/RFM95_API/inc" -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-RFM95_API-2f-src

clean-RFM95_API-2f-src:
	-$(RM) ./RFM95_API/src/RFM95.cyclo ./RFM95_API/src/RFM95.d ./RFM95_API/src/RFM95.o ./RFM95_API/src/RFM95.su

.PHONY: clean-RFM95_API-2f-src

