################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../foc_layer/adctovoltage_and_current.c \
../foc_layer/vabctoduty.c 

OBJS += \
./foc_layer/adctovoltage_and_current.o \
./foc_layer/vabctoduty.o 

C_DEPS += \
./foc_layer/adctovoltage_and_current.d \
./foc_layer/vabctoduty.d 


# Each subdirectory must supply rules for building sources it contributes
foc_layer/%.o foc_layer/%.su foc_layer/%.cyclo: ../foc_layer/%.c foc_layer/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/JavierMunozSaez/Documents/GitHub/javiFOC/javiFOC/foc_layer" -I"C:/Users/JavierMunozSaez/Documents/GitHub/javiFOC/javiFOC/hw_layer" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-foc_layer

clean-foc_layer:
	-$(RM) ./foc_layer/adctovoltage_and_current.cyclo ./foc_layer/adctovoltage_and_current.d ./foc_layer/adctovoltage_and_current.o ./foc_layer/adctovoltage_and_current.su ./foc_layer/vabctoduty.cyclo ./foc_layer/vabctoduty.d ./foc_layer/vabctoduty.o ./foc_layer/vabctoduty.su

.PHONY: clean-foc_layer

