################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/hw_layer/hardware_pwm.c 

OBJS += \
./Core/hw_layer/hardware_pwm.o 

C_DEPS += \
./Core/hw_layer/hardware_pwm.d 


# Each subdirectory must supply rules for building sources it contributes
Core/hw_layer/%.o Core/hw_layer/%.su Core/hw_layer/%.cyclo: ../Core/hw_layer/%.c Core/hw_layer/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-hw_layer

clean-Core-2f-hw_layer:
	-$(RM) ./Core/hw_layer/hardware_pwm.cyclo ./Core/hw_layer/hardware_pwm.d ./Core/hw_layer/hardware_pwm.o ./Core/hw_layer/hardware_pwm.su

.PHONY: clean-Core-2f-hw_layer

