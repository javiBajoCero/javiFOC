################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../hw_layer/hardware_adc.c \
../hw_layer/hardware_opamp.c \
../hw_layer/hardware_pwm.c 

OBJS += \
./hw_layer/hardware_adc.o \
./hw_layer/hardware_opamp.o \
./hw_layer/hardware_pwm.o 

C_DEPS += \
./hw_layer/hardware_adc.d \
./hw_layer/hardware_opamp.d \
./hw_layer/hardware_pwm.d 


# Each subdirectory must supply rules for building sources it contributes
hw_layer/%.o hw_layer/%.su hw_layer/%.cyclo: ../hw_layer/%.c hw_layer/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/JavierMunozSaez/Documents/GitHub/javiFOC/javiFOC/foc_layer" -I"C:/Users/JavierMunozSaez/Documents/GitHub/javiFOC/javiFOC/hw_layer" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-hw_layer

clean-hw_layer:
	-$(RM) ./hw_layer/hardware_adc.cyclo ./hw_layer/hardware_adc.d ./hw_layer/hardware_adc.o ./hw_layer/hardware_adc.su ./hw_layer/hardware_opamp.cyclo ./hw_layer/hardware_opamp.d ./hw_layer/hardware_opamp.o ./hw_layer/hardware_opamp.su ./hw_layer/hardware_pwm.cyclo ./hw_layer/hardware_pwm.d ./hw_layer/hardware_pwm.o ./hw_layer/hardware_pwm.su

.PHONY: clean-hw_layer

