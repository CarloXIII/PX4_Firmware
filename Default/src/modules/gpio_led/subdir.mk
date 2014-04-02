################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/modules/gpio_led/gpio_led.c 

OBJS += \
./src/modules/gpio_led/gpio_led.o 

C_DEPS += \
./src/modules/gpio_led/gpio_led.d 


# Each subdirectory must supply rules for building sources it contributes
src/modules/gpio_led/%.o: ../src/modules/gpio_led/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


