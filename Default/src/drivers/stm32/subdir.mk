################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/drivers/stm32/drv_hrt.c \
../src/drivers/stm32/drv_pwm_servo.c 

OBJS += \
./src/drivers/stm32/drv_hrt.o \
./src/drivers/stm32/drv_pwm_servo.o 

C_DEPS += \
./src/drivers/stm32/drv_hrt.d \
./src/drivers/stm32/drv_pwm_servo.d 


# Each subdirectory must supply rules for building sources it contributes
src/drivers/stm32/%.o: ../src/drivers/stm32/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


