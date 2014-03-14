################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/modules/systemlib/pwm_limit/pwm_limit.c 

OBJS += \
./src/modules/systemlib/pwm_limit/pwm_limit.o 

C_DEPS += \
./src/modules/systemlib/pwm_limit/pwm_limit.d 


# Each subdirectory must supply rules for building sources it contributes
src/modules/systemlib/pwm_limit/%.o: ../src/modules/systemlib/pwm_limit/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


