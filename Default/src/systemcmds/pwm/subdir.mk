################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/systemcmds/pwm/pwm.c 

OBJS += \
./src/systemcmds/pwm/pwm.o 

C_DEPS += \
./src/systemcmds/pwm/pwm.d 


# Each subdirectory must supply rules for building sources it contributes
src/systemcmds/pwm/%.o: ../src/systemcmds/pwm/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


