################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/apps/examples/pwm/pwm_main.c 

OBJS += \
./NuttX/apps/examples/pwm/pwm_main.o 

C_DEPS += \
./NuttX/apps/examples/pwm/pwm_main.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/apps/examples/pwm/%.o: ../NuttX/apps/examples/pwm/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


