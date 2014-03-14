################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/systemcmds/preflight_check/preflight_check.c 

OBJS += \
./src/systemcmds/preflight_check/preflight_check.o 

C_DEPS += \
./src/systemcmds/preflight_check/preflight_check.d 


# Each subdirectory must supply rules for building sources it contributes
src/systemcmds/preflight_check/%.o: ../src/systemcmds/preflight_check/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


