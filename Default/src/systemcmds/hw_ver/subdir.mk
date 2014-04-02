################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/systemcmds/hw_ver/hw_ver.c 

OBJS += \
./src/systemcmds/hw_ver/hw_ver.o 

C_DEPS += \
./src/systemcmds/hw_ver/hw_ver.d 


# Each subdirectory must supply rules for building sources it contributes
src/systemcmds/hw_ver/%.o: ../src/systemcmds/hw_ver/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


