################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/systemcmds/reboot/reboot.c 

OBJS += \
./src/systemcmds/reboot/reboot.o 

C_DEPS += \
./src/systemcmds/reboot/reboot.d 


# Each subdirectory must supply rules for building sources it contributes
src/systemcmds/reboot/%.o: ../src/systemcmds/reboot/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


