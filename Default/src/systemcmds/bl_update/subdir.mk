################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/systemcmds/bl_update/bl_update.c 

OBJS += \
./src/systemcmds/bl_update/bl_update.o 

C_DEPS += \
./src/systemcmds/bl_update/bl_update.d 


# Each subdirectory must supply rules for building sources it contributes
src/systemcmds/bl_update/%.o: ../src/systemcmds/bl_update/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


