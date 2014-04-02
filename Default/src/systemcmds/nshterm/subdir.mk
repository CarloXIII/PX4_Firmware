################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/systemcmds/nshterm/nshterm.c 

OBJS += \
./src/systemcmds/nshterm/nshterm.o 

C_DEPS += \
./src/systemcmds/nshterm/nshterm.d 


# Each subdirectory must supply rules for building sources it contributes
src/systemcmds/nshterm/%.o: ../src/systemcmds/nshterm/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


