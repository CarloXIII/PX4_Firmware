################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/systemcmds/esc_calib/esc_calib.c 

OBJS += \
./src/systemcmds/esc_calib/esc_calib.o 

C_DEPS += \
./src/systemcmds/esc_calib/esc_calib.d 


# Each subdirectory must supply rules for building sources it contributes
src/systemcmds/esc_calib/%.o: ../src/systemcmds/esc_calib/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


