################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/modules/fixedwing_pos_control/fixedwing_pos_control_main.c 

OBJS += \
./src/modules/fixedwing_pos_control/fixedwing_pos_control_main.o 

C_DEPS += \
./src/modules/fixedwing_pos_control/fixedwing_pos_control_main.d 


# Each subdirectory must supply rules for building sources it contributes
src/modules/fixedwing_pos_control/%.o: ../src/modules/fixedwing_pos_control/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


