################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/modules/twist_angle_control/twist_angle_control.c \
../src/modules/twist_angle_control/twist_angle_control_main.c 

OBJS += \
./src/modules/twist_angle_control/twist_angle_control.o \
./src/modules/twist_angle_control/twist_angle_control_main.o 

C_DEPS += \
./src/modules/twist_angle_control/twist_angle_control.d \
./src/modules/twist_angle_control/twist_angle_control_main.d 


# Each subdirectory must supply rules for building sources it contributes
src/modules/twist_angle_control/%.o: ../src/modules/twist_angle_control/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


