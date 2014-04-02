################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/modules/multirotor_pos_control/multirotor_pos_control.c \
../src/modules/multirotor_pos_control/multirotor_pos_control_params.c \
../src/modules/multirotor_pos_control/thrust_pid.c 

OBJS += \
./src/modules/multirotor_pos_control/multirotor_pos_control.o \
./src/modules/multirotor_pos_control/multirotor_pos_control_params.o \
./src/modules/multirotor_pos_control/thrust_pid.o 

C_DEPS += \
./src/modules/multirotor_pos_control/multirotor_pos_control.d \
./src/modules/multirotor_pos_control/multirotor_pos_control_params.d \
./src/modules/multirotor_pos_control/thrust_pid.d 


# Each subdirectory must supply rules for building sources it contributes
src/modules/multirotor_pos_control/%.o: ../src/modules/multirotor_pos_control/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


