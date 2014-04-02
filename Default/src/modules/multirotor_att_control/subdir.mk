################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/modules/multirotor_att_control/multirotor_att_control_main.c \
../src/modules/multirotor_att_control/multirotor_attitude_control.c \
../src/modules/multirotor_att_control/multirotor_rate_control.c 

OBJS += \
./src/modules/multirotor_att_control/multirotor_att_control_main.o \
./src/modules/multirotor_att_control/multirotor_attitude_control.o \
./src/modules/multirotor_att_control/multirotor_rate_control.o 

C_DEPS += \
./src/modules/multirotor_att_control/multirotor_att_control_main.d \
./src/modules/multirotor_att_control/multirotor_attitude_control.d \
./src/modules/multirotor_att_control/multirotor_rate_control.d 


# Each subdirectory must supply rules for building sources it contributes
src/modules/multirotor_att_control/%.o: ../src/modules/multirotor_att_control/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


