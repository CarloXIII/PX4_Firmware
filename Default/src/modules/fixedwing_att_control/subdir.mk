################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/modules/fixedwing_att_control/fixedwing_att_control_att.c \
../src/modules/fixedwing_att_control/fixedwing_att_control_main.c \
../src/modules/fixedwing_att_control/fixedwing_att_control_rate.c 

OBJS += \
./src/modules/fixedwing_att_control/fixedwing_att_control_att.o \
./src/modules/fixedwing_att_control/fixedwing_att_control_main.o \
./src/modules/fixedwing_att_control/fixedwing_att_control_rate.o 

C_DEPS += \
./src/modules/fixedwing_att_control/fixedwing_att_control_att.d \
./src/modules/fixedwing_att_control/fixedwing_att_control_main.d \
./src/modules/fixedwing_att_control/fixedwing_att_control_rate.d 


# Each subdirectory must supply rules for building sources it contributes
src/modules/fixedwing_att_control/%.o: ../src/modules/fixedwing_att_control/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


