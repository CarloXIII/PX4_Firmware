################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/drivers/ardrone_interface/ardrone_interface.c \
../src/drivers/ardrone_interface/ardrone_motor_control.c 

OBJS += \
./src/drivers/ardrone_interface/ardrone_interface.o \
./src/drivers/ardrone_interface/ardrone_motor_control.o 

C_DEPS += \
./src/drivers/ardrone_interface/ardrone_interface.d \
./src/drivers/ardrone_interface/ardrone_motor_control.d 


# Each subdirectory must supply rules for building sources it contributes
src/drivers/ardrone_interface/%.o: ../src/drivers/ardrone_interface/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


