################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/examples/px4_mavlink_debug/px4_mavlink_debug.c 

OBJS += \
./src/examples/px4_mavlink_debug/px4_mavlink_debug.o 

C_DEPS += \
./src/examples/px4_mavlink_debug/px4_mavlink_debug.d 


# Each subdirectory must supply rules for building sources it contributes
src/examples/px4_mavlink_debug/%.o: ../src/examples/px4_mavlink_debug/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


