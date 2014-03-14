################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/apps/modbus/ascii/mbascii.c 

OBJS += \
./NuttX/apps/modbus/ascii/mbascii.o 

C_DEPS += \
./NuttX/apps/modbus/ascii/mbascii.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/apps/modbus/ascii/%.o: ../NuttX/apps/modbus/ascii/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


