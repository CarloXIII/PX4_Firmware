################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/apps/modbus/rtu/mbcrc.c \
../NuttX/apps/modbus/rtu/mbrtu.c 

OBJS += \
./NuttX/apps/modbus/rtu/mbcrc.o \
./NuttX/apps/modbus/rtu/mbrtu.o 

C_DEPS += \
./NuttX/apps/modbus/rtu/mbcrc.d \
./NuttX/apps/modbus/rtu/mbrtu.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/apps/modbus/rtu/%.o: ../NuttX/apps/modbus/rtu/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


