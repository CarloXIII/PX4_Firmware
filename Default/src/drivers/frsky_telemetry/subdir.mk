################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/drivers/frsky_telemetry/frsky_data.c \
../src/drivers/frsky_telemetry/frsky_telemetry.c 

OBJS += \
./src/drivers/frsky_telemetry/frsky_data.o \
./src/drivers/frsky_telemetry/frsky_telemetry.o 

C_DEPS += \
./src/drivers/frsky_telemetry/frsky_data.d \
./src/drivers/frsky_telemetry/frsky_telemetry.d 


# Each subdirectory must supply rules for building sources it contributes
src/drivers/frsky_telemetry/%.o: ../src/drivers/frsky_telemetry/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


