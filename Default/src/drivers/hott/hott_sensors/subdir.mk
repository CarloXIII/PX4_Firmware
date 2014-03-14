################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/drivers/hott/hott_sensors/hott_sensors.cpp 

OBJS += \
./src/drivers/hott/hott_sensors/hott_sensors.o 

CPP_DEPS += \
./src/drivers/hott/hott_sensors/hott_sensors.d 


# Each subdirectory must supply rules for building sources it contributes
src/drivers/hott/hott_sensors/%.o: ../src/drivers/hott/hott_sensors/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


