################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/drivers/current_sensor/current_sensor.cpp 

OBJS += \
./src/drivers/current_sensor/current_sensor.o 

CPP_DEPS += \
./src/drivers/current_sensor/current_sensor.d 


# Each subdirectory must supply rules for building sources it contributes
src/drivers/current_sensor/%.o: ../src/drivers/current_sensor/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


