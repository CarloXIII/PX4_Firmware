################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/drivers/airspeed/airspeed.cpp 

OBJS += \
./src/drivers/airspeed/airspeed.o 

CPP_DEPS += \
./src/drivers/airspeed/airspeed.d 


# Each subdirectory must supply rules for building sources it contributes
src/drivers/airspeed/%.o: ../src/drivers/airspeed/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


