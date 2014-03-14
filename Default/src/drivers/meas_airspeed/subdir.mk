################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/drivers/meas_airspeed/meas_airspeed.cpp 

OBJS += \
./src/drivers/meas_airspeed/meas_airspeed.o 

CPP_DEPS += \
./src/drivers/meas_airspeed/meas_airspeed.d 


# Each subdirectory must supply rules for building sources it contributes
src/drivers/meas_airspeed/%.o: ../src/drivers/meas_airspeed/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


