################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/drivers/hmc5883/hmc5883.cpp 

OBJS += \
./src/drivers/hmc5883/hmc5883.o 

CPP_DEPS += \
./src/drivers/hmc5883/hmc5883.d 


# Each subdirectory must supply rules for building sources it contributes
src/drivers/hmc5883/%.o: ../src/drivers/hmc5883/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


