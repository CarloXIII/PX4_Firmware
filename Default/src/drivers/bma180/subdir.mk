################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/drivers/bma180/bma180.cpp 

OBJS += \
./src/drivers/bma180/bma180.o 

CPP_DEPS += \
./src/drivers/bma180/bma180.d 


# Each subdirectory must supply rules for building sources it contributes
src/drivers/bma180/%.o: ../src/drivers/bma180/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


