################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/drivers/blinkm/blinkm.cpp 

OBJS += \
./src/drivers/blinkm/blinkm.o 

CPP_DEPS += \
./src/drivers/blinkm/blinkm.d 


# Each subdirectory must supply rules for building sources it contributes
src/drivers/blinkm/%.o: ../src/drivers/blinkm/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


