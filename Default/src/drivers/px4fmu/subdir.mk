################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/drivers/px4fmu/fmu.cpp 

OBJS += \
./src/drivers/px4fmu/fmu.o 

CPP_DEPS += \
./src/drivers/px4fmu/fmu.d 


# Each subdirectory must supply rules for building sources it contributes
src/drivers/px4fmu/%.o: ../src/drivers/px4fmu/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


