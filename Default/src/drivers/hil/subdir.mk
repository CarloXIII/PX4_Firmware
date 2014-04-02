################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/drivers/hil/hil.cpp 

OBJS += \
./src/drivers/hil/hil.o 

CPP_DEPS += \
./src/drivers/hil/hil.d 


# Each subdirectory must supply rules for building sources it contributes
src/drivers/hil/%.o: ../src/drivers/hil/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


