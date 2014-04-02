################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/lib/conversion/rotation.cpp 

OBJS += \
./src/lib/conversion/rotation.o 

CPP_DEPS += \
./src/lib/conversion/rotation.d 


# Each subdirectory must supply rules for building sources it contributes
src/lib/conversion/%.o: ../src/lib/conversion/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


