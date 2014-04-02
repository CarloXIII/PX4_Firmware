################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/modules/unit_test/unit_test.cpp 

OBJS += \
./src/modules/unit_test/unit_test.o 

CPP_DEPS += \
./src/modules/unit_test/unit_test.d 


# Each subdirectory must supply rules for building sources it contributes
src/modules/unit_test/%.o: ../src/modules/unit_test/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


