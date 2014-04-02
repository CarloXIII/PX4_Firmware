################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/lib/external_lgpl/tecs/tecs.cpp 

OBJS += \
./src/lib/external_lgpl/tecs/tecs.o 

CPP_DEPS += \
./src/lib/external_lgpl/tecs/tecs.d 


# Each subdirectory must supply rules for building sources it contributes
src/lib/external_lgpl/tecs/%.o: ../src/lib/external_lgpl/tecs/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


