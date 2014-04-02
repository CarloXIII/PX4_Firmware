################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/drivers/max127/max127.cpp 

OBJS += \
./src/drivers/max127/max127.o 

CPP_DEPS += \
./src/drivers/max127/max127.d 


# Each subdirectory must supply rules for building sources it contributes
src/drivers/max127/%.o: ../src/drivers/max127/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


