################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/drivers/mb12xx/mb12xx.cpp 

OBJS += \
./src/drivers/mb12xx/mb12xx.o 

CPP_DEPS += \
./src/drivers/mb12xx/mb12xx.d 


# Each subdirectory must supply rules for building sources it contributes
src/drivers/mb12xx/%.o: ../src/drivers/mb12xx/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


