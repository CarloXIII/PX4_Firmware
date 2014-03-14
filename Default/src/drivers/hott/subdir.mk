################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/drivers/hott/comms.cpp \
../src/drivers/hott/messages.cpp 

OBJS += \
./src/drivers/hott/comms.o \
./src/drivers/hott/messages.o 

CPP_DEPS += \
./src/drivers/hott/comms.d \
./src/drivers/hott/messages.d 


# Each subdirectory must supply rules for building sources it contributes
src/drivers/hott/%.o: ../src/drivers/hott/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


