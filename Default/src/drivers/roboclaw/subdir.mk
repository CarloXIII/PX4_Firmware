################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/drivers/roboclaw/RoboClaw.cpp \
../src/drivers/roboclaw/roboclaw_main.cpp 

OBJS += \
./src/drivers/roboclaw/RoboClaw.o \
./src/drivers/roboclaw/roboclaw_main.o 

CPP_DEPS += \
./src/drivers/roboclaw/RoboClaw.d \
./src/drivers/roboclaw/roboclaw_main.d 


# Each subdirectory must supply rules for building sources it contributes
src/drivers/roboclaw/%.o: ../src/drivers/roboclaw/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


