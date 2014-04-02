################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/drivers/mpu6000/mpu6000.cpp 

OBJS += \
./src/drivers/mpu6000/mpu6000.o 

CPP_DEPS += \
./src/drivers/mpu6000/mpu6000.d 


# Each subdirectory must supply rules for building sources it contributes
src/drivers/mpu6000/%.o: ../src/drivers/mpu6000/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


