################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/modules/sensors/sensors.cpp 

C_SRCS += \
../src/modules/sensors/sensor_params.c 

OBJS += \
./src/modules/sensors/sensor_params.o \
./src/modules/sensors/sensors.o 

C_DEPS += \
./src/modules/sensors/sensor_params.d 

CPP_DEPS += \
./src/modules/sensors/sensors.d 


# Each subdirectory must supply rules for building sources it contributes
src/modules/sensors/%.o: ../src/modules/sensors/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/modules/sensors/%.o: ../src/modules/sensors/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


