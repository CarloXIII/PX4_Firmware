################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/modules/uORB/objects_common.cpp \
../src/modules/uORB/uORB.cpp 

OBJS += \
./src/modules/uORB/objects_common.o \
./src/modules/uORB/uORB.o 

CPP_DEPS += \
./src/modules/uORB/objects_common.d \
./src/modules/uORB/uORB.d 


# Each subdirectory must supply rules for building sources it contributes
src/modules/uORB/%.o: ../src/modules/uORB/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


