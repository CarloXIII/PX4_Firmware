################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Tools/tests-host/hrt.cpp \
../Tools/tests-host/mixer_test.cpp 

OBJS += \
./Tools/tests-host/hrt.o \
./Tools/tests-host/mixer_test.o 

CPP_DEPS += \
./Tools/tests-host/hrt.d \
./Tools/tests-host/mixer_test.d 


# Each subdirectory must supply rules for building sources it contributes
Tools/tests-host/%.o: ../Tools/tests-host/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


