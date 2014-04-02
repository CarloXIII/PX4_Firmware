################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/modules/commander/commander_tests/commander_tests.cpp \
../src/modules/commander/commander_tests/state_machine_helper_test.cpp 

OBJS += \
./src/modules/commander/commander_tests/commander_tests.o \
./src/modules/commander/commander_tests/state_machine_helper_test.o 

CPP_DEPS += \
./src/modules/commander/commander_tests/commander_tests.d \
./src/modules/commander/commander_tests/state_machine_helper_test.d 


# Each subdirectory must supply rules for building sources it contributes
src/modules/commander/commander_tests/%.o: ../src/modules/commander/commander_tests/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


