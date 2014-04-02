################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/drivers/rpm_arduino/rpm_arduino.cpp \
../src/drivers/rpm_arduino/rpm_arduino_helper.cpp \
../src/drivers/rpm_arduino/rpm_arduino_parser.cpp 

OBJS += \
./src/drivers/rpm_arduino/rpm_arduino.o \
./src/drivers/rpm_arduino/rpm_arduino_helper.o \
./src/drivers/rpm_arduino/rpm_arduino_parser.o 

CPP_DEPS += \
./src/drivers/rpm_arduino/rpm_arduino.d \
./src/drivers/rpm_arduino/rpm_arduino_helper.d \
./src/drivers/rpm_arduino/rpm_arduino_parser.d 


# Each subdirectory must supply rules for building sources it contributes
src/drivers/rpm_arduino/%.o: ../src/drivers/rpm_arduino/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


