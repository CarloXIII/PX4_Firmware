################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../mavlink/share/pyshared/pymavlink/generator/C/test/windows/stdafx.cpp \
../mavlink/share/pyshared/pymavlink/generator/C/test/windows/testmav.cpp 

OBJS += \
./mavlink/share/pyshared/pymavlink/generator/C/test/windows/stdafx.o \
./mavlink/share/pyshared/pymavlink/generator/C/test/windows/testmav.o 

CPP_DEPS += \
./mavlink/share/pyshared/pymavlink/generator/C/test/windows/stdafx.d \
./mavlink/share/pyshared/pymavlink/generator/C/test/windows/testmav.d 


# Each subdirectory must supply rules for building sources it contributes
mavlink/share/pyshared/pymavlink/generator/C/test/windows/%.o: ../mavlink/share/pyshared/pymavlink/generator/C/test/windows/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


