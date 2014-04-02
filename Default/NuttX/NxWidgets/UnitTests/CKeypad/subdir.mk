################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CXX_SRCS += \
../NuttX/NxWidgets/UnitTests/CKeypad/ckeypad_main.cxx \
../NuttX/NxWidgets/UnitTests/CKeypad/ckeypadtest.cxx 

OBJS += \
./NuttX/NxWidgets/UnitTests/CKeypad/ckeypad_main.o \
./NuttX/NxWidgets/UnitTests/CKeypad/ckeypadtest.o 

CXX_DEPS += \
./NuttX/NxWidgets/UnitTests/CKeypad/ckeypad_main.d \
./NuttX/NxWidgets/UnitTests/CKeypad/ckeypadtest.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/NxWidgets/UnitTests/CKeypad/%.o: ../NuttX/NxWidgets/UnitTests/CKeypad/%.cxx
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


