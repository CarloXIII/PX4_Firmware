################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CXX_SRCS += \
../NuttX/NxWidgets/UnitTests/CRadioButton/cradiobutton_main.cxx \
../NuttX/NxWidgets/UnitTests/CRadioButton/cradiobuttontest.cxx 

OBJS += \
./NuttX/NxWidgets/UnitTests/CRadioButton/cradiobutton_main.o \
./NuttX/NxWidgets/UnitTests/CRadioButton/cradiobuttontest.o 

CXX_DEPS += \
./NuttX/NxWidgets/UnitTests/CRadioButton/cradiobutton_main.d \
./NuttX/NxWidgets/UnitTests/CRadioButton/cradiobuttontest.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/NxWidgets/UnitTests/CRadioButton/%.o: ../NuttX/NxWidgets/UnitTests/CRadioButton/%.cxx
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


