################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CXX_SRCS += \
../NuttX/NxWidgets/UnitTests/CLatchButton/clatchbutton_main.cxx \
../NuttX/NxWidgets/UnitTests/CLatchButton/clatchbuttontest.cxx 

OBJS += \
./NuttX/NxWidgets/UnitTests/CLatchButton/clatchbutton_main.o \
./NuttX/NxWidgets/UnitTests/CLatchButton/clatchbuttontest.o 

CXX_DEPS += \
./NuttX/NxWidgets/UnitTests/CLatchButton/clatchbutton_main.d \
./NuttX/NxWidgets/UnitTests/CLatchButton/clatchbuttontest.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/NxWidgets/UnitTests/CLatchButton/%.o: ../NuttX/NxWidgets/UnitTests/CLatchButton/%.cxx
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


