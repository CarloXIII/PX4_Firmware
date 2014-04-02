################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CXX_SRCS += \
../NuttX/NxWidgets/UnitTests/CButton/cbutton_main.cxx \
../NuttX/NxWidgets/UnitTests/CButton/cbuttontest.cxx 

OBJS += \
./NuttX/NxWidgets/UnitTests/CButton/cbutton_main.o \
./NuttX/NxWidgets/UnitTests/CButton/cbuttontest.o 

CXX_DEPS += \
./NuttX/NxWidgets/UnitTests/CButton/cbutton_main.d \
./NuttX/NxWidgets/UnitTests/CButton/cbuttontest.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/NxWidgets/UnitTests/CButton/%.o: ../NuttX/NxWidgets/UnitTests/CButton/%.cxx
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


