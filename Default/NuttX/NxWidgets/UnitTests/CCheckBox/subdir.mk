################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CXX_SRCS += \
../NuttX/NxWidgets/UnitTests/CCheckBox/ccheckbox_main.cxx \
../NuttX/NxWidgets/UnitTests/CCheckBox/ccheckboxtest.cxx 

OBJS += \
./NuttX/NxWidgets/UnitTests/CCheckBox/ccheckbox_main.o \
./NuttX/NxWidgets/UnitTests/CCheckBox/ccheckboxtest.o 

CXX_DEPS += \
./NuttX/NxWidgets/UnitTests/CCheckBox/ccheckbox_main.d \
./NuttX/NxWidgets/UnitTests/CCheckBox/ccheckboxtest.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/NxWidgets/UnitTests/CCheckBox/%.o: ../NuttX/NxWidgets/UnitTests/CCheckBox/%.cxx
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


