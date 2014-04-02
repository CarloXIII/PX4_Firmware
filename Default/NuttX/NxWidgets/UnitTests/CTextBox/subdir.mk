################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CXX_SRCS += \
../NuttX/NxWidgets/UnitTests/CTextBox/ctextbox_main.cxx \
../NuttX/NxWidgets/UnitTests/CTextBox/ctextboxtest.cxx 

OBJS += \
./NuttX/NxWidgets/UnitTests/CTextBox/ctextbox_main.o \
./NuttX/NxWidgets/UnitTests/CTextBox/ctextboxtest.o 

CXX_DEPS += \
./NuttX/NxWidgets/UnitTests/CTextBox/ctextbox_main.d \
./NuttX/NxWidgets/UnitTests/CTextBox/ctextboxtest.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/NxWidgets/UnitTests/CTextBox/%.o: ../NuttX/NxWidgets/UnitTests/CTextBox/%.cxx
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


