################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CXX_SRCS += \
../NuttX/NxWidgets/UnitTests/CListBox/clistbox_main.cxx \
../NuttX/NxWidgets/UnitTests/CListBox/clistboxtest.cxx 

OBJS += \
./NuttX/NxWidgets/UnitTests/CListBox/clistbox_main.o \
./NuttX/NxWidgets/UnitTests/CListBox/clistboxtest.o 

CXX_DEPS += \
./NuttX/NxWidgets/UnitTests/CListBox/clistbox_main.d \
./NuttX/NxWidgets/UnitTests/CListBox/clistboxtest.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/NxWidgets/UnitTests/CListBox/%.o: ../NuttX/NxWidgets/UnitTests/CListBox/%.cxx
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


