################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CXX_SRCS += \
../NuttX/NxWidgets/UnitTests/CScrollbarVertical/cscrollbarvertical_main.cxx \
../NuttX/NxWidgets/UnitTests/CScrollbarVertical/cscrollbarverticaltest.cxx 

OBJS += \
./NuttX/NxWidgets/UnitTests/CScrollbarVertical/cscrollbarvertical_main.o \
./NuttX/NxWidgets/UnitTests/CScrollbarVertical/cscrollbarverticaltest.o 

CXX_DEPS += \
./NuttX/NxWidgets/UnitTests/CScrollbarVertical/cscrollbarvertical_main.d \
./NuttX/NxWidgets/UnitTests/CScrollbarVertical/cscrollbarverticaltest.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/NxWidgets/UnitTests/CScrollbarVertical/%.o: ../NuttX/NxWidgets/UnitTests/CScrollbarVertical/%.cxx
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


