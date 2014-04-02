################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CXX_SRCS += \
../NuttX/NxWidgets/UnitTests/CScrollbarHorizontal/cscrollbarhorizontal_main.cxx \
../NuttX/NxWidgets/UnitTests/CScrollbarHorizontal/cscrollbarhorizontaltest.cxx 

OBJS += \
./NuttX/NxWidgets/UnitTests/CScrollbarHorizontal/cscrollbarhorizontal_main.o \
./NuttX/NxWidgets/UnitTests/CScrollbarHorizontal/cscrollbarhorizontaltest.o 

CXX_DEPS += \
./NuttX/NxWidgets/UnitTests/CScrollbarHorizontal/cscrollbarhorizontal_main.d \
./NuttX/NxWidgets/UnitTests/CScrollbarHorizontal/cscrollbarhorizontaltest.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/NxWidgets/UnitTests/CScrollbarHorizontal/%.o: ../NuttX/NxWidgets/UnitTests/CScrollbarHorizontal/%.cxx
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


