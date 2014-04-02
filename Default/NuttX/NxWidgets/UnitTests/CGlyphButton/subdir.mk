################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CXX_SRCS += \
../NuttX/NxWidgets/UnitTests/CGlyphButton/cglyphbutton_main.cxx \
../NuttX/NxWidgets/UnitTests/CGlyphButton/cglyphbuttontest.cxx 

OBJS += \
./NuttX/NxWidgets/UnitTests/CGlyphButton/cglyphbutton_main.o \
./NuttX/NxWidgets/UnitTests/CGlyphButton/cglyphbuttontest.o 

CXX_DEPS += \
./NuttX/NxWidgets/UnitTests/CGlyphButton/cglyphbutton_main.d \
./NuttX/NxWidgets/UnitTests/CGlyphButton/cglyphbuttontest.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/NxWidgets/UnitTests/CGlyphButton/%.o: ../NuttX/NxWidgets/UnitTests/CGlyphButton/%.cxx
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


