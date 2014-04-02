################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CXX_SRCS += \
../NuttX/NxWidgets/UnitTests/CGlyphSliderHorizontal/cglyphsliderhorizontal_main.cxx \
../NuttX/NxWidgets/UnitTests/CGlyphSliderHorizontal/cglyphsliderhorizontaltest.cxx 

OBJS += \
./NuttX/NxWidgets/UnitTests/CGlyphSliderHorizontal/cglyphsliderhorizontal_main.o \
./NuttX/NxWidgets/UnitTests/CGlyphSliderHorizontal/cglyphsliderhorizontaltest.o 

CXX_DEPS += \
./NuttX/NxWidgets/UnitTests/CGlyphSliderHorizontal/cglyphsliderhorizontal_main.d \
./NuttX/NxWidgets/UnitTests/CGlyphSliderHorizontal/cglyphsliderhorizontaltest.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/NxWidgets/UnitTests/CGlyphSliderHorizontal/%.o: ../NuttX/NxWidgets/UnitTests/CGlyphSliderHorizontal/%.cxx
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


