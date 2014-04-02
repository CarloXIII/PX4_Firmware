################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CXX_SRCS += \
../NuttX/NxWidgets/UnitTests/CSliderHorizonal/csliderhorizontal_main.cxx \
../NuttX/NxWidgets/UnitTests/CSliderHorizonal/csliderhorizontaltest.cxx 

OBJS += \
./NuttX/NxWidgets/UnitTests/CSliderHorizonal/csliderhorizontal_main.o \
./NuttX/NxWidgets/UnitTests/CSliderHorizonal/csliderhorizontaltest.o 

CXX_DEPS += \
./NuttX/NxWidgets/UnitTests/CSliderHorizonal/csliderhorizontal_main.d \
./NuttX/NxWidgets/UnitTests/CSliderHorizonal/csliderhorizontaltest.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/NxWidgets/UnitTests/CSliderHorizonal/%.o: ../NuttX/NxWidgets/UnitTests/CSliderHorizonal/%.cxx
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


