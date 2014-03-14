################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CXX_SRCS += \
../NuttX/NxWidgets/UnitTests/CLatchButtonArray/clatchbuttonarray_main.cxx \
../NuttX/NxWidgets/UnitTests/CLatchButtonArray/clatchbuttonarraytest.cxx 

OBJS += \
./NuttX/NxWidgets/UnitTests/CLatchButtonArray/clatchbuttonarray_main.o \
./NuttX/NxWidgets/UnitTests/CLatchButtonArray/clatchbuttonarraytest.o 

CXX_DEPS += \
./NuttX/NxWidgets/UnitTests/CLatchButtonArray/clatchbuttonarray_main.d \
./NuttX/NxWidgets/UnitTests/CLatchButtonArray/clatchbuttonarraytest.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/NxWidgets/UnitTests/CLatchButtonArray/%.o: ../NuttX/NxWidgets/UnitTests/CLatchButtonArray/%.cxx
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


