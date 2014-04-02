################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CXX_SRCS += \
../NuttX/NxWidgets/UnitTests/CButtonArray/cbuttonarray_main.cxx \
../NuttX/NxWidgets/UnitTests/CButtonArray/cbuttonarraytest.cxx 

OBJS += \
./NuttX/NxWidgets/UnitTests/CButtonArray/cbuttonarray_main.o \
./NuttX/NxWidgets/UnitTests/CButtonArray/cbuttonarraytest.o 

CXX_DEPS += \
./NuttX/NxWidgets/UnitTests/CButtonArray/cbuttonarray_main.d \
./NuttX/NxWidgets/UnitTests/CButtonArray/cbuttonarraytest.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/NxWidgets/UnitTests/CButtonArray/%.o: ../NuttX/NxWidgets/UnitTests/CButtonArray/%.cxx
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


