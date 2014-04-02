################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CXX_SRCS += \
../NuttX/NxWidgets/UnitTests/CProgressBar/cprogressbar_main.cxx \
../NuttX/NxWidgets/UnitTests/CProgressBar/cprogressbartest.cxx 

OBJS += \
./NuttX/NxWidgets/UnitTests/CProgressBar/cprogressbar_main.o \
./NuttX/NxWidgets/UnitTests/CProgressBar/cprogressbartest.o 

CXX_DEPS += \
./NuttX/NxWidgets/UnitTests/CProgressBar/cprogressbar_main.d \
./NuttX/NxWidgets/UnitTests/CProgressBar/cprogressbartest.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/NxWidgets/UnitTests/CProgressBar/%.o: ../NuttX/NxWidgets/UnitTests/CProgressBar/%.cxx
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


