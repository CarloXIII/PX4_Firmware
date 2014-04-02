################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CXX_SRCS += \
../NuttX/NxWidgets/UnitTests/nxwm/nxwm_main.cxx 

OBJS += \
./NuttX/NxWidgets/UnitTests/nxwm/nxwm_main.o 

CXX_DEPS += \
./NuttX/NxWidgets/UnitTests/nxwm/nxwm_main.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/NxWidgets/UnitTests/nxwm/%.o: ../NuttX/NxWidgets/UnitTests/nxwm/%.cxx
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


