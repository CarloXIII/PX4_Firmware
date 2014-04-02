################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CXX_SRCS += \
../NuttX/apps/examples/cxxtest/cxxtest_main.cxx 

OBJS += \
./NuttX/apps/examples/cxxtest/cxxtest_main.o 

CXX_DEPS += \
./NuttX/apps/examples/cxxtest/cxxtest_main.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/apps/examples/cxxtest/%.o: ../NuttX/apps/examples/cxxtest/%.cxx
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


