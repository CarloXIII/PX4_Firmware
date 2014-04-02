################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/lib/ecl/l1/ecl_l1_pos_controller.cpp 

OBJS += \
./src/lib/ecl/l1/ecl_l1_pos_controller.o 

CPP_DEPS += \
./src/lib/ecl/l1/ecl_l1_pos_controller.d 


# Each subdirectory must supply rules for building sources it contributes
src/lib/ecl/l1/%.o: ../src/lib/ecl/l1/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


