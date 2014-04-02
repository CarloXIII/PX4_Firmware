################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/examples/flow_position_estimator/flow_position_estimator_main.c \
../src/examples/flow_position_estimator/flow_position_estimator_params.c 

OBJS += \
./src/examples/flow_position_estimator/flow_position_estimator_main.o \
./src/examples/flow_position_estimator/flow_position_estimator_params.o 

C_DEPS += \
./src/examples/flow_position_estimator/flow_position_estimator_main.d \
./src/examples/flow_position_estimator/flow_position_estimator_params.d 


# Each subdirectory must supply rules for building sources it contributes
src/examples/flow_position_estimator/%.o: ../src/examples/flow_position_estimator/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


