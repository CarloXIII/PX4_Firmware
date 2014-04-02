################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/modules/position_estimator/position_estimator_main.c 

OBJS += \
./src/modules/position_estimator/position_estimator_main.o 

C_DEPS += \
./src/modules/position_estimator/position_estimator_main.d 


# Each subdirectory must supply rules for building sources it contributes
src/modules/position_estimator/%.o: ../src/modules/position_estimator/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


