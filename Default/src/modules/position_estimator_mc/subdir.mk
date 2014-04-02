################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/modules/position_estimator_mc/position_estimator_mc_main.c \
../src/modules/position_estimator_mc/position_estimator_mc_params.c 

OBJS += \
./src/modules/position_estimator_mc/position_estimator_mc_main.o \
./src/modules/position_estimator_mc/position_estimator_mc_params.o 

C_DEPS += \
./src/modules/position_estimator_mc/position_estimator_mc_main.d \
./src/modules/position_estimator_mc/position_estimator_mc_params.d 


# Each subdirectory must supply rules for building sources it contributes
src/modules/position_estimator_mc/%.o: ../src/modules/position_estimator_mc/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


