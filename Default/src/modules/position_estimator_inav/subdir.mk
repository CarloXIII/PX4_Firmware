################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/modules/position_estimator_inav/inertial_filter.c \
../src/modules/position_estimator_inav/position_estimator_inav_main.c \
../src/modules/position_estimator_inav/position_estimator_inav_params.c 

OBJS += \
./src/modules/position_estimator_inav/inertial_filter.o \
./src/modules/position_estimator_inav/position_estimator_inav_main.o \
./src/modules/position_estimator_inav/position_estimator_inav_params.o 

C_DEPS += \
./src/modules/position_estimator_inav/inertial_filter.d \
./src/modules/position_estimator_inav/position_estimator_inav_main.d \
./src/modules/position_estimator_inav/position_estimator_inav_params.d 


# Each subdirectory must supply rules for building sources it contributes
src/modules/position_estimator_inav/%.o: ../src/modules/position_estimator_inav/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


