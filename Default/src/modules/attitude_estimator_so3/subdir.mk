################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/modules/attitude_estimator_so3/attitude_estimator_so3_main.cpp 

C_SRCS += \
../src/modules/attitude_estimator_so3/attitude_estimator_so3_params.c 

OBJS += \
./src/modules/attitude_estimator_so3/attitude_estimator_so3_main.o \
./src/modules/attitude_estimator_so3/attitude_estimator_so3_params.o 

C_DEPS += \
./src/modules/attitude_estimator_so3/attitude_estimator_so3_params.d 

CPP_DEPS += \
./src/modules/attitude_estimator_so3/attitude_estimator_so3_main.d 


# Each subdirectory must supply rules for building sources it contributes
src/modules/attitude_estimator_so3/%.o: ../src/modules/attitude_estimator_so3/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/modules/attitude_estimator_so3/%.o: ../src/modules/attitude_estimator_so3/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


