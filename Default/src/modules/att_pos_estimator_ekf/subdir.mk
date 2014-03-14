################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/modules/att_pos_estimator_ekf/KalmanNav.cpp \
../src/modules/att_pos_estimator_ekf/kalman_main.cpp 

C_SRCS += \
../src/modules/att_pos_estimator_ekf/params.c 

OBJS += \
./src/modules/att_pos_estimator_ekf/KalmanNav.o \
./src/modules/att_pos_estimator_ekf/kalman_main.o \
./src/modules/att_pos_estimator_ekf/params.o 

C_DEPS += \
./src/modules/att_pos_estimator_ekf/params.d 

CPP_DEPS += \
./src/modules/att_pos_estimator_ekf/KalmanNav.d \
./src/modules/att_pos_estimator_ekf/kalman_main.d 


# Each subdirectory must supply rules for building sources it contributes
src/modules/att_pos_estimator_ekf/%.o: ../src/modules/att_pos_estimator_ekf/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/modules/att_pos_estimator_ekf/%.o: ../src/modules/att_pos_estimator_ekf/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


