################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/modules/position_estimator_mc/codegen/kalman_dlqe1.c \
../src/modules/position_estimator_mc/codegen/kalman_dlqe1_initialize.c \
../src/modules/position_estimator_mc/codegen/kalman_dlqe1_terminate.c \
../src/modules/position_estimator_mc/codegen/kalman_dlqe2.c \
../src/modules/position_estimator_mc/codegen/kalman_dlqe2_initialize.c \
../src/modules/position_estimator_mc/codegen/kalman_dlqe2_terminate.c \
../src/modules/position_estimator_mc/codegen/kalman_dlqe3.c \
../src/modules/position_estimator_mc/codegen/kalman_dlqe3_data.c \
../src/modules/position_estimator_mc/codegen/kalman_dlqe3_initialize.c \
../src/modules/position_estimator_mc/codegen/kalman_dlqe3_terminate.c \
../src/modules/position_estimator_mc/codegen/positionKalmanFilter1D.c \
../src/modules/position_estimator_mc/codegen/positionKalmanFilter1D_dT.c \
../src/modules/position_estimator_mc/codegen/positionKalmanFilter1D_dT_initialize.c \
../src/modules/position_estimator_mc/codegen/positionKalmanFilter1D_dT_terminate.c \
../src/modules/position_estimator_mc/codegen/positionKalmanFilter1D_initialize.c \
../src/modules/position_estimator_mc/codegen/positionKalmanFilter1D_terminate.c \
../src/modules/position_estimator_mc/codegen/randn.c \
../src/modules/position_estimator_mc/codegen/rtGetInf.c \
../src/modules/position_estimator_mc/codegen/rtGetNaN.c \
../src/modules/position_estimator_mc/codegen/rt_nonfinite.c 

OBJS += \
./src/modules/position_estimator_mc/codegen/kalman_dlqe1.o \
./src/modules/position_estimator_mc/codegen/kalman_dlqe1_initialize.o \
./src/modules/position_estimator_mc/codegen/kalman_dlqe1_terminate.o \
./src/modules/position_estimator_mc/codegen/kalman_dlqe2.o \
./src/modules/position_estimator_mc/codegen/kalman_dlqe2_initialize.o \
./src/modules/position_estimator_mc/codegen/kalman_dlqe2_terminate.o \
./src/modules/position_estimator_mc/codegen/kalman_dlqe3.o \
./src/modules/position_estimator_mc/codegen/kalman_dlqe3_data.o \
./src/modules/position_estimator_mc/codegen/kalman_dlqe3_initialize.o \
./src/modules/position_estimator_mc/codegen/kalman_dlqe3_terminate.o \
./src/modules/position_estimator_mc/codegen/positionKalmanFilter1D.o \
./src/modules/position_estimator_mc/codegen/positionKalmanFilter1D_dT.o \
./src/modules/position_estimator_mc/codegen/positionKalmanFilter1D_dT_initialize.o \
./src/modules/position_estimator_mc/codegen/positionKalmanFilter1D_dT_terminate.o \
./src/modules/position_estimator_mc/codegen/positionKalmanFilter1D_initialize.o \
./src/modules/position_estimator_mc/codegen/positionKalmanFilter1D_terminate.o \
./src/modules/position_estimator_mc/codegen/randn.o \
./src/modules/position_estimator_mc/codegen/rtGetInf.o \
./src/modules/position_estimator_mc/codegen/rtGetNaN.o \
./src/modules/position_estimator_mc/codegen/rt_nonfinite.o 

C_DEPS += \
./src/modules/position_estimator_mc/codegen/kalman_dlqe1.d \
./src/modules/position_estimator_mc/codegen/kalman_dlqe1_initialize.d \
./src/modules/position_estimator_mc/codegen/kalman_dlqe1_terminate.d \
./src/modules/position_estimator_mc/codegen/kalman_dlqe2.d \
./src/modules/position_estimator_mc/codegen/kalman_dlqe2_initialize.d \
./src/modules/position_estimator_mc/codegen/kalman_dlqe2_terminate.d \
./src/modules/position_estimator_mc/codegen/kalman_dlqe3.d \
./src/modules/position_estimator_mc/codegen/kalman_dlqe3_data.d \
./src/modules/position_estimator_mc/codegen/kalman_dlqe3_initialize.d \
./src/modules/position_estimator_mc/codegen/kalman_dlqe3_terminate.d \
./src/modules/position_estimator_mc/codegen/positionKalmanFilter1D.d \
./src/modules/position_estimator_mc/codegen/positionKalmanFilter1D_dT.d \
./src/modules/position_estimator_mc/codegen/positionKalmanFilter1D_dT_initialize.d \
./src/modules/position_estimator_mc/codegen/positionKalmanFilter1D_dT_terminate.d \
./src/modules/position_estimator_mc/codegen/positionKalmanFilter1D_initialize.d \
./src/modules/position_estimator_mc/codegen/positionKalmanFilter1D_terminate.d \
./src/modules/position_estimator_mc/codegen/randn.d \
./src/modules/position_estimator_mc/codegen/rtGetInf.d \
./src/modules/position_estimator_mc/codegen/rtGetNaN.d \
./src/modules/position_estimator_mc/codegen/rt_nonfinite.d 


# Each subdirectory must supply rules for building sources it contributes
src/modules/position_estimator_mc/codegen/%.o: ../src/modules/position_estimator_mc/codegen/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


