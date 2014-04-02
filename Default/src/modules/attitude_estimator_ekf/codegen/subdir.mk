################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/modules/attitude_estimator_ekf/codegen/attitudeKalmanfilter.c \
../src/modules/attitude_estimator_ekf/codegen/attitudeKalmanfilter_initialize.c \
../src/modules/attitude_estimator_ekf/codegen/attitudeKalmanfilter_terminate.c \
../src/modules/attitude_estimator_ekf/codegen/cross.c \
../src/modules/attitude_estimator_ekf/codegen/eye.c \
../src/modules/attitude_estimator_ekf/codegen/mrdivide.c \
../src/modules/attitude_estimator_ekf/codegen/norm.c \
../src/modules/attitude_estimator_ekf/codegen/rdivide.c \
../src/modules/attitude_estimator_ekf/codegen/rtGetInf.c \
../src/modules/attitude_estimator_ekf/codegen/rtGetNaN.c \
../src/modules/attitude_estimator_ekf/codegen/rt_nonfinite.c 

OBJS += \
./src/modules/attitude_estimator_ekf/codegen/attitudeKalmanfilter.o \
./src/modules/attitude_estimator_ekf/codegen/attitudeKalmanfilter_initialize.o \
./src/modules/attitude_estimator_ekf/codegen/attitudeKalmanfilter_terminate.o \
./src/modules/attitude_estimator_ekf/codegen/cross.o \
./src/modules/attitude_estimator_ekf/codegen/eye.o \
./src/modules/attitude_estimator_ekf/codegen/mrdivide.o \
./src/modules/attitude_estimator_ekf/codegen/norm.o \
./src/modules/attitude_estimator_ekf/codegen/rdivide.o \
./src/modules/attitude_estimator_ekf/codegen/rtGetInf.o \
./src/modules/attitude_estimator_ekf/codegen/rtGetNaN.o \
./src/modules/attitude_estimator_ekf/codegen/rt_nonfinite.o 

C_DEPS += \
./src/modules/attitude_estimator_ekf/codegen/attitudeKalmanfilter.d \
./src/modules/attitude_estimator_ekf/codegen/attitudeKalmanfilter_initialize.d \
./src/modules/attitude_estimator_ekf/codegen/attitudeKalmanfilter_terminate.d \
./src/modules/attitude_estimator_ekf/codegen/cross.d \
./src/modules/attitude_estimator_ekf/codegen/eye.d \
./src/modules/attitude_estimator_ekf/codegen/mrdivide.d \
./src/modules/attitude_estimator_ekf/codegen/norm.d \
./src/modules/attitude_estimator_ekf/codegen/rdivide.d \
./src/modules/attitude_estimator_ekf/codegen/rtGetInf.d \
./src/modules/attitude_estimator_ekf/codegen/rtGetNaN.d \
./src/modules/attitude_estimator_ekf/codegen/rt_nonfinite.d 


# Each subdirectory must supply rules for building sources it contributes
src/modules/attitude_estimator_ekf/codegen/%.o: ../src/modules/attitude_estimator_ekf/codegen/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


