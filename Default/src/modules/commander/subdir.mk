################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/modules/commander/accelerometer_calibration.cpp \
../src/modules/commander/airspeed_calibration.cpp \
../src/modules/commander/baro_calibration.cpp \
../src/modules/commander/calibration_routines.cpp \
../src/modules/commander/commander.cpp \
../src/modules/commander/commander_helper.cpp \
../src/modules/commander/gyro_calibration.cpp \
../src/modules/commander/mag_calibration.cpp \
../src/modules/commander/rc_calibration.cpp \
../src/modules/commander/state_machine_helper.cpp 

C_SRCS += \
../src/modules/commander/commander_params.c 

OBJS += \
./src/modules/commander/accelerometer_calibration.o \
./src/modules/commander/airspeed_calibration.o \
./src/modules/commander/baro_calibration.o \
./src/modules/commander/calibration_routines.o \
./src/modules/commander/commander.o \
./src/modules/commander/commander_helper.o \
./src/modules/commander/commander_params.o \
./src/modules/commander/gyro_calibration.o \
./src/modules/commander/mag_calibration.o \
./src/modules/commander/rc_calibration.o \
./src/modules/commander/state_machine_helper.o 

C_DEPS += \
./src/modules/commander/commander_params.d 

CPP_DEPS += \
./src/modules/commander/accelerometer_calibration.d \
./src/modules/commander/airspeed_calibration.d \
./src/modules/commander/baro_calibration.d \
./src/modules/commander/calibration_routines.d \
./src/modules/commander/commander.d \
./src/modules/commander/commander_helper.d \
./src/modules/commander/gyro_calibration.d \
./src/modules/commander/mag_calibration.d \
./src/modules/commander/rc_calibration.d \
./src/modules/commander/state_machine_helper.d 


# Each subdirectory must supply rules for building sources it contributes
src/modules/commander/%.o: ../src/modules/commander/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/modules/commander/%.o: ../src/modules/commander/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


