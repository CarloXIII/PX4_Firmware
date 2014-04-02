################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/lib/ecl/attitude_fw/ecl_pitch_controller.cpp \
../src/lib/ecl/attitude_fw/ecl_roll_controller.cpp \
../src/lib/ecl/attitude_fw/ecl_yaw_controller.cpp 

OBJS += \
./src/lib/ecl/attitude_fw/ecl_pitch_controller.o \
./src/lib/ecl/attitude_fw/ecl_roll_controller.o \
./src/lib/ecl/attitude_fw/ecl_yaw_controller.o 

CPP_DEPS += \
./src/lib/ecl/attitude_fw/ecl_pitch_controller.d \
./src/lib/ecl/attitude_fw/ecl_roll_controller.d \
./src/lib/ecl/attitude_fw/ecl_yaw_controller.d 


# Each subdirectory must supply rules for building sources it contributes
src/lib/ecl/attitude_fw/%.o: ../src/lib/ecl/attitude_fw/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


