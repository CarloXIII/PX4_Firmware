################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/drivers/gps/gps.cpp \
../src/drivers/gps/gps_helper.cpp \
../src/drivers/gps/mtk.cpp \
../src/drivers/gps/novatel.cpp \
../src/drivers/gps/ubx.cpp 

OBJS += \
./src/drivers/gps/gps.o \
./src/drivers/gps/gps_helper.o \
./src/drivers/gps/mtk.o \
./src/drivers/gps/novatel.o \
./src/drivers/gps/ubx.o 

CPP_DEPS += \
./src/drivers/gps/gps.d \
./src/drivers/gps/gps_helper.d \
./src/drivers/gps/mtk.d \
./src/drivers/gps/novatel.d \
./src/drivers/gps/ubx.d 


# Each subdirectory must supply rules for building sources it contributes
src/drivers/gps/%.o: ../src/drivers/gps/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


