################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/drivers/device/cdev.cpp \
../src/drivers/device/device.cpp \
../src/drivers/device/i2c.cpp \
../src/drivers/device/pio.cpp \
../src/drivers/device/spi.cpp 

OBJS += \
./src/drivers/device/cdev.o \
./src/drivers/device/device.o \
./src/drivers/device/i2c.o \
./src/drivers/device/pio.o \
./src/drivers/device/spi.o 

CPP_DEPS += \
./src/drivers/device/cdev.d \
./src/drivers/device/device.d \
./src/drivers/device/i2c.d \
./src/drivers/device/pio.d \
./src/drivers/device/spi.d 


# Each subdirectory must supply rules for building sources it contributes
src/drivers/device/%.o: ../src/drivers/device/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


