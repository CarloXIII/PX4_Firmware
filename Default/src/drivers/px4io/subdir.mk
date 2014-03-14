################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/drivers/px4io/px4io.cpp \
../src/drivers/px4io/px4io_i2c.cpp \
../src/drivers/px4io/px4io_serial.cpp \
../src/drivers/px4io/px4io_uploader.cpp 

OBJS += \
./src/drivers/px4io/px4io.o \
./src/drivers/px4io/px4io_i2c.o \
./src/drivers/px4io/px4io_serial.o \
./src/drivers/px4io/px4io_uploader.o 

CPP_DEPS += \
./src/drivers/px4io/px4io.d \
./src/drivers/px4io/px4io_i2c.d \
./src/drivers/px4io/px4io_serial.d \
./src/drivers/px4io/px4io_uploader.d 


# Each subdirectory must supply rules for building sources it contributes
src/drivers/px4io/%.o: ../src/drivers/px4io/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


