################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/drivers/ms5611/ms5611.cpp \
../src/drivers/ms5611/ms5611_i2c.cpp \
../src/drivers/ms5611/ms5611_spi.cpp 

OBJS += \
./src/drivers/ms5611/ms5611.o \
./src/drivers/ms5611/ms5611_i2c.o \
./src/drivers/ms5611/ms5611_spi.o 

CPP_DEPS += \
./src/drivers/ms5611/ms5611.d \
./src/drivers/ms5611/ms5611_i2c.d \
./src/drivers/ms5611/ms5611_spi.d 


# Each subdirectory must supply rules for building sources it contributes
src/drivers/ms5611/%.o: ../src/drivers/ms5611/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


