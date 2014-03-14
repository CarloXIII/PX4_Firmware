################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/modules/px4iofirmware/mixer.cpp 

C_SRCS += \
../src/modules/px4iofirmware/adc.c \
../src/modules/px4iofirmware/controls.c \
../src/modules/px4iofirmware/dsm.c \
../src/modules/px4iofirmware/i2c.c \
../src/modules/px4iofirmware/px4io.c \
../src/modules/px4iofirmware/registers.c \
../src/modules/px4iofirmware/safety.c \
../src/modules/px4iofirmware/sbus.c \
../src/modules/px4iofirmware/serial.c 

OBJS += \
./src/modules/px4iofirmware/adc.o \
./src/modules/px4iofirmware/controls.o \
./src/modules/px4iofirmware/dsm.o \
./src/modules/px4iofirmware/i2c.o \
./src/modules/px4iofirmware/mixer.o \
./src/modules/px4iofirmware/px4io.o \
./src/modules/px4iofirmware/registers.o \
./src/modules/px4iofirmware/safety.o \
./src/modules/px4iofirmware/sbus.o \
./src/modules/px4iofirmware/serial.o 

C_DEPS += \
./src/modules/px4iofirmware/adc.d \
./src/modules/px4iofirmware/controls.d \
./src/modules/px4iofirmware/dsm.d \
./src/modules/px4iofirmware/i2c.d \
./src/modules/px4iofirmware/px4io.d \
./src/modules/px4iofirmware/registers.d \
./src/modules/px4iofirmware/safety.d \
./src/modules/px4iofirmware/sbus.d \
./src/modules/px4iofirmware/serial.d 

CPP_DEPS += \
./src/modules/px4iofirmware/mixer.d 


# Each subdirectory must supply rules for building sources it contributes
src/modules/px4iofirmware/%.o: ../src/modules/px4iofirmware/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/modules/px4iofirmware/%.o: ../src/modules/px4iofirmware/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


