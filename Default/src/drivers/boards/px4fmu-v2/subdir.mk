################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/drivers/boards/px4fmu-v2/px4fmu2_init.c \
../src/drivers/boards/px4fmu-v2/px4fmu2_led.c \
../src/drivers/boards/px4fmu-v2/px4fmu_can.c \
../src/drivers/boards/px4fmu-v2/px4fmu_pwm_servo.c \
../src/drivers/boards/px4fmu-v2/px4fmu_spi.c \
../src/drivers/boards/px4fmu-v2/px4fmu_usb.c 

OBJS += \
./src/drivers/boards/px4fmu-v2/px4fmu2_init.o \
./src/drivers/boards/px4fmu-v2/px4fmu2_led.o \
./src/drivers/boards/px4fmu-v2/px4fmu_can.o \
./src/drivers/boards/px4fmu-v2/px4fmu_pwm_servo.o \
./src/drivers/boards/px4fmu-v2/px4fmu_spi.o \
./src/drivers/boards/px4fmu-v2/px4fmu_usb.o 

C_DEPS += \
./src/drivers/boards/px4fmu-v2/px4fmu2_init.d \
./src/drivers/boards/px4fmu-v2/px4fmu2_led.d \
./src/drivers/boards/px4fmu-v2/px4fmu_can.d \
./src/drivers/boards/px4fmu-v2/px4fmu_pwm_servo.d \
./src/drivers/boards/px4fmu-v2/px4fmu_spi.d \
./src/drivers/boards/px4fmu-v2/px4fmu_usb.d 


# Each subdirectory must supply rules for building sources it contributes
src/drivers/boards/px4fmu-v2/%.o: ../src/drivers/boards/px4fmu-v2/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


