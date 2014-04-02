################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/systemcmds/tests/test_conv.cpp \
../src/systemcmds/tests/test_mixer.cpp 

C_SRCS += \
../src/systemcmds/tests/test_adc.c \
../src/systemcmds/tests/test_bson.c \
../src/systemcmds/tests/test_file.c \
../src/systemcmds/tests/test_float.c \
../src/systemcmds/tests/test_gpio.c \
../src/systemcmds/tests/test_hott_telemetry.c \
../src/systemcmds/tests/test_hrt.c \
../src/systemcmds/tests/test_int.c \
../src/systemcmds/tests/test_jig_voltages.c \
../src/systemcmds/tests/test_led.c \
../src/systemcmds/tests/test_mount.c \
../src/systemcmds/tests/test_mtd.c \
../src/systemcmds/tests/test_param.c \
../src/systemcmds/tests/test_ppm_loopback.c \
../src/systemcmds/tests/test_rc.c \
../src/systemcmds/tests/test_sensors.c \
../src/systemcmds/tests/test_servo.c \
../src/systemcmds/tests/test_sleep.c \
../src/systemcmds/tests/test_time.c \
../src/systemcmds/tests/test_uart_baudchange.c \
../src/systemcmds/tests/test_uart_console.c \
../src/systemcmds/tests/test_uart_loopback.c \
../src/systemcmds/tests/test_uart_send.c \
../src/systemcmds/tests/tests_main.c 

OBJS += \
./src/systemcmds/tests/test_adc.o \
./src/systemcmds/tests/test_bson.o \
./src/systemcmds/tests/test_conv.o \
./src/systemcmds/tests/test_file.o \
./src/systemcmds/tests/test_float.o \
./src/systemcmds/tests/test_gpio.o \
./src/systemcmds/tests/test_hott_telemetry.o \
./src/systemcmds/tests/test_hrt.o \
./src/systemcmds/tests/test_int.o \
./src/systemcmds/tests/test_jig_voltages.o \
./src/systemcmds/tests/test_led.o \
./src/systemcmds/tests/test_mixer.o \
./src/systemcmds/tests/test_mount.o \
./src/systemcmds/tests/test_mtd.o \
./src/systemcmds/tests/test_param.o \
./src/systemcmds/tests/test_ppm_loopback.o \
./src/systemcmds/tests/test_rc.o \
./src/systemcmds/tests/test_sensors.o \
./src/systemcmds/tests/test_servo.o \
./src/systemcmds/tests/test_sleep.o \
./src/systemcmds/tests/test_time.o \
./src/systemcmds/tests/test_uart_baudchange.o \
./src/systemcmds/tests/test_uart_console.o \
./src/systemcmds/tests/test_uart_loopback.o \
./src/systemcmds/tests/test_uart_send.o \
./src/systemcmds/tests/tests_main.o 

C_DEPS += \
./src/systemcmds/tests/test_adc.d \
./src/systemcmds/tests/test_bson.d \
./src/systemcmds/tests/test_file.d \
./src/systemcmds/tests/test_float.d \
./src/systemcmds/tests/test_gpio.d \
./src/systemcmds/tests/test_hott_telemetry.d \
./src/systemcmds/tests/test_hrt.d \
./src/systemcmds/tests/test_int.d \
./src/systemcmds/tests/test_jig_voltages.d \
./src/systemcmds/tests/test_led.d \
./src/systemcmds/tests/test_mount.d \
./src/systemcmds/tests/test_mtd.d \
./src/systemcmds/tests/test_param.d \
./src/systemcmds/tests/test_ppm_loopback.d \
./src/systemcmds/tests/test_rc.d \
./src/systemcmds/tests/test_sensors.d \
./src/systemcmds/tests/test_servo.d \
./src/systemcmds/tests/test_sleep.d \
./src/systemcmds/tests/test_time.d \
./src/systemcmds/tests/test_uart_baudchange.d \
./src/systemcmds/tests/test_uart_console.d \
./src/systemcmds/tests/test_uart_loopback.d \
./src/systemcmds/tests/test_uart_send.d \
./src/systemcmds/tests/tests_main.d 

CPP_DEPS += \
./src/systemcmds/tests/test_conv.d \
./src/systemcmds/tests/test_mixer.d 


# Each subdirectory must supply rules for building sources it contributes
src/systemcmds/tests/%.o: ../src/systemcmds/tests/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/systemcmds/tests/%.o: ../src/systemcmds/tests/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


