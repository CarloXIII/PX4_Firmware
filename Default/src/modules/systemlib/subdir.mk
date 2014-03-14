################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/modules/systemlib/airspeed.c \
../src/modules/systemlib/board_serial.c \
../src/modules/systemlib/conversions.c \
../src/modules/systemlib/cpuload.c \
../src/modules/systemlib/err.c \
../src/modules/systemlib/getopt_long.c \
../src/modules/systemlib/hx_stream.c \
../src/modules/systemlib/mavlink_log.c \
../src/modules/systemlib/otp.c \
../src/modules/systemlib/perf_counter.c \
../src/modules/systemlib/ppm_decode.c \
../src/modules/systemlib/rc_check.c \
../src/modules/systemlib/system_params.c \
../src/modules/systemlib/systemlib.c \
../src/modules/systemlib/up_cxxinitialize.c 

OBJS += \
./src/modules/systemlib/airspeed.o \
./src/modules/systemlib/board_serial.o \
./src/modules/systemlib/conversions.o \
./src/modules/systemlib/cpuload.o \
./src/modules/systemlib/err.o \
./src/modules/systemlib/getopt_long.o \
./src/modules/systemlib/hx_stream.o \
./src/modules/systemlib/mavlink_log.o \
./src/modules/systemlib/otp.o \
./src/modules/systemlib/perf_counter.o \
./src/modules/systemlib/ppm_decode.o \
./src/modules/systemlib/rc_check.o \
./src/modules/systemlib/system_params.o \
./src/modules/systemlib/systemlib.o \
./src/modules/systemlib/up_cxxinitialize.o 

C_DEPS += \
./src/modules/systemlib/airspeed.d \
./src/modules/systemlib/board_serial.d \
./src/modules/systemlib/conversions.d \
./src/modules/systemlib/cpuload.d \
./src/modules/systemlib/err.d \
./src/modules/systemlib/getopt_long.d \
./src/modules/systemlib/hx_stream.d \
./src/modules/systemlib/mavlink_log.d \
./src/modules/systemlib/otp.d \
./src/modules/systemlib/perf_counter.d \
./src/modules/systemlib/ppm_decode.d \
./src/modules/systemlib/rc_check.d \
./src/modules/systemlib/system_params.d \
./src/modules/systemlib/systemlib.d \
./src/modules/systemlib/up_cxxinitialize.d 


# Each subdirectory must supply rules for building sources it contributes
src/modules/systemlib/%.o: ../src/modules/systemlib/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


