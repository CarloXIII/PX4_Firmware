################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/libc/time/lib_calendar2utc.c \
../NuttX/nuttx/libc/time/lib_daysbeforemonth.c \
../NuttX/nuttx/libc/time/lib_gmtime.c \
../NuttX/nuttx/libc/time/lib_gmtimer.c \
../NuttX/nuttx/libc/time/lib_isleapyear.c \
../NuttX/nuttx/libc/time/lib_mktime.c \
../NuttX/nuttx/libc/time/lib_strftime.c \
../NuttX/nuttx/libc/time/lib_time.c 

OBJS += \
./NuttX/nuttx/libc/time/lib_calendar2utc.o \
./NuttX/nuttx/libc/time/lib_daysbeforemonth.o \
./NuttX/nuttx/libc/time/lib_gmtime.o \
./NuttX/nuttx/libc/time/lib_gmtimer.o \
./NuttX/nuttx/libc/time/lib_isleapyear.o \
./NuttX/nuttx/libc/time/lib_mktime.o \
./NuttX/nuttx/libc/time/lib_strftime.o \
./NuttX/nuttx/libc/time/lib_time.o 

C_DEPS += \
./NuttX/nuttx/libc/time/lib_calendar2utc.d \
./NuttX/nuttx/libc/time/lib_daysbeforemonth.d \
./NuttX/nuttx/libc/time/lib_gmtime.d \
./NuttX/nuttx/libc/time/lib_gmtimer.d \
./NuttX/nuttx/libc/time/lib_isleapyear.d \
./NuttX/nuttx/libc/time/lib_mktime.d \
./NuttX/nuttx/libc/time/lib_strftime.d \
./NuttX/nuttx/libc/time/lib_time.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/libc/time/%.o: ../NuttX/nuttx/libc/time/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


