################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/drivers/can.c \
../NuttX/nuttx/drivers/dev_null.c \
../NuttX/nuttx/drivers/dev_zero.c \
../NuttX/nuttx/drivers/loop.c \
../NuttX/nuttx/drivers/pwm.c \
../NuttX/nuttx/drivers/ramdisk.c \
../NuttX/nuttx/drivers/rwbuffer.c \
../NuttX/nuttx/drivers/watchdog.c 

OBJS += \
./NuttX/nuttx/drivers/can.o \
./NuttX/nuttx/drivers/dev_null.o \
./NuttX/nuttx/drivers/dev_zero.o \
./NuttX/nuttx/drivers/loop.o \
./NuttX/nuttx/drivers/pwm.o \
./NuttX/nuttx/drivers/ramdisk.o \
./NuttX/nuttx/drivers/rwbuffer.o \
./NuttX/nuttx/drivers/watchdog.o 

C_DEPS += \
./NuttX/nuttx/drivers/can.d \
./NuttX/nuttx/drivers/dev_null.d \
./NuttX/nuttx/drivers/dev_zero.d \
./NuttX/nuttx/drivers/loop.d \
./NuttX/nuttx/drivers/pwm.d \
./NuttX/nuttx/drivers/ramdisk.d \
./NuttX/nuttx/drivers/rwbuffer.d \
./NuttX/nuttx/drivers/watchdog.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/drivers/%.o: ../NuttX/nuttx/drivers/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


