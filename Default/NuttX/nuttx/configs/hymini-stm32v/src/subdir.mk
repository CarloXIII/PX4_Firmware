################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/configs/hymini-stm32v/src/up_boot.c \
../NuttX/nuttx/configs/hymini-stm32v/src/up_buttons.c \
../NuttX/nuttx/configs/hymini-stm32v/src/up_leds.c \
../NuttX/nuttx/configs/hymini-stm32v/src/up_nsh.c \
../NuttX/nuttx/configs/hymini-stm32v/src/up_r61505u.c \
../NuttX/nuttx/configs/hymini-stm32v/src/up_spi.c \
../NuttX/nuttx/configs/hymini-stm32v/src/up_ssd1289.c \
../NuttX/nuttx/configs/hymini-stm32v/src/up_ts.c \
../NuttX/nuttx/configs/hymini-stm32v/src/up_usbdev.c \
../NuttX/nuttx/configs/hymini-stm32v/src/up_usbmsc.c \
../NuttX/nuttx/configs/hymini-stm32v/src/up_watchdog.c 

OBJS += \
./NuttX/nuttx/configs/hymini-stm32v/src/up_boot.o \
./NuttX/nuttx/configs/hymini-stm32v/src/up_buttons.o \
./NuttX/nuttx/configs/hymini-stm32v/src/up_leds.o \
./NuttX/nuttx/configs/hymini-stm32v/src/up_nsh.o \
./NuttX/nuttx/configs/hymini-stm32v/src/up_r61505u.o \
./NuttX/nuttx/configs/hymini-stm32v/src/up_spi.o \
./NuttX/nuttx/configs/hymini-stm32v/src/up_ssd1289.o \
./NuttX/nuttx/configs/hymini-stm32v/src/up_ts.o \
./NuttX/nuttx/configs/hymini-stm32v/src/up_usbdev.o \
./NuttX/nuttx/configs/hymini-stm32v/src/up_usbmsc.o \
./NuttX/nuttx/configs/hymini-stm32v/src/up_watchdog.o 

C_DEPS += \
./NuttX/nuttx/configs/hymini-stm32v/src/up_boot.d \
./NuttX/nuttx/configs/hymini-stm32v/src/up_buttons.d \
./NuttX/nuttx/configs/hymini-stm32v/src/up_leds.d \
./NuttX/nuttx/configs/hymini-stm32v/src/up_nsh.d \
./NuttX/nuttx/configs/hymini-stm32v/src/up_r61505u.d \
./NuttX/nuttx/configs/hymini-stm32v/src/up_spi.d \
./NuttX/nuttx/configs/hymini-stm32v/src/up_ssd1289.d \
./NuttX/nuttx/configs/hymini-stm32v/src/up_ts.d \
./NuttX/nuttx/configs/hymini-stm32v/src/up_usbdev.d \
./NuttX/nuttx/configs/hymini-stm32v/src/up_usbmsc.d \
./NuttX/nuttx/configs/hymini-stm32v/src/up_watchdog.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/configs/hymini-stm32v/src/%.o: ../NuttX/nuttx/configs/hymini-stm32v/src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


