################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/configs/stm32_tiny/src/up_boot.c \
../NuttX/nuttx/configs/stm32_tiny/src/up_leds.c \
../NuttX/nuttx/configs/stm32_tiny/src/up_nsh.c \
../NuttX/nuttx/configs/stm32_tiny/src/up_pwm.c \
../NuttX/nuttx/configs/stm32_tiny/src/up_spi.c \
../NuttX/nuttx/configs/stm32_tiny/src/up_usbdev.c \
../NuttX/nuttx/configs/stm32_tiny/src/up_watchdog.c \
../NuttX/nuttx/configs/stm32_tiny/src/up_wireless.c 

OBJS += \
./NuttX/nuttx/configs/stm32_tiny/src/up_boot.o \
./NuttX/nuttx/configs/stm32_tiny/src/up_leds.o \
./NuttX/nuttx/configs/stm32_tiny/src/up_nsh.o \
./NuttX/nuttx/configs/stm32_tiny/src/up_pwm.o \
./NuttX/nuttx/configs/stm32_tiny/src/up_spi.o \
./NuttX/nuttx/configs/stm32_tiny/src/up_usbdev.o \
./NuttX/nuttx/configs/stm32_tiny/src/up_watchdog.o \
./NuttX/nuttx/configs/stm32_tiny/src/up_wireless.o 

C_DEPS += \
./NuttX/nuttx/configs/stm32_tiny/src/up_boot.d \
./NuttX/nuttx/configs/stm32_tiny/src/up_leds.d \
./NuttX/nuttx/configs/stm32_tiny/src/up_nsh.d \
./NuttX/nuttx/configs/stm32_tiny/src/up_pwm.d \
./NuttX/nuttx/configs/stm32_tiny/src/up_spi.d \
./NuttX/nuttx/configs/stm32_tiny/src/up_usbdev.d \
./NuttX/nuttx/configs/stm32_tiny/src/up_watchdog.d \
./NuttX/nuttx/configs/stm32_tiny/src/up_wireless.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/configs/stm32_tiny/src/%.o: ../NuttX/nuttx/configs/stm32_tiny/src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


