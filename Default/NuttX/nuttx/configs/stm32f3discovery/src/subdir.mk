################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/configs/stm32f3discovery/src/up_autoleds.c \
../NuttX/nuttx/configs/stm32f3discovery/src/up_boot.c \
../NuttX/nuttx/configs/stm32f3discovery/src/up_buttons.c \
../NuttX/nuttx/configs/stm32f3discovery/src/up_cxxinitialize.c \
../NuttX/nuttx/configs/stm32f3discovery/src/up_nsh.c \
../NuttX/nuttx/configs/stm32f3discovery/src/up_pwm.c \
../NuttX/nuttx/configs/stm32f3discovery/src/up_qencoder.c \
../NuttX/nuttx/configs/stm32f3discovery/src/up_spi.c \
../NuttX/nuttx/configs/stm32f3discovery/src/up_usb.c \
../NuttX/nuttx/configs/stm32f3discovery/src/up_userleds.c \
../NuttX/nuttx/configs/stm32f3discovery/src/up_watchdog.c 

OBJS += \
./NuttX/nuttx/configs/stm32f3discovery/src/up_autoleds.o \
./NuttX/nuttx/configs/stm32f3discovery/src/up_boot.o \
./NuttX/nuttx/configs/stm32f3discovery/src/up_buttons.o \
./NuttX/nuttx/configs/stm32f3discovery/src/up_cxxinitialize.o \
./NuttX/nuttx/configs/stm32f3discovery/src/up_nsh.o \
./NuttX/nuttx/configs/stm32f3discovery/src/up_pwm.o \
./NuttX/nuttx/configs/stm32f3discovery/src/up_qencoder.o \
./NuttX/nuttx/configs/stm32f3discovery/src/up_spi.o \
./NuttX/nuttx/configs/stm32f3discovery/src/up_usb.o \
./NuttX/nuttx/configs/stm32f3discovery/src/up_userleds.o \
./NuttX/nuttx/configs/stm32f3discovery/src/up_watchdog.o 

C_DEPS += \
./NuttX/nuttx/configs/stm32f3discovery/src/up_autoleds.d \
./NuttX/nuttx/configs/stm32f3discovery/src/up_boot.d \
./NuttX/nuttx/configs/stm32f3discovery/src/up_buttons.d \
./NuttX/nuttx/configs/stm32f3discovery/src/up_cxxinitialize.d \
./NuttX/nuttx/configs/stm32f3discovery/src/up_nsh.d \
./NuttX/nuttx/configs/stm32f3discovery/src/up_pwm.d \
./NuttX/nuttx/configs/stm32f3discovery/src/up_qencoder.d \
./NuttX/nuttx/configs/stm32f3discovery/src/up_spi.d \
./NuttX/nuttx/configs/stm32f3discovery/src/up_usb.d \
./NuttX/nuttx/configs/stm32f3discovery/src/up_userleds.d \
./NuttX/nuttx/configs/stm32f3discovery/src/up_watchdog.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/configs/stm32f3discovery/src/%.o: ../NuttX/nuttx/configs/stm32f3discovery/src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


