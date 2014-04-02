################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/configs/mikroe-stm32f4/src/up_boot.c \
../NuttX/nuttx/configs/mikroe-stm32f4/src/up_clockconfig.c \
../NuttX/nuttx/configs/mikroe-stm32f4/src/up_cxxinitialize.c \
../NuttX/nuttx/configs/mikroe-stm32f4/src/up_extmem.c \
../NuttX/nuttx/configs/mikroe-stm32f4/src/up_idle.c \
../NuttX/nuttx/configs/mikroe-stm32f4/src/up_mio283qt2.c \
../NuttX/nuttx/configs/mikroe-stm32f4/src/up_nsh.c \
../NuttX/nuttx/configs/mikroe-stm32f4/src/up_pm.c \
../NuttX/nuttx/configs/mikroe-stm32f4/src/up_pwm.c \
../NuttX/nuttx/configs/mikroe-stm32f4/src/up_qencoder.c \
../NuttX/nuttx/configs/mikroe-stm32f4/src/up_spi.c \
../NuttX/nuttx/configs/mikroe-stm32f4/src/up_touchscreen.c \
../NuttX/nuttx/configs/mikroe-stm32f4/src/up_usb.c \
../NuttX/nuttx/configs/mikroe-stm32f4/src/up_vs1053.c \
../NuttX/nuttx/configs/mikroe-stm32f4/src/up_watchdog.c 

OBJS += \
./NuttX/nuttx/configs/mikroe-stm32f4/src/up_boot.o \
./NuttX/nuttx/configs/mikroe-stm32f4/src/up_clockconfig.o \
./NuttX/nuttx/configs/mikroe-stm32f4/src/up_cxxinitialize.o \
./NuttX/nuttx/configs/mikroe-stm32f4/src/up_extmem.o \
./NuttX/nuttx/configs/mikroe-stm32f4/src/up_idle.o \
./NuttX/nuttx/configs/mikroe-stm32f4/src/up_mio283qt2.o \
./NuttX/nuttx/configs/mikroe-stm32f4/src/up_nsh.o \
./NuttX/nuttx/configs/mikroe-stm32f4/src/up_pm.o \
./NuttX/nuttx/configs/mikroe-stm32f4/src/up_pwm.o \
./NuttX/nuttx/configs/mikroe-stm32f4/src/up_qencoder.o \
./NuttX/nuttx/configs/mikroe-stm32f4/src/up_spi.o \
./NuttX/nuttx/configs/mikroe-stm32f4/src/up_touchscreen.o \
./NuttX/nuttx/configs/mikroe-stm32f4/src/up_usb.o \
./NuttX/nuttx/configs/mikroe-stm32f4/src/up_vs1053.o \
./NuttX/nuttx/configs/mikroe-stm32f4/src/up_watchdog.o 

C_DEPS += \
./NuttX/nuttx/configs/mikroe-stm32f4/src/up_boot.d \
./NuttX/nuttx/configs/mikroe-stm32f4/src/up_clockconfig.d \
./NuttX/nuttx/configs/mikroe-stm32f4/src/up_cxxinitialize.d \
./NuttX/nuttx/configs/mikroe-stm32f4/src/up_extmem.d \
./NuttX/nuttx/configs/mikroe-stm32f4/src/up_idle.d \
./NuttX/nuttx/configs/mikroe-stm32f4/src/up_mio283qt2.d \
./NuttX/nuttx/configs/mikroe-stm32f4/src/up_nsh.d \
./NuttX/nuttx/configs/mikroe-stm32f4/src/up_pm.d \
./NuttX/nuttx/configs/mikroe-stm32f4/src/up_pwm.d \
./NuttX/nuttx/configs/mikroe-stm32f4/src/up_qencoder.d \
./NuttX/nuttx/configs/mikroe-stm32f4/src/up_spi.d \
./NuttX/nuttx/configs/mikroe-stm32f4/src/up_touchscreen.d \
./NuttX/nuttx/configs/mikroe-stm32f4/src/up_usb.d \
./NuttX/nuttx/configs/mikroe-stm32f4/src/up_vs1053.d \
./NuttX/nuttx/configs/mikroe-stm32f4/src/up_watchdog.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/configs/mikroe-stm32f4/src/%.o: ../NuttX/nuttx/configs/mikroe-stm32f4/src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


