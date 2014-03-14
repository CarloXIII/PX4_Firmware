################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/configs/stm32ldiscovery/src/stm32_autoleds.c \
../NuttX/nuttx/configs/stm32ldiscovery/src/stm32_boot.c \
../NuttX/nuttx/configs/stm32ldiscovery/src/stm32_buttons.c \
../NuttX/nuttx/configs/stm32ldiscovery/src/stm32_cxxinitialize.c \
../NuttX/nuttx/configs/stm32ldiscovery/src/stm32_lcd.c \
../NuttX/nuttx/configs/stm32ldiscovery/src/stm32_nsh.c \
../NuttX/nuttx/configs/stm32ldiscovery/src/stm32_pwm.c \
../NuttX/nuttx/configs/stm32ldiscovery/src/stm32_qencoder.c \
../NuttX/nuttx/configs/stm32ldiscovery/src/stm32_spi.c \
../NuttX/nuttx/configs/stm32ldiscovery/src/stm32_userleds.c \
../NuttX/nuttx/configs/stm32ldiscovery/src/stm32_watchdog.c 

OBJS += \
./NuttX/nuttx/configs/stm32ldiscovery/src/stm32_autoleds.o \
./NuttX/nuttx/configs/stm32ldiscovery/src/stm32_boot.o \
./NuttX/nuttx/configs/stm32ldiscovery/src/stm32_buttons.o \
./NuttX/nuttx/configs/stm32ldiscovery/src/stm32_cxxinitialize.o \
./NuttX/nuttx/configs/stm32ldiscovery/src/stm32_lcd.o \
./NuttX/nuttx/configs/stm32ldiscovery/src/stm32_nsh.o \
./NuttX/nuttx/configs/stm32ldiscovery/src/stm32_pwm.o \
./NuttX/nuttx/configs/stm32ldiscovery/src/stm32_qencoder.o \
./NuttX/nuttx/configs/stm32ldiscovery/src/stm32_spi.o \
./NuttX/nuttx/configs/stm32ldiscovery/src/stm32_userleds.o \
./NuttX/nuttx/configs/stm32ldiscovery/src/stm32_watchdog.o 

C_DEPS += \
./NuttX/nuttx/configs/stm32ldiscovery/src/stm32_autoleds.d \
./NuttX/nuttx/configs/stm32ldiscovery/src/stm32_boot.d \
./NuttX/nuttx/configs/stm32ldiscovery/src/stm32_buttons.d \
./NuttX/nuttx/configs/stm32ldiscovery/src/stm32_cxxinitialize.d \
./NuttX/nuttx/configs/stm32ldiscovery/src/stm32_lcd.d \
./NuttX/nuttx/configs/stm32ldiscovery/src/stm32_nsh.d \
./NuttX/nuttx/configs/stm32ldiscovery/src/stm32_pwm.d \
./NuttX/nuttx/configs/stm32ldiscovery/src/stm32_qencoder.d \
./NuttX/nuttx/configs/stm32ldiscovery/src/stm32_spi.d \
./NuttX/nuttx/configs/stm32ldiscovery/src/stm32_userleds.d \
./NuttX/nuttx/configs/stm32ldiscovery/src/stm32_watchdog.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/configs/stm32ldiscovery/src/%.o: ../NuttX/nuttx/configs/stm32ldiscovery/src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


