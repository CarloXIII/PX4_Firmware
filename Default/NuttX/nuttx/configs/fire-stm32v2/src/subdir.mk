################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/configs/fire-stm32v2/src/up_autoleds.c \
../NuttX/nuttx/configs/fire-stm32v2/src/up_boot.c \
../NuttX/nuttx/configs/fire-stm32v2/src/up_buttons.c \
../NuttX/nuttx/configs/fire-stm32v2/src/up_composite.c \
../NuttX/nuttx/configs/fire-stm32v2/src/up_cxxinitialize.c \
../NuttX/nuttx/configs/fire-stm32v2/src/up_enc28j60.c \
../NuttX/nuttx/configs/fire-stm32v2/src/up_mmcsd.c \
../NuttX/nuttx/configs/fire-stm32v2/src/up_nsh.c \
../NuttX/nuttx/configs/fire-stm32v2/src/up_selectlcd.c \
../NuttX/nuttx/configs/fire-stm32v2/src/up_spi.c \
../NuttX/nuttx/configs/fire-stm32v2/src/up_usbdev.c \
../NuttX/nuttx/configs/fire-stm32v2/src/up_usbmsc.c \
../NuttX/nuttx/configs/fire-stm32v2/src/up_userleds.c \
../NuttX/nuttx/configs/fire-stm32v2/src/up_w25.c \
../NuttX/nuttx/configs/fire-stm32v2/src/up_watchdog.c 

OBJS += \
./NuttX/nuttx/configs/fire-stm32v2/src/up_autoleds.o \
./NuttX/nuttx/configs/fire-stm32v2/src/up_boot.o \
./NuttX/nuttx/configs/fire-stm32v2/src/up_buttons.o \
./NuttX/nuttx/configs/fire-stm32v2/src/up_composite.o \
./NuttX/nuttx/configs/fire-stm32v2/src/up_cxxinitialize.o \
./NuttX/nuttx/configs/fire-stm32v2/src/up_enc28j60.o \
./NuttX/nuttx/configs/fire-stm32v2/src/up_mmcsd.o \
./NuttX/nuttx/configs/fire-stm32v2/src/up_nsh.o \
./NuttX/nuttx/configs/fire-stm32v2/src/up_selectlcd.o \
./NuttX/nuttx/configs/fire-stm32v2/src/up_spi.o \
./NuttX/nuttx/configs/fire-stm32v2/src/up_usbdev.o \
./NuttX/nuttx/configs/fire-stm32v2/src/up_usbmsc.o \
./NuttX/nuttx/configs/fire-stm32v2/src/up_userleds.o \
./NuttX/nuttx/configs/fire-stm32v2/src/up_w25.o \
./NuttX/nuttx/configs/fire-stm32v2/src/up_watchdog.o 

C_DEPS += \
./NuttX/nuttx/configs/fire-stm32v2/src/up_autoleds.d \
./NuttX/nuttx/configs/fire-stm32v2/src/up_boot.d \
./NuttX/nuttx/configs/fire-stm32v2/src/up_buttons.d \
./NuttX/nuttx/configs/fire-stm32v2/src/up_composite.d \
./NuttX/nuttx/configs/fire-stm32v2/src/up_cxxinitialize.d \
./NuttX/nuttx/configs/fire-stm32v2/src/up_enc28j60.d \
./NuttX/nuttx/configs/fire-stm32v2/src/up_mmcsd.d \
./NuttX/nuttx/configs/fire-stm32v2/src/up_nsh.d \
./NuttX/nuttx/configs/fire-stm32v2/src/up_selectlcd.d \
./NuttX/nuttx/configs/fire-stm32v2/src/up_spi.d \
./NuttX/nuttx/configs/fire-stm32v2/src/up_usbdev.d \
./NuttX/nuttx/configs/fire-stm32v2/src/up_usbmsc.d \
./NuttX/nuttx/configs/fire-stm32v2/src/up_userleds.d \
./NuttX/nuttx/configs/fire-stm32v2/src/up_w25.d \
./NuttX/nuttx/configs/fire-stm32v2/src/up_watchdog.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/configs/fire-stm32v2/src/%.o: ../NuttX/nuttx/configs/fire-stm32v2/src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


