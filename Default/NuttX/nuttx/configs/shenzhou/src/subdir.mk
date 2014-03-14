################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/configs/shenzhou/src/up_adc.c \
../NuttX/nuttx/configs/shenzhou/src/up_autoleds.c \
../NuttX/nuttx/configs/shenzhou/src/up_boot.c \
../NuttX/nuttx/configs/shenzhou/src/up_buttons.c \
../NuttX/nuttx/configs/shenzhou/src/up_can.c \
../NuttX/nuttx/configs/shenzhou/src/up_chipid.c \
../NuttX/nuttx/configs/shenzhou/src/up_composite.c \
../NuttX/nuttx/configs/shenzhou/src/up_cxxinitialize.c \
../NuttX/nuttx/configs/shenzhou/src/up_ili93xx.c \
../NuttX/nuttx/configs/shenzhou/src/up_mmcsd.c \
../NuttX/nuttx/configs/shenzhou/src/up_nsh.c \
../NuttX/nuttx/configs/shenzhou/src/up_relays.c \
../NuttX/nuttx/configs/shenzhou/src/up_spi.c \
../NuttX/nuttx/configs/shenzhou/src/up_ssd1289.c \
../NuttX/nuttx/configs/shenzhou/src/up_touchscreen.c \
../NuttX/nuttx/configs/shenzhou/src/up_usb.c \
../NuttX/nuttx/configs/shenzhou/src/up_usbmsc.c \
../NuttX/nuttx/configs/shenzhou/src/up_userleds.c \
../NuttX/nuttx/configs/shenzhou/src/up_w25.c \
../NuttX/nuttx/configs/shenzhou/src/up_watchdog.c 

OBJS += \
./NuttX/nuttx/configs/shenzhou/src/up_adc.o \
./NuttX/nuttx/configs/shenzhou/src/up_autoleds.o \
./NuttX/nuttx/configs/shenzhou/src/up_boot.o \
./NuttX/nuttx/configs/shenzhou/src/up_buttons.o \
./NuttX/nuttx/configs/shenzhou/src/up_can.o \
./NuttX/nuttx/configs/shenzhou/src/up_chipid.o \
./NuttX/nuttx/configs/shenzhou/src/up_composite.o \
./NuttX/nuttx/configs/shenzhou/src/up_cxxinitialize.o \
./NuttX/nuttx/configs/shenzhou/src/up_ili93xx.o \
./NuttX/nuttx/configs/shenzhou/src/up_mmcsd.o \
./NuttX/nuttx/configs/shenzhou/src/up_nsh.o \
./NuttX/nuttx/configs/shenzhou/src/up_relays.o \
./NuttX/nuttx/configs/shenzhou/src/up_spi.o \
./NuttX/nuttx/configs/shenzhou/src/up_ssd1289.o \
./NuttX/nuttx/configs/shenzhou/src/up_touchscreen.o \
./NuttX/nuttx/configs/shenzhou/src/up_usb.o \
./NuttX/nuttx/configs/shenzhou/src/up_usbmsc.o \
./NuttX/nuttx/configs/shenzhou/src/up_userleds.o \
./NuttX/nuttx/configs/shenzhou/src/up_w25.o \
./NuttX/nuttx/configs/shenzhou/src/up_watchdog.o 

C_DEPS += \
./NuttX/nuttx/configs/shenzhou/src/up_adc.d \
./NuttX/nuttx/configs/shenzhou/src/up_autoleds.d \
./NuttX/nuttx/configs/shenzhou/src/up_boot.d \
./NuttX/nuttx/configs/shenzhou/src/up_buttons.d \
./NuttX/nuttx/configs/shenzhou/src/up_can.d \
./NuttX/nuttx/configs/shenzhou/src/up_chipid.d \
./NuttX/nuttx/configs/shenzhou/src/up_composite.d \
./NuttX/nuttx/configs/shenzhou/src/up_cxxinitialize.d \
./NuttX/nuttx/configs/shenzhou/src/up_ili93xx.d \
./NuttX/nuttx/configs/shenzhou/src/up_mmcsd.d \
./NuttX/nuttx/configs/shenzhou/src/up_nsh.d \
./NuttX/nuttx/configs/shenzhou/src/up_relays.d \
./NuttX/nuttx/configs/shenzhou/src/up_spi.d \
./NuttX/nuttx/configs/shenzhou/src/up_ssd1289.d \
./NuttX/nuttx/configs/shenzhou/src/up_touchscreen.d \
./NuttX/nuttx/configs/shenzhou/src/up_usb.d \
./NuttX/nuttx/configs/shenzhou/src/up_usbmsc.d \
./NuttX/nuttx/configs/shenzhou/src/up_userleds.d \
./NuttX/nuttx/configs/shenzhou/src/up_w25.d \
./NuttX/nuttx/configs/shenzhou/src/up_watchdog.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/configs/shenzhou/src/%.o: ../NuttX/nuttx/configs/shenzhou/src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


