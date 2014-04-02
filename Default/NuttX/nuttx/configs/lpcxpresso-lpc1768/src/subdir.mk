################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/configs/lpcxpresso-lpc1768/src/up_boot.c \
../NuttX/nuttx/configs/lpcxpresso-lpc1768/src/up_leds.c \
../NuttX/nuttx/configs/lpcxpresso-lpc1768/src/up_nsh.c \
../NuttX/nuttx/configs/lpcxpresso-lpc1768/src/up_oled.c \
../NuttX/nuttx/configs/lpcxpresso-lpc1768/src/up_ssp.c \
../NuttX/nuttx/configs/lpcxpresso-lpc1768/src/up_usbmsc.c 

OBJS += \
./NuttX/nuttx/configs/lpcxpresso-lpc1768/src/up_boot.o \
./NuttX/nuttx/configs/lpcxpresso-lpc1768/src/up_leds.o \
./NuttX/nuttx/configs/lpcxpresso-lpc1768/src/up_nsh.o \
./NuttX/nuttx/configs/lpcxpresso-lpc1768/src/up_oled.o \
./NuttX/nuttx/configs/lpcxpresso-lpc1768/src/up_ssp.o \
./NuttX/nuttx/configs/lpcxpresso-lpc1768/src/up_usbmsc.o 

C_DEPS += \
./NuttX/nuttx/configs/lpcxpresso-lpc1768/src/up_boot.d \
./NuttX/nuttx/configs/lpcxpresso-lpc1768/src/up_leds.d \
./NuttX/nuttx/configs/lpcxpresso-lpc1768/src/up_nsh.d \
./NuttX/nuttx/configs/lpcxpresso-lpc1768/src/up_oled.d \
./NuttX/nuttx/configs/lpcxpresso-lpc1768/src/up_ssp.d \
./NuttX/nuttx/configs/lpcxpresso-lpc1768/src/up_usbmsc.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/configs/lpcxpresso-lpc1768/src/%.o: ../NuttX/nuttx/configs/lpcxpresso-lpc1768/src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


