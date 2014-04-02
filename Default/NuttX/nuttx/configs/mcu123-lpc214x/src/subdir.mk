################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/configs/mcu123-lpc214x/src/up_composite.c \
../NuttX/nuttx/configs/mcu123-lpc214x/src/up_leds.c \
../NuttX/nuttx/configs/mcu123-lpc214x/src/up_nsh.c \
../NuttX/nuttx/configs/mcu123-lpc214x/src/up_spi1.c \
../NuttX/nuttx/configs/mcu123-lpc214x/src/up_usbmsc.c 

OBJS += \
./NuttX/nuttx/configs/mcu123-lpc214x/src/up_composite.o \
./NuttX/nuttx/configs/mcu123-lpc214x/src/up_leds.o \
./NuttX/nuttx/configs/mcu123-lpc214x/src/up_nsh.o \
./NuttX/nuttx/configs/mcu123-lpc214x/src/up_spi1.o \
./NuttX/nuttx/configs/mcu123-lpc214x/src/up_usbmsc.o 

C_DEPS += \
./NuttX/nuttx/configs/mcu123-lpc214x/src/up_composite.d \
./NuttX/nuttx/configs/mcu123-lpc214x/src/up_leds.d \
./NuttX/nuttx/configs/mcu123-lpc214x/src/up_nsh.d \
./NuttX/nuttx/configs/mcu123-lpc214x/src/up_spi1.d \
./NuttX/nuttx/configs/mcu123-lpc214x/src/up_usbmsc.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/configs/mcu123-lpc214x/src/%.o: ../NuttX/nuttx/configs/mcu123-lpc214x/src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


