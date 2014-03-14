################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/configs/sam3u-ek/src/up_boot.c \
../NuttX/nuttx/configs/sam3u-ek/src/up_buttons.c \
../NuttX/nuttx/configs/sam3u-ek/src/up_lcd.c \
../NuttX/nuttx/configs/sam3u-ek/src/up_leds.c \
../NuttX/nuttx/configs/sam3u-ek/src/up_mmcsd.c \
../NuttX/nuttx/configs/sam3u-ek/src/up_nsh.c \
../NuttX/nuttx/configs/sam3u-ek/src/up_spi.c \
../NuttX/nuttx/configs/sam3u-ek/src/up_touchscreen.c \
../NuttX/nuttx/configs/sam3u-ek/src/up_usbdev.c \
../NuttX/nuttx/configs/sam3u-ek/src/up_usbmsc.c 

OBJS += \
./NuttX/nuttx/configs/sam3u-ek/src/up_boot.o \
./NuttX/nuttx/configs/sam3u-ek/src/up_buttons.o \
./NuttX/nuttx/configs/sam3u-ek/src/up_lcd.o \
./NuttX/nuttx/configs/sam3u-ek/src/up_leds.o \
./NuttX/nuttx/configs/sam3u-ek/src/up_mmcsd.o \
./NuttX/nuttx/configs/sam3u-ek/src/up_nsh.o \
./NuttX/nuttx/configs/sam3u-ek/src/up_spi.o \
./NuttX/nuttx/configs/sam3u-ek/src/up_touchscreen.o \
./NuttX/nuttx/configs/sam3u-ek/src/up_usbdev.o \
./NuttX/nuttx/configs/sam3u-ek/src/up_usbmsc.o 

C_DEPS += \
./NuttX/nuttx/configs/sam3u-ek/src/up_boot.d \
./NuttX/nuttx/configs/sam3u-ek/src/up_buttons.d \
./NuttX/nuttx/configs/sam3u-ek/src/up_lcd.d \
./NuttX/nuttx/configs/sam3u-ek/src/up_leds.d \
./NuttX/nuttx/configs/sam3u-ek/src/up_mmcsd.d \
./NuttX/nuttx/configs/sam3u-ek/src/up_nsh.d \
./NuttX/nuttx/configs/sam3u-ek/src/up_spi.d \
./NuttX/nuttx/configs/sam3u-ek/src/up_touchscreen.d \
./NuttX/nuttx/configs/sam3u-ek/src/up_usbdev.d \
./NuttX/nuttx/configs/sam3u-ek/src/up_usbmsc.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/configs/sam3u-ek/src/%.o: ../NuttX/nuttx/configs/sam3u-ek/src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


