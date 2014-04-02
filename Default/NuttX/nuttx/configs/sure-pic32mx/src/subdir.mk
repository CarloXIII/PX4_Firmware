################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/configs/sure-pic32mx/src/pic32mx_autoleds.c \
../NuttX/nuttx/configs/sure-pic32mx/src/pic32mx_boot.c \
../NuttX/nuttx/configs/sure-pic32mx/src/pic32mx_buttons.c \
../NuttX/nuttx/configs/sure-pic32mx/src/pic32mx_lcd1602.c \
../NuttX/nuttx/configs/sure-pic32mx/src/pic32mx_nsh.c \
../NuttX/nuttx/configs/sure-pic32mx/src/pic32mx_spi.c \
../NuttX/nuttx/configs/sure-pic32mx/src/pic32mx_usbdev.c \
../NuttX/nuttx/configs/sure-pic32mx/src/pic32mx_usbterm.c 

OBJS += \
./NuttX/nuttx/configs/sure-pic32mx/src/pic32mx_autoleds.o \
./NuttX/nuttx/configs/sure-pic32mx/src/pic32mx_boot.o \
./NuttX/nuttx/configs/sure-pic32mx/src/pic32mx_buttons.o \
./NuttX/nuttx/configs/sure-pic32mx/src/pic32mx_lcd1602.o \
./NuttX/nuttx/configs/sure-pic32mx/src/pic32mx_nsh.o \
./NuttX/nuttx/configs/sure-pic32mx/src/pic32mx_spi.o \
./NuttX/nuttx/configs/sure-pic32mx/src/pic32mx_usbdev.o \
./NuttX/nuttx/configs/sure-pic32mx/src/pic32mx_usbterm.o 

C_DEPS += \
./NuttX/nuttx/configs/sure-pic32mx/src/pic32mx_autoleds.d \
./NuttX/nuttx/configs/sure-pic32mx/src/pic32mx_boot.d \
./NuttX/nuttx/configs/sure-pic32mx/src/pic32mx_buttons.d \
./NuttX/nuttx/configs/sure-pic32mx/src/pic32mx_lcd1602.d \
./NuttX/nuttx/configs/sure-pic32mx/src/pic32mx_nsh.d \
./NuttX/nuttx/configs/sure-pic32mx/src/pic32mx_spi.d \
./NuttX/nuttx/configs/sure-pic32mx/src/pic32mx_usbdev.d \
./NuttX/nuttx/configs/sure-pic32mx/src/pic32mx_usbterm.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/configs/sure-pic32mx/src/%.o: ../NuttX/nuttx/configs/sure-pic32mx/src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


