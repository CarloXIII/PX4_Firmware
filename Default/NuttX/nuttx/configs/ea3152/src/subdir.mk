################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/configs/ea3152/src/up_boot.c \
../NuttX/nuttx/configs/ea3152/src/up_buttons.c \
../NuttX/nuttx/configs/ea3152/src/up_clkinit.c \
../NuttX/nuttx/configs/ea3152/src/up_fillpage.c \
../NuttX/nuttx/configs/ea3152/src/up_leds.c \
../NuttX/nuttx/configs/ea3152/src/up_mem.c \
../NuttX/nuttx/configs/ea3152/src/up_nsh.c \
../NuttX/nuttx/configs/ea3152/src/up_spi.c \
../NuttX/nuttx/configs/ea3152/src/up_usbmsc.c 

OBJS += \
./NuttX/nuttx/configs/ea3152/src/up_boot.o \
./NuttX/nuttx/configs/ea3152/src/up_buttons.o \
./NuttX/nuttx/configs/ea3152/src/up_clkinit.o \
./NuttX/nuttx/configs/ea3152/src/up_fillpage.o \
./NuttX/nuttx/configs/ea3152/src/up_leds.o \
./NuttX/nuttx/configs/ea3152/src/up_mem.o \
./NuttX/nuttx/configs/ea3152/src/up_nsh.o \
./NuttX/nuttx/configs/ea3152/src/up_spi.o \
./NuttX/nuttx/configs/ea3152/src/up_usbmsc.o 

C_DEPS += \
./NuttX/nuttx/configs/ea3152/src/up_boot.d \
./NuttX/nuttx/configs/ea3152/src/up_buttons.d \
./NuttX/nuttx/configs/ea3152/src/up_clkinit.d \
./NuttX/nuttx/configs/ea3152/src/up_fillpage.d \
./NuttX/nuttx/configs/ea3152/src/up_leds.d \
./NuttX/nuttx/configs/ea3152/src/up_mem.d \
./NuttX/nuttx/configs/ea3152/src/up_nsh.d \
./NuttX/nuttx/configs/ea3152/src/up_spi.d \
./NuttX/nuttx/configs/ea3152/src/up_usbmsc.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/configs/ea3152/src/%.o: ../NuttX/nuttx/configs/ea3152/src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


