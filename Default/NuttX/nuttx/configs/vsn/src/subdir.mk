################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/configs/vsn/src/boot.c \
../NuttX/nuttx/configs/vsn/src/buttons.c \
../NuttX/nuttx/configs/vsn/src/chipcon.c \
../NuttX/nuttx/configs/vsn/src/leds.c \
../NuttX/nuttx/configs/vsn/src/muxbus.c \
../NuttX/nuttx/configs/vsn/src/power.c \
../NuttX/nuttx/configs/vsn/src/rtac.c \
../NuttX/nuttx/configs/vsn/src/sif.c \
../NuttX/nuttx/configs/vsn/src/spi.c \
../NuttX/nuttx/configs/vsn/src/sysclock.c \
../NuttX/nuttx/configs/vsn/src/usbdev.c \
../NuttX/nuttx/configs/vsn/src/usbmsc.c 

OBJS += \
./NuttX/nuttx/configs/vsn/src/boot.o \
./NuttX/nuttx/configs/vsn/src/buttons.o \
./NuttX/nuttx/configs/vsn/src/chipcon.o \
./NuttX/nuttx/configs/vsn/src/leds.o \
./NuttX/nuttx/configs/vsn/src/muxbus.o \
./NuttX/nuttx/configs/vsn/src/power.o \
./NuttX/nuttx/configs/vsn/src/rtac.o \
./NuttX/nuttx/configs/vsn/src/sif.o \
./NuttX/nuttx/configs/vsn/src/spi.o \
./NuttX/nuttx/configs/vsn/src/sysclock.o \
./NuttX/nuttx/configs/vsn/src/usbdev.o \
./NuttX/nuttx/configs/vsn/src/usbmsc.o 

C_DEPS += \
./NuttX/nuttx/configs/vsn/src/boot.d \
./NuttX/nuttx/configs/vsn/src/buttons.d \
./NuttX/nuttx/configs/vsn/src/chipcon.d \
./NuttX/nuttx/configs/vsn/src/leds.d \
./NuttX/nuttx/configs/vsn/src/muxbus.d \
./NuttX/nuttx/configs/vsn/src/power.d \
./NuttX/nuttx/configs/vsn/src/rtac.d \
./NuttX/nuttx/configs/vsn/src/sif.d \
./NuttX/nuttx/configs/vsn/src/spi.d \
./NuttX/nuttx/configs/vsn/src/sysclock.d \
./NuttX/nuttx/configs/vsn/src/usbdev.d \
./NuttX/nuttx/configs/vsn/src/usbmsc.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/configs/vsn/src/%.o: ../NuttX/nuttx/configs/vsn/src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


