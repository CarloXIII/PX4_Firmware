################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/configs/skp16c26/src/up_buttons.c \
../NuttX/nuttx/configs/skp16c26/src/up_lcd.c \
../NuttX/nuttx/configs/skp16c26/src/up_lcdconsole.c \
../NuttX/nuttx/configs/skp16c26/src/up_leds.c 

OBJS += \
./NuttX/nuttx/configs/skp16c26/src/up_buttons.o \
./NuttX/nuttx/configs/skp16c26/src/up_lcd.o \
./NuttX/nuttx/configs/skp16c26/src/up_lcdconsole.o \
./NuttX/nuttx/configs/skp16c26/src/up_leds.o 

C_DEPS += \
./NuttX/nuttx/configs/skp16c26/src/up_buttons.d \
./NuttX/nuttx/configs/skp16c26/src/up_lcd.d \
./NuttX/nuttx/configs/skp16c26/src/up_lcdconsole.d \
./NuttX/nuttx/configs/skp16c26/src/up_leds.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/configs/skp16c26/src/%.o: ../NuttX/nuttx/configs/skp16c26/src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


