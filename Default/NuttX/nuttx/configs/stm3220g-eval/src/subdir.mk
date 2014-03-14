################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/configs/stm3220g-eval/src/up_adc.c \
../NuttX/nuttx/configs/stm3220g-eval/src/up_autoleds.c \
../NuttX/nuttx/configs/stm3220g-eval/src/up_boot.c \
../NuttX/nuttx/configs/stm3220g-eval/src/up_buttons.c \
../NuttX/nuttx/configs/stm3220g-eval/src/up_can.c \
../NuttX/nuttx/configs/stm3220g-eval/src/up_cxxinitialize.c \
../NuttX/nuttx/configs/stm3220g-eval/src/up_deselectlcd.c \
../NuttX/nuttx/configs/stm3220g-eval/src/up_deselectsram.c \
../NuttX/nuttx/configs/stm3220g-eval/src/up_extmem.c \
../NuttX/nuttx/configs/stm3220g-eval/src/up_lcd.c \
../NuttX/nuttx/configs/stm3220g-eval/src/up_nsh.c \
../NuttX/nuttx/configs/stm3220g-eval/src/up_pwm.c \
../NuttX/nuttx/configs/stm3220g-eval/src/up_selectlcd.c \
../NuttX/nuttx/configs/stm3220g-eval/src/up_selectsram.c \
../NuttX/nuttx/configs/stm3220g-eval/src/up_spi.c \
../NuttX/nuttx/configs/stm3220g-eval/src/up_stmpe811.c \
../NuttX/nuttx/configs/stm3220g-eval/src/up_usb.c \
../NuttX/nuttx/configs/stm3220g-eval/src/up_userleds.c \
../NuttX/nuttx/configs/stm3220g-eval/src/up_watchdog.c 

OBJS += \
./NuttX/nuttx/configs/stm3220g-eval/src/up_adc.o \
./NuttX/nuttx/configs/stm3220g-eval/src/up_autoleds.o \
./NuttX/nuttx/configs/stm3220g-eval/src/up_boot.o \
./NuttX/nuttx/configs/stm3220g-eval/src/up_buttons.o \
./NuttX/nuttx/configs/stm3220g-eval/src/up_can.o \
./NuttX/nuttx/configs/stm3220g-eval/src/up_cxxinitialize.o \
./NuttX/nuttx/configs/stm3220g-eval/src/up_deselectlcd.o \
./NuttX/nuttx/configs/stm3220g-eval/src/up_deselectsram.o \
./NuttX/nuttx/configs/stm3220g-eval/src/up_extmem.o \
./NuttX/nuttx/configs/stm3220g-eval/src/up_lcd.o \
./NuttX/nuttx/configs/stm3220g-eval/src/up_nsh.o \
./NuttX/nuttx/configs/stm3220g-eval/src/up_pwm.o \
./NuttX/nuttx/configs/stm3220g-eval/src/up_selectlcd.o \
./NuttX/nuttx/configs/stm3220g-eval/src/up_selectsram.o \
./NuttX/nuttx/configs/stm3220g-eval/src/up_spi.o \
./NuttX/nuttx/configs/stm3220g-eval/src/up_stmpe811.o \
./NuttX/nuttx/configs/stm3220g-eval/src/up_usb.o \
./NuttX/nuttx/configs/stm3220g-eval/src/up_userleds.o \
./NuttX/nuttx/configs/stm3220g-eval/src/up_watchdog.o 

C_DEPS += \
./NuttX/nuttx/configs/stm3220g-eval/src/up_adc.d \
./NuttX/nuttx/configs/stm3220g-eval/src/up_autoleds.d \
./NuttX/nuttx/configs/stm3220g-eval/src/up_boot.d \
./NuttX/nuttx/configs/stm3220g-eval/src/up_buttons.d \
./NuttX/nuttx/configs/stm3220g-eval/src/up_can.d \
./NuttX/nuttx/configs/stm3220g-eval/src/up_cxxinitialize.d \
./NuttX/nuttx/configs/stm3220g-eval/src/up_deselectlcd.d \
./NuttX/nuttx/configs/stm3220g-eval/src/up_deselectsram.d \
./NuttX/nuttx/configs/stm3220g-eval/src/up_extmem.d \
./NuttX/nuttx/configs/stm3220g-eval/src/up_lcd.d \
./NuttX/nuttx/configs/stm3220g-eval/src/up_nsh.d \
./NuttX/nuttx/configs/stm3220g-eval/src/up_pwm.d \
./NuttX/nuttx/configs/stm3220g-eval/src/up_selectlcd.d \
./NuttX/nuttx/configs/stm3220g-eval/src/up_selectsram.d \
./NuttX/nuttx/configs/stm3220g-eval/src/up_spi.d \
./NuttX/nuttx/configs/stm3220g-eval/src/up_stmpe811.d \
./NuttX/nuttx/configs/stm3220g-eval/src/up_usb.d \
./NuttX/nuttx/configs/stm3220g-eval/src/up_userleds.d \
./NuttX/nuttx/configs/stm3220g-eval/src/up_watchdog.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/configs/stm3220g-eval/src/%.o: ../NuttX/nuttx/configs/stm3220g-eval/src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


