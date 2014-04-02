################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/configs/open1788/src/lpc17_autoleds.c \
../NuttX/nuttx/configs/open1788/src/lpc17_boardinitialize.c \
../NuttX/nuttx/configs/open1788/src/lpc17_buttons.c \
../NuttX/nuttx/configs/open1788/src/lpc17_lcd.c \
../NuttX/nuttx/configs/open1788/src/lpc17_nandinitialize.c \
../NuttX/nuttx/configs/open1788/src/lpc17_norinitialize.c \
../NuttX/nuttx/configs/open1788/src/lpc17_nsh.c \
../NuttX/nuttx/configs/open1788/src/lpc17_sdraminitialize.c \
../NuttX/nuttx/configs/open1788/src/lpc17_ssp.c \
../NuttX/nuttx/configs/open1788/src/lpc17_touchscreen.c \
../NuttX/nuttx/configs/open1788/src/lpc17_userleds.c 

OBJS += \
./NuttX/nuttx/configs/open1788/src/lpc17_autoleds.o \
./NuttX/nuttx/configs/open1788/src/lpc17_boardinitialize.o \
./NuttX/nuttx/configs/open1788/src/lpc17_buttons.o \
./NuttX/nuttx/configs/open1788/src/lpc17_lcd.o \
./NuttX/nuttx/configs/open1788/src/lpc17_nandinitialize.o \
./NuttX/nuttx/configs/open1788/src/lpc17_norinitialize.o \
./NuttX/nuttx/configs/open1788/src/lpc17_nsh.o \
./NuttX/nuttx/configs/open1788/src/lpc17_sdraminitialize.o \
./NuttX/nuttx/configs/open1788/src/lpc17_ssp.o \
./NuttX/nuttx/configs/open1788/src/lpc17_touchscreen.o \
./NuttX/nuttx/configs/open1788/src/lpc17_userleds.o 

C_DEPS += \
./NuttX/nuttx/configs/open1788/src/lpc17_autoleds.d \
./NuttX/nuttx/configs/open1788/src/lpc17_boardinitialize.d \
./NuttX/nuttx/configs/open1788/src/lpc17_buttons.d \
./NuttX/nuttx/configs/open1788/src/lpc17_lcd.d \
./NuttX/nuttx/configs/open1788/src/lpc17_nandinitialize.d \
./NuttX/nuttx/configs/open1788/src/lpc17_norinitialize.d \
./NuttX/nuttx/configs/open1788/src/lpc17_nsh.d \
./NuttX/nuttx/configs/open1788/src/lpc17_sdraminitialize.d \
./NuttX/nuttx/configs/open1788/src/lpc17_ssp.d \
./NuttX/nuttx/configs/open1788/src/lpc17_touchscreen.d \
./NuttX/nuttx/configs/open1788/src/lpc17_userleds.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/configs/open1788/src/%.o: ../NuttX/nuttx/configs/open1788/src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


