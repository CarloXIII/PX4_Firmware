################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/configs/lpc4330-xplorer/src/up_autoleds.c \
../NuttX/nuttx/configs/lpc4330-xplorer/src/up_boot.c \
../NuttX/nuttx/configs/lpc4330-xplorer/src/up_buttons.c \
../NuttX/nuttx/configs/lpc4330-xplorer/src/up_nsh.c \
../NuttX/nuttx/configs/lpc4330-xplorer/src/up_ostest.c \
../NuttX/nuttx/configs/lpc4330-xplorer/src/up_userleds.c 

OBJS += \
./NuttX/nuttx/configs/lpc4330-xplorer/src/up_autoleds.o \
./NuttX/nuttx/configs/lpc4330-xplorer/src/up_boot.o \
./NuttX/nuttx/configs/lpc4330-xplorer/src/up_buttons.o \
./NuttX/nuttx/configs/lpc4330-xplorer/src/up_nsh.o \
./NuttX/nuttx/configs/lpc4330-xplorer/src/up_ostest.o \
./NuttX/nuttx/configs/lpc4330-xplorer/src/up_userleds.o 

C_DEPS += \
./NuttX/nuttx/configs/lpc4330-xplorer/src/up_autoleds.d \
./NuttX/nuttx/configs/lpc4330-xplorer/src/up_boot.d \
./NuttX/nuttx/configs/lpc4330-xplorer/src/up_buttons.d \
./NuttX/nuttx/configs/lpc4330-xplorer/src/up_nsh.d \
./NuttX/nuttx/configs/lpc4330-xplorer/src/up_ostest.d \
./NuttX/nuttx/configs/lpc4330-xplorer/src/up_userleds.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/configs/lpc4330-xplorer/src/%.o: ../NuttX/nuttx/configs/lpc4330-xplorer/src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


