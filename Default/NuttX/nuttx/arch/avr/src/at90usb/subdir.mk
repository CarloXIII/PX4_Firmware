################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/arch/avr/src/at90usb/at90usb_lowconsole.c \
../NuttX/nuttx/arch/avr/src/at90usb/at90usb_lowinit.c \
../NuttX/nuttx/arch/avr/src/at90usb/at90usb_serial.c \
../NuttX/nuttx/arch/avr/src/at90usb/at90usb_timerisr.c \
../NuttX/nuttx/arch/avr/src/at90usb/at90usb_usbdev.c 

S_UPPER_SRCS += \
../NuttX/nuttx/arch/avr/src/at90usb/at90usb_exceptions.S \
../NuttX/nuttx/arch/avr/src/at90usb/at90usb_head.S 

OBJS += \
./NuttX/nuttx/arch/avr/src/at90usb/at90usb_exceptions.o \
./NuttX/nuttx/arch/avr/src/at90usb/at90usb_head.o \
./NuttX/nuttx/arch/avr/src/at90usb/at90usb_lowconsole.o \
./NuttX/nuttx/arch/avr/src/at90usb/at90usb_lowinit.o \
./NuttX/nuttx/arch/avr/src/at90usb/at90usb_serial.o \
./NuttX/nuttx/arch/avr/src/at90usb/at90usb_timerisr.o \
./NuttX/nuttx/arch/avr/src/at90usb/at90usb_usbdev.o 

C_DEPS += \
./NuttX/nuttx/arch/avr/src/at90usb/at90usb_lowconsole.d \
./NuttX/nuttx/arch/avr/src/at90usb/at90usb_lowinit.d \
./NuttX/nuttx/arch/avr/src/at90usb/at90usb_serial.d \
./NuttX/nuttx/arch/avr/src/at90usb/at90usb_timerisr.d \
./NuttX/nuttx/arch/avr/src/at90usb/at90usb_usbdev.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/arch/avr/src/at90usb/%.o: ../NuttX/nuttx/arch/avr/src/at90usb/%.S
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Assembler'
	as  -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

NuttX/nuttx/arch/avr/src/at90usb/%.o: ../NuttX/nuttx/arch/avr/src/at90usb/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


