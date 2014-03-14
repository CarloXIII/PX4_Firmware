################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/arch/mips/src/pic32mx/pic32mx-decodeirq.c \
../NuttX/nuttx/arch/mips/src/pic32mx/pic32mx-ethernet.c \
../NuttX/nuttx/arch/mips/src/pic32mx/pic32mx-exception.c \
../NuttX/nuttx/arch/mips/src/pic32mx/pic32mx-gpio.c \
../NuttX/nuttx/arch/mips/src/pic32mx/pic32mx-gpioirq.c \
../NuttX/nuttx/arch/mips/src/pic32mx/pic32mx-irq.c \
../NuttX/nuttx/arch/mips/src/pic32mx/pic32mx-lowconsole.c \
../NuttX/nuttx/arch/mips/src/pic32mx/pic32mx-lowinit.c \
../NuttX/nuttx/arch/mips/src/pic32mx/pic32mx-serial.c \
../NuttX/nuttx/arch/mips/src/pic32mx/pic32mx-spi.c \
../NuttX/nuttx/arch/mips/src/pic32mx/pic32mx-timerisr.c \
../NuttX/nuttx/arch/mips/src/pic32mx/pic32mx-usbdev.c 

S_UPPER_SRCS += \
../NuttX/nuttx/arch/mips/src/pic32mx/pic32mx-head.S 

OBJS += \
./NuttX/nuttx/arch/mips/src/pic32mx/pic32mx-decodeirq.o \
./NuttX/nuttx/arch/mips/src/pic32mx/pic32mx-ethernet.o \
./NuttX/nuttx/arch/mips/src/pic32mx/pic32mx-exception.o \
./NuttX/nuttx/arch/mips/src/pic32mx/pic32mx-gpio.o \
./NuttX/nuttx/arch/mips/src/pic32mx/pic32mx-gpioirq.o \
./NuttX/nuttx/arch/mips/src/pic32mx/pic32mx-head.o \
./NuttX/nuttx/arch/mips/src/pic32mx/pic32mx-irq.o \
./NuttX/nuttx/arch/mips/src/pic32mx/pic32mx-lowconsole.o \
./NuttX/nuttx/arch/mips/src/pic32mx/pic32mx-lowinit.o \
./NuttX/nuttx/arch/mips/src/pic32mx/pic32mx-serial.o \
./NuttX/nuttx/arch/mips/src/pic32mx/pic32mx-spi.o \
./NuttX/nuttx/arch/mips/src/pic32mx/pic32mx-timerisr.o \
./NuttX/nuttx/arch/mips/src/pic32mx/pic32mx-usbdev.o 

C_DEPS += \
./NuttX/nuttx/arch/mips/src/pic32mx/pic32mx-decodeirq.d \
./NuttX/nuttx/arch/mips/src/pic32mx/pic32mx-ethernet.d \
./NuttX/nuttx/arch/mips/src/pic32mx/pic32mx-exception.d \
./NuttX/nuttx/arch/mips/src/pic32mx/pic32mx-gpio.d \
./NuttX/nuttx/arch/mips/src/pic32mx/pic32mx-gpioirq.d \
./NuttX/nuttx/arch/mips/src/pic32mx/pic32mx-irq.d \
./NuttX/nuttx/arch/mips/src/pic32mx/pic32mx-lowconsole.d \
./NuttX/nuttx/arch/mips/src/pic32mx/pic32mx-lowinit.d \
./NuttX/nuttx/arch/mips/src/pic32mx/pic32mx-serial.d \
./NuttX/nuttx/arch/mips/src/pic32mx/pic32mx-spi.d \
./NuttX/nuttx/arch/mips/src/pic32mx/pic32mx-timerisr.d \
./NuttX/nuttx/arch/mips/src/pic32mx/pic32mx-usbdev.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/arch/mips/src/pic32mx/%.o: ../NuttX/nuttx/arch/mips/src/pic32mx/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

NuttX/nuttx/arch/mips/src/pic32mx/%.o: ../NuttX/nuttx/arch/mips/src/pic32mx/%.S
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Assembler'
	as  -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


