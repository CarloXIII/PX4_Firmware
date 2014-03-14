################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/arch/arm/src/lpc214x/lpc214x_decodeirq.c \
../NuttX/nuttx/arch/arm/src/lpc214x/lpc214x_irq.c \
../NuttX/nuttx/arch/arm/src/lpc214x/lpc214x_serial.c \
../NuttX/nuttx/arch/arm/src/lpc214x/lpc214x_timerisr.c \
../NuttX/nuttx/arch/arm/src/lpc214x/lpc214x_usbdev.c 

S_UPPER_SRCS += \
../NuttX/nuttx/arch/arm/src/lpc214x/lpc214x_head.S \
../NuttX/nuttx/arch/arm/src/lpc214x/lpc214x_lowputc.S 

OBJS += \
./NuttX/nuttx/arch/arm/src/lpc214x/lpc214x_decodeirq.o \
./NuttX/nuttx/arch/arm/src/lpc214x/lpc214x_head.o \
./NuttX/nuttx/arch/arm/src/lpc214x/lpc214x_irq.o \
./NuttX/nuttx/arch/arm/src/lpc214x/lpc214x_lowputc.o \
./NuttX/nuttx/arch/arm/src/lpc214x/lpc214x_serial.o \
./NuttX/nuttx/arch/arm/src/lpc214x/lpc214x_timerisr.o \
./NuttX/nuttx/arch/arm/src/lpc214x/lpc214x_usbdev.o 

C_DEPS += \
./NuttX/nuttx/arch/arm/src/lpc214x/lpc214x_decodeirq.d \
./NuttX/nuttx/arch/arm/src/lpc214x/lpc214x_irq.d \
./NuttX/nuttx/arch/arm/src/lpc214x/lpc214x_serial.d \
./NuttX/nuttx/arch/arm/src/lpc214x/lpc214x_timerisr.d \
./NuttX/nuttx/arch/arm/src/lpc214x/lpc214x_usbdev.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/arch/arm/src/lpc214x/%.o: ../NuttX/nuttx/arch/arm/src/lpc214x/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

NuttX/nuttx/arch/arm/src/lpc214x/%.o: ../NuttX/nuttx/arch/arm/src/lpc214x/%.S
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Assembler'
	as  -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


