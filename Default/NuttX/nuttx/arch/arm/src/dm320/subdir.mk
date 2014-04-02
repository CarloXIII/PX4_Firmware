################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/arch/arm/src/dm320/dm320_allocateheap.c \
../NuttX/nuttx/arch/arm/src/dm320/dm320_boot.c \
../NuttX/nuttx/arch/arm/src/dm320/dm320_decodeirq.c \
../NuttX/nuttx/arch/arm/src/dm320/dm320_framebuffer.c \
../NuttX/nuttx/arch/arm/src/dm320/dm320_irq.c \
../NuttX/nuttx/arch/arm/src/dm320/dm320_serial.c \
../NuttX/nuttx/arch/arm/src/dm320/dm320_timerisr.c \
../NuttX/nuttx/arch/arm/src/dm320/dm320_usbdev.c 

S_UPPER_SRCS += \
../NuttX/nuttx/arch/arm/src/dm320/dm320_lowputc.S \
../NuttX/nuttx/arch/arm/src/dm320/dm320_restart.S 

OBJS += \
./NuttX/nuttx/arch/arm/src/dm320/dm320_allocateheap.o \
./NuttX/nuttx/arch/arm/src/dm320/dm320_boot.o \
./NuttX/nuttx/arch/arm/src/dm320/dm320_decodeirq.o \
./NuttX/nuttx/arch/arm/src/dm320/dm320_framebuffer.o \
./NuttX/nuttx/arch/arm/src/dm320/dm320_irq.o \
./NuttX/nuttx/arch/arm/src/dm320/dm320_lowputc.o \
./NuttX/nuttx/arch/arm/src/dm320/dm320_restart.o \
./NuttX/nuttx/arch/arm/src/dm320/dm320_serial.o \
./NuttX/nuttx/arch/arm/src/dm320/dm320_timerisr.o \
./NuttX/nuttx/arch/arm/src/dm320/dm320_usbdev.o 

C_DEPS += \
./NuttX/nuttx/arch/arm/src/dm320/dm320_allocateheap.d \
./NuttX/nuttx/arch/arm/src/dm320/dm320_boot.d \
./NuttX/nuttx/arch/arm/src/dm320/dm320_decodeirq.d \
./NuttX/nuttx/arch/arm/src/dm320/dm320_framebuffer.d \
./NuttX/nuttx/arch/arm/src/dm320/dm320_irq.d \
./NuttX/nuttx/arch/arm/src/dm320/dm320_serial.d \
./NuttX/nuttx/arch/arm/src/dm320/dm320_timerisr.d \
./NuttX/nuttx/arch/arm/src/dm320/dm320_usbdev.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/arch/arm/src/dm320/%.o: ../NuttX/nuttx/arch/arm/src/dm320/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

NuttX/nuttx/arch/arm/src/dm320/%.o: ../NuttX/nuttx/arch/arm/src/dm320/%.S
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Assembler'
	as  -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


