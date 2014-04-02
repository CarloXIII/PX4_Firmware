################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/arch/arm/src/lm/lm_allocateheap.c \
../NuttX/nuttx/arch/arm/src/lm/lm_dumpgpio.c \
../NuttX/nuttx/arch/arm/src/lm/lm_ethernet.c \
../NuttX/nuttx/arch/arm/src/lm/lm_flash.c \
../NuttX/nuttx/arch/arm/src/lm/lm_gpio.c \
../NuttX/nuttx/arch/arm/src/lm/lm_gpioirq.c \
../NuttX/nuttx/arch/arm/src/lm/lm_irq.c \
../NuttX/nuttx/arch/arm/src/lm/lm_lowputc.c \
../NuttX/nuttx/arch/arm/src/lm/lm_mpuinit.c \
../NuttX/nuttx/arch/arm/src/lm/lm_serial.c \
../NuttX/nuttx/arch/arm/src/lm/lm_ssi.c \
../NuttX/nuttx/arch/arm/src/lm/lm_start.c \
../NuttX/nuttx/arch/arm/src/lm/lm_syscontrol.c \
../NuttX/nuttx/arch/arm/src/lm/lm_timerisr.c \
../NuttX/nuttx/arch/arm/src/lm/lm_userspace.c 

S_UPPER_SRCS += \
../NuttX/nuttx/arch/arm/src/lm/lm_vectors.S 

OBJS += \
./NuttX/nuttx/arch/arm/src/lm/lm_allocateheap.o \
./NuttX/nuttx/arch/arm/src/lm/lm_dumpgpio.o \
./NuttX/nuttx/arch/arm/src/lm/lm_ethernet.o \
./NuttX/nuttx/arch/arm/src/lm/lm_flash.o \
./NuttX/nuttx/arch/arm/src/lm/lm_gpio.o \
./NuttX/nuttx/arch/arm/src/lm/lm_gpioirq.o \
./NuttX/nuttx/arch/arm/src/lm/lm_irq.o \
./NuttX/nuttx/arch/arm/src/lm/lm_lowputc.o \
./NuttX/nuttx/arch/arm/src/lm/lm_mpuinit.o \
./NuttX/nuttx/arch/arm/src/lm/lm_serial.o \
./NuttX/nuttx/arch/arm/src/lm/lm_ssi.o \
./NuttX/nuttx/arch/arm/src/lm/lm_start.o \
./NuttX/nuttx/arch/arm/src/lm/lm_syscontrol.o \
./NuttX/nuttx/arch/arm/src/lm/lm_timerisr.o \
./NuttX/nuttx/arch/arm/src/lm/lm_userspace.o \
./NuttX/nuttx/arch/arm/src/lm/lm_vectors.o 

C_DEPS += \
./NuttX/nuttx/arch/arm/src/lm/lm_allocateheap.d \
./NuttX/nuttx/arch/arm/src/lm/lm_dumpgpio.d \
./NuttX/nuttx/arch/arm/src/lm/lm_ethernet.d \
./NuttX/nuttx/arch/arm/src/lm/lm_flash.d \
./NuttX/nuttx/arch/arm/src/lm/lm_gpio.d \
./NuttX/nuttx/arch/arm/src/lm/lm_gpioirq.d \
./NuttX/nuttx/arch/arm/src/lm/lm_irq.d \
./NuttX/nuttx/arch/arm/src/lm/lm_lowputc.d \
./NuttX/nuttx/arch/arm/src/lm/lm_mpuinit.d \
./NuttX/nuttx/arch/arm/src/lm/lm_serial.d \
./NuttX/nuttx/arch/arm/src/lm/lm_ssi.d \
./NuttX/nuttx/arch/arm/src/lm/lm_start.d \
./NuttX/nuttx/arch/arm/src/lm/lm_syscontrol.d \
./NuttX/nuttx/arch/arm/src/lm/lm_timerisr.d \
./NuttX/nuttx/arch/arm/src/lm/lm_userspace.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/arch/arm/src/lm/%.o: ../NuttX/nuttx/arch/arm/src/lm/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

NuttX/nuttx/arch/arm/src/lm/%.o: ../NuttX/nuttx/arch/arm/src/lm/%.S
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Assembler'
	as  -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


