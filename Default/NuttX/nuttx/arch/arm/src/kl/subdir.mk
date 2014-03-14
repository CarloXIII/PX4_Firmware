################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/arch/arm/src/kl/kl_cfmconfig.c \
../NuttX/nuttx/arch/arm/src/kl/kl_clockconfig.c \
../NuttX/nuttx/arch/arm/src/kl/kl_dumpgpio.c \
../NuttX/nuttx/arch/arm/src/kl/kl_gpio.c \
../NuttX/nuttx/arch/arm/src/kl/kl_idle.c \
../NuttX/nuttx/arch/arm/src/kl/kl_irq.c \
../NuttX/nuttx/arch/arm/src/kl/kl_irqprio.c \
../NuttX/nuttx/arch/arm/src/kl/kl_lowputc.c \
../NuttX/nuttx/arch/arm/src/kl/kl_serial.c \
../NuttX/nuttx/arch/arm/src/kl/kl_start.c \
../NuttX/nuttx/arch/arm/src/kl/kl_timerisr.c \
../NuttX/nuttx/arch/arm/src/kl/kl_userspace.c 

OBJS += \
./NuttX/nuttx/arch/arm/src/kl/kl_cfmconfig.o \
./NuttX/nuttx/arch/arm/src/kl/kl_clockconfig.o \
./NuttX/nuttx/arch/arm/src/kl/kl_dumpgpio.o \
./NuttX/nuttx/arch/arm/src/kl/kl_gpio.o \
./NuttX/nuttx/arch/arm/src/kl/kl_idle.o \
./NuttX/nuttx/arch/arm/src/kl/kl_irq.o \
./NuttX/nuttx/arch/arm/src/kl/kl_irqprio.o \
./NuttX/nuttx/arch/arm/src/kl/kl_lowputc.o \
./NuttX/nuttx/arch/arm/src/kl/kl_serial.o \
./NuttX/nuttx/arch/arm/src/kl/kl_start.o \
./NuttX/nuttx/arch/arm/src/kl/kl_timerisr.o \
./NuttX/nuttx/arch/arm/src/kl/kl_userspace.o 

C_DEPS += \
./NuttX/nuttx/arch/arm/src/kl/kl_cfmconfig.d \
./NuttX/nuttx/arch/arm/src/kl/kl_clockconfig.d \
./NuttX/nuttx/arch/arm/src/kl/kl_dumpgpio.d \
./NuttX/nuttx/arch/arm/src/kl/kl_gpio.d \
./NuttX/nuttx/arch/arm/src/kl/kl_idle.d \
./NuttX/nuttx/arch/arm/src/kl/kl_irq.d \
./NuttX/nuttx/arch/arm/src/kl/kl_irqprio.d \
./NuttX/nuttx/arch/arm/src/kl/kl_lowputc.d \
./NuttX/nuttx/arch/arm/src/kl/kl_serial.d \
./NuttX/nuttx/arch/arm/src/kl/kl_start.d \
./NuttX/nuttx/arch/arm/src/kl/kl_timerisr.d \
./NuttX/nuttx/arch/arm/src/kl/kl_userspace.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/arch/arm/src/kl/%.o: ../NuttX/nuttx/arch/arm/src/kl/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


