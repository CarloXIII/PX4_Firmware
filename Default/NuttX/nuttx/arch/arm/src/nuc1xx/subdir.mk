################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/arch/arm/src/nuc1xx/nuc_clockconfig.c \
../NuttX/nuttx/arch/arm/src/nuc1xx/nuc_dumpgpio.c \
../NuttX/nuttx/arch/arm/src/nuc1xx/nuc_gpio.c \
../NuttX/nuttx/arch/arm/src/nuc1xx/nuc_idle.c \
../NuttX/nuttx/arch/arm/src/nuc1xx/nuc_irq.c \
../NuttX/nuttx/arch/arm/src/nuc1xx/nuc_lowputc.c \
../NuttX/nuttx/arch/arm/src/nuc1xx/nuc_serial.c \
../NuttX/nuttx/arch/arm/src/nuc1xx/nuc_start.c \
../NuttX/nuttx/arch/arm/src/nuc1xx/nuc_timerisr.c \
../NuttX/nuttx/arch/arm/src/nuc1xx/nuc_userspace.c 

OBJS += \
./NuttX/nuttx/arch/arm/src/nuc1xx/nuc_clockconfig.o \
./NuttX/nuttx/arch/arm/src/nuc1xx/nuc_dumpgpio.o \
./NuttX/nuttx/arch/arm/src/nuc1xx/nuc_gpio.o \
./NuttX/nuttx/arch/arm/src/nuc1xx/nuc_idle.o \
./NuttX/nuttx/arch/arm/src/nuc1xx/nuc_irq.o \
./NuttX/nuttx/arch/arm/src/nuc1xx/nuc_lowputc.o \
./NuttX/nuttx/arch/arm/src/nuc1xx/nuc_serial.o \
./NuttX/nuttx/arch/arm/src/nuc1xx/nuc_start.o \
./NuttX/nuttx/arch/arm/src/nuc1xx/nuc_timerisr.o \
./NuttX/nuttx/arch/arm/src/nuc1xx/nuc_userspace.o 

C_DEPS += \
./NuttX/nuttx/arch/arm/src/nuc1xx/nuc_clockconfig.d \
./NuttX/nuttx/arch/arm/src/nuc1xx/nuc_dumpgpio.d \
./NuttX/nuttx/arch/arm/src/nuc1xx/nuc_gpio.d \
./NuttX/nuttx/arch/arm/src/nuc1xx/nuc_idle.d \
./NuttX/nuttx/arch/arm/src/nuc1xx/nuc_irq.d \
./NuttX/nuttx/arch/arm/src/nuc1xx/nuc_lowputc.d \
./NuttX/nuttx/arch/arm/src/nuc1xx/nuc_serial.d \
./NuttX/nuttx/arch/arm/src/nuc1xx/nuc_start.d \
./NuttX/nuttx/arch/arm/src/nuc1xx/nuc_timerisr.d \
./NuttX/nuttx/arch/arm/src/nuc1xx/nuc_userspace.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/arch/arm/src/nuc1xx/%.o: ../NuttX/nuttx/arch/arm/src/nuc1xx/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


