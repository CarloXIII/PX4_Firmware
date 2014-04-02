################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/arch/arm/src/c5471/c5471_ethernet.c \
../NuttX/nuttx/arch/arm/src/c5471/c5471_irq.c \
../NuttX/nuttx/arch/arm/src/c5471/c5471_serial.c \
../NuttX/nuttx/arch/arm/src/c5471/c5471_timerisr.c \
../NuttX/nuttx/arch/arm/src/c5471/c5471_watchdog.c 

S_UPPER_SRCS += \
../NuttX/nuttx/arch/arm/src/c5471/c5471_lowputc.S \
../NuttX/nuttx/arch/arm/src/c5471/c5471_vectors.S 

OBJS += \
./NuttX/nuttx/arch/arm/src/c5471/c5471_ethernet.o \
./NuttX/nuttx/arch/arm/src/c5471/c5471_irq.o \
./NuttX/nuttx/arch/arm/src/c5471/c5471_lowputc.o \
./NuttX/nuttx/arch/arm/src/c5471/c5471_serial.o \
./NuttX/nuttx/arch/arm/src/c5471/c5471_timerisr.o \
./NuttX/nuttx/arch/arm/src/c5471/c5471_vectors.o \
./NuttX/nuttx/arch/arm/src/c5471/c5471_watchdog.o 

C_DEPS += \
./NuttX/nuttx/arch/arm/src/c5471/c5471_ethernet.d \
./NuttX/nuttx/arch/arm/src/c5471/c5471_irq.d \
./NuttX/nuttx/arch/arm/src/c5471/c5471_serial.d \
./NuttX/nuttx/arch/arm/src/c5471/c5471_timerisr.d \
./NuttX/nuttx/arch/arm/src/c5471/c5471_watchdog.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/arch/arm/src/c5471/%.o: ../NuttX/nuttx/arch/arm/src/c5471/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

NuttX/nuttx/arch/arm/src/c5471/%.o: ../NuttX/nuttx/arch/arm/src/c5471/%.S
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Assembler'
	as  -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


