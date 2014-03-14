################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/arch/arm/src/lpc2378/lpc23xx_decodeirq.c \
../NuttX/nuttx/arch/arm/src/lpc2378/lpc23xx_io.c \
../NuttX/nuttx/arch/arm/src/lpc2378/lpc23xx_irq.c \
../NuttX/nuttx/arch/arm/src/lpc2378/lpc23xx_pllsetup.c \
../NuttX/nuttx/arch/arm/src/lpc2378/lpc23xx_serial.c \
../NuttX/nuttx/arch/arm/src/lpc2378/lpc23xx_timerisr.c 

S_UPPER_SRCS += \
../NuttX/nuttx/arch/arm/src/lpc2378/lpc23xx_head.S \
../NuttX/nuttx/arch/arm/src/lpc2378/lpc23xx_lowputc.S 

OBJS += \
./NuttX/nuttx/arch/arm/src/lpc2378/lpc23xx_decodeirq.o \
./NuttX/nuttx/arch/arm/src/lpc2378/lpc23xx_head.o \
./NuttX/nuttx/arch/arm/src/lpc2378/lpc23xx_io.o \
./NuttX/nuttx/arch/arm/src/lpc2378/lpc23xx_irq.o \
./NuttX/nuttx/arch/arm/src/lpc2378/lpc23xx_lowputc.o \
./NuttX/nuttx/arch/arm/src/lpc2378/lpc23xx_pllsetup.o \
./NuttX/nuttx/arch/arm/src/lpc2378/lpc23xx_serial.o \
./NuttX/nuttx/arch/arm/src/lpc2378/lpc23xx_timerisr.o 

C_DEPS += \
./NuttX/nuttx/arch/arm/src/lpc2378/lpc23xx_decodeirq.d \
./NuttX/nuttx/arch/arm/src/lpc2378/lpc23xx_io.d \
./NuttX/nuttx/arch/arm/src/lpc2378/lpc23xx_irq.d \
./NuttX/nuttx/arch/arm/src/lpc2378/lpc23xx_pllsetup.d \
./NuttX/nuttx/arch/arm/src/lpc2378/lpc23xx_serial.d \
./NuttX/nuttx/arch/arm/src/lpc2378/lpc23xx_timerisr.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/arch/arm/src/lpc2378/%.o: ../NuttX/nuttx/arch/arm/src/lpc2378/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

NuttX/nuttx/arch/arm/src/lpc2378/%.o: ../NuttX/nuttx/arch/arm/src/lpc2378/%.S
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Assembler'
	as  -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


