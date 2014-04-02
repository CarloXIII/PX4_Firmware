################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/arch/sh/src/m16c/m16c_copystate.c \
../NuttX/nuttx/arch/sh/src/m16c/m16c_dumpstate.c \
../NuttX/nuttx/arch/sh/src/m16c/m16c_initialstate.c \
../NuttX/nuttx/arch/sh/src/m16c/m16c_irq.c \
../NuttX/nuttx/arch/sh/src/m16c/m16c_lowputc.c \
../NuttX/nuttx/arch/sh/src/m16c/m16c_schedulesigaction.c \
../NuttX/nuttx/arch/sh/src/m16c/m16c_serial.c \
../NuttX/nuttx/arch/sh/src/m16c/m16c_sigdeliver.c \
../NuttX/nuttx/arch/sh/src/m16c/m16c_timerisr.c 

S_UPPER_SRCS += \
../NuttX/nuttx/arch/sh/src/m16c/m16c_head.S \
../NuttX/nuttx/arch/sh/src/m16c/m16c_vectors.S 

OBJS += \
./NuttX/nuttx/arch/sh/src/m16c/m16c_copystate.o \
./NuttX/nuttx/arch/sh/src/m16c/m16c_dumpstate.o \
./NuttX/nuttx/arch/sh/src/m16c/m16c_head.o \
./NuttX/nuttx/arch/sh/src/m16c/m16c_initialstate.o \
./NuttX/nuttx/arch/sh/src/m16c/m16c_irq.o \
./NuttX/nuttx/arch/sh/src/m16c/m16c_lowputc.o \
./NuttX/nuttx/arch/sh/src/m16c/m16c_schedulesigaction.o \
./NuttX/nuttx/arch/sh/src/m16c/m16c_serial.o \
./NuttX/nuttx/arch/sh/src/m16c/m16c_sigdeliver.o \
./NuttX/nuttx/arch/sh/src/m16c/m16c_timerisr.o \
./NuttX/nuttx/arch/sh/src/m16c/m16c_vectors.o 

C_DEPS += \
./NuttX/nuttx/arch/sh/src/m16c/m16c_copystate.d \
./NuttX/nuttx/arch/sh/src/m16c/m16c_dumpstate.d \
./NuttX/nuttx/arch/sh/src/m16c/m16c_initialstate.d \
./NuttX/nuttx/arch/sh/src/m16c/m16c_irq.d \
./NuttX/nuttx/arch/sh/src/m16c/m16c_lowputc.d \
./NuttX/nuttx/arch/sh/src/m16c/m16c_schedulesigaction.d \
./NuttX/nuttx/arch/sh/src/m16c/m16c_serial.d \
./NuttX/nuttx/arch/sh/src/m16c/m16c_sigdeliver.d \
./NuttX/nuttx/arch/sh/src/m16c/m16c_timerisr.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/arch/sh/src/m16c/%.o: ../NuttX/nuttx/arch/sh/src/m16c/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

NuttX/nuttx/arch/sh/src/m16c/%.o: ../NuttX/nuttx/arch/sh/src/m16c/%.S
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Assembler'
	as  -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


