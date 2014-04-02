################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/arch/sh/src/sh1/sh1_copystate.c \
../NuttX/nuttx/arch/sh/src/sh1/sh1_dumpstate.c \
../NuttX/nuttx/arch/sh/src/sh1/sh1_initialstate.c \
../NuttX/nuttx/arch/sh/src/sh1/sh1_irq.c \
../NuttX/nuttx/arch/sh/src/sh1/sh1_lowputc.c \
../NuttX/nuttx/arch/sh/src/sh1/sh1_schedulesigaction.c \
../NuttX/nuttx/arch/sh/src/sh1/sh1_serial.c \
../NuttX/nuttx/arch/sh/src/sh1/sh1_sigdeliver.c \
../NuttX/nuttx/arch/sh/src/sh1/sh1_timerisr.c 

S_UPPER_SRCS += \
../NuttX/nuttx/arch/sh/src/sh1/sh1_head.S \
../NuttX/nuttx/arch/sh/src/sh1/sh1_saveusercontext.S \
../NuttX/nuttx/arch/sh/src/sh1/sh1_vector.S 

OBJS += \
./NuttX/nuttx/arch/sh/src/sh1/sh1_copystate.o \
./NuttX/nuttx/arch/sh/src/sh1/sh1_dumpstate.o \
./NuttX/nuttx/arch/sh/src/sh1/sh1_head.o \
./NuttX/nuttx/arch/sh/src/sh1/sh1_initialstate.o \
./NuttX/nuttx/arch/sh/src/sh1/sh1_irq.o \
./NuttX/nuttx/arch/sh/src/sh1/sh1_lowputc.o \
./NuttX/nuttx/arch/sh/src/sh1/sh1_saveusercontext.o \
./NuttX/nuttx/arch/sh/src/sh1/sh1_schedulesigaction.o \
./NuttX/nuttx/arch/sh/src/sh1/sh1_serial.o \
./NuttX/nuttx/arch/sh/src/sh1/sh1_sigdeliver.o \
./NuttX/nuttx/arch/sh/src/sh1/sh1_timerisr.o \
./NuttX/nuttx/arch/sh/src/sh1/sh1_vector.o 

C_DEPS += \
./NuttX/nuttx/arch/sh/src/sh1/sh1_copystate.d \
./NuttX/nuttx/arch/sh/src/sh1/sh1_dumpstate.d \
./NuttX/nuttx/arch/sh/src/sh1/sh1_initialstate.d \
./NuttX/nuttx/arch/sh/src/sh1/sh1_irq.d \
./NuttX/nuttx/arch/sh/src/sh1/sh1_lowputc.d \
./NuttX/nuttx/arch/sh/src/sh1/sh1_schedulesigaction.d \
./NuttX/nuttx/arch/sh/src/sh1/sh1_serial.d \
./NuttX/nuttx/arch/sh/src/sh1/sh1_sigdeliver.d \
./NuttX/nuttx/arch/sh/src/sh1/sh1_timerisr.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/arch/sh/src/sh1/%.o: ../NuttX/nuttx/arch/sh/src/sh1/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

NuttX/nuttx/arch/sh/src/sh1/%.o: ../NuttX/nuttx/arch/sh/src/sh1/%.S
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Assembler'
	as  -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


