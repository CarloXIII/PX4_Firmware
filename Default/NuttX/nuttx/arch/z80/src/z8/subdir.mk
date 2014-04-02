################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/arch/z80/src/z8/z8_i2c.c \
../NuttX/nuttx/arch/z80/src/z8/z8_initialstate.c \
../NuttX/nuttx/arch/z80/src/z8/z8_irq.c \
../NuttX/nuttx/arch/z80/src/z8/z8_lowuart.c \
../NuttX/nuttx/arch/z80/src/z8/z8_registerdump.c \
../NuttX/nuttx/arch/z80/src/z8/z8_saveirqcontext.c \
../NuttX/nuttx/arch/z80/src/z8/z8_schedulesigaction.c \
../NuttX/nuttx/arch/z80/src/z8/z8_serial.c \
../NuttX/nuttx/arch/z80/src/z8/z8_sigdeliver.c \
../NuttX/nuttx/arch/z80/src/z8/z8_timerisr.c 

S_UPPER_SRCS += \
../NuttX/nuttx/arch/z80/src/z8/z8_head.S \
../NuttX/nuttx/arch/z80/src/z8/z8_restorecontext.S \
../NuttX/nuttx/arch/z80/src/z8/z8_saveusercontext.S \
../NuttX/nuttx/arch/z80/src/z8/z8_vector.S 

OBJS += \
./NuttX/nuttx/arch/z80/src/z8/z8_head.o \
./NuttX/nuttx/arch/z80/src/z8/z8_i2c.o \
./NuttX/nuttx/arch/z80/src/z8/z8_initialstate.o \
./NuttX/nuttx/arch/z80/src/z8/z8_irq.o \
./NuttX/nuttx/arch/z80/src/z8/z8_lowuart.o \
./NuttX/nuttx/arch/z80/src/z8/z8_registerdump.o \
./NuttX/nuttx/arch/z80/src/z8/z8_restorecontext.o \
./NuttX/nuttx/arch/z80/src/z8/z8_saveirqcontext.o \
./NuttX/nuttx/arch/z80/src/z8/z8_saveusercontext.o \
./NuttX/nuttx/arch/z80/src/z8/z8_schedulesigaction.o \
./NuttX/nuttx/arch/z80/src/z8/z8_serial.o \
./NuttX/nuttx/arch/z80/src/z8/z8_sigdeliver.o \
./NuttX/nuttx/arch/z80/src/z8/z8_timerisr.o \
./NuttX/nuttx/arch/z80/src/z8/z8_vector.o 

C_DEPS += \
./NuttX/nuttx/arch/z80/src/z8/z8_i2c.d \
./NuttX/nuttx/arch/z80/src/z8/z8_initialstate.d \
./NuttX/nuttx/arch/z80/src/z8/z8_irq.d \
./NuttX/nuttx/arch/z80/src/z8/z8_lowuart.d \
./NuttX/nuttx/arch/z80/src/z8/z8_registerdump.d \
./NuttX/nuttx/arch/z80/src/z8/z8_saveirqcontext.d \
./NuttX/nuttx/arch/z80/src/z8/z8_schedulesigaction.d \
./NuttX/nuttx/arch/z80/src/z8/z8_serial.d \
./NuttX/nuttx/arch/z80/src/z8/z8_sigdeliver.d \
./NuttX/nuttx/arch/z80/src/z8/z8_timerisr.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/arch/z80/src/z8/%.o: ../NuttX/nuttx/arch/z80/src/z8/%.S
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Assembler'
	as  -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

NuttX/nuttx/arch/z80/src/z8/%.o: ../NuttX/nuttx/arch/z80/src/z8/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


