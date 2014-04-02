################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/arch/z80/src/ez80/ez80_clock.c \
../NuttX/nuttx/arch/z80/src/ez80/ez80_copystate.c \
../NuttX/nuttx/arch/z80/src/ez80/ez80_emac.c \
../NuttX/nuttx/arch/z80/src/ez80/ez80_i2c.c \
../NuttX/nuttx/arch/z80/src/ez80/ez80_initialstate.c \
../NuttX/nuttx/arch/z80/src/ez80/ez80_irq.c \
../NuttX/nuttx/arch/z80/src/ez80/ez80_lowuart.c \
../NuttX/nuttx/arch/z80/src/ez80/ez80_registerdump.c \
../NuttX/nuttx/arch/z80/src/ez80/ez80_schedulesigaction.c \
../NuttX/nuttx/arch/z80/src/ez80/ez80_serial.c \
../NuttX/nuttx/arch/z80/src/ez80/ez80_sigdeliver.c \
../NuttX/nuttx/arch/z80/src/ez80/ez80_spi.c \
../NuttX/nuttx/arch/z80/src/ez80/ez80_timerisr.c 

ASM_SRCS += \
../NuttX/nuttx/arch/z80/src/ez80/ez80_io.asm \
../NuttX/nuttx/arch/z80/src/ez80/ez80_irqsave.asm \
../NuttX/nuttx/arch/z80/src/ez80/ez80_restorecontext.asm \
../NuttX/nuttx/arch/z80/src/ez80/ez80_saveusercontext.asm \
../NuttX/nuttx/arch/z80/src/ez80/ez80_startup.asm \
../NuttX/nuttx/arch/z80/src/ez80/ez80_vectors.asm \
../NuttX/nuttx/arch/z80/src/ez80/ez80f91_init.asm 

OBJS += \
./NuttX/nuttx/arch/z80/src/ez80/ez80_clock.o \
./NuttX/nuttx/arch/z80/src/ez80/ez80_copystate.o \
./NuttX/nuttx/arch/z80/src/ez80/ez80_emac.o \
./NuttX/nuttx/arch/z80/src/ez80/ez80_i2c.o \
./NuttX/nuttx/arch/z80/src/ez80/ez80_initialstate.o \
./NuttX/nuttx/arch/z80/src/ez80/ez80_io.o \
./NuttX/nuttx/arch/z80/src/ez80/ez80_irq.o \
./NuttX/nuttx/arch/z80/src/ez80/ez80_irqsave.o \
./NuttX/nuttx/arch/z80/src/ez80/ez80_lowuart.o \
./NuttX/nuttx/arch/z80/src/ez80/ez80_registerdump.o \
./NuttX/nuttx/arch/z80/src/ez80/ez80_restorecontext.o \
./NuttX/nuttx/arch/z80/src/ez80/ez80_saveusercontext.o \
./NuttX/nuttx/arch/z80/src/ez80/ez80_schedulesigaction.o \
./NuttX/nuttx/arch/z80/src/ez80/ez80_serial.o \
./NuttX/nuttx/arch/z80/src/ez80/ez80_sigdeliver.o \
./NuttX/nuttx/arch/z80/src/ez80/ez80_spi.o \
./NuttX/nuttx/arch/z80/src/ez80/ez80_startup.o \
./NuttX/nuttx/arch/z80/src/ez80/ez80_timerisr.o \
./NuttX/nuttx/arch/z80/src/ez80/ez80_vectors.o \
./NuttX/nuttx/arch/z80/src/ez80/ez80f91_init.o 

C_DEPS += \
./NuttX/nuttx/arch/z80/src/ez80/ez80_clock.d \
./NuttX/nuttx/arch/z80/src/ez80/ez80_copystate.d \
./NuttX/nuttx/arch/z80/src/ez80/ez80_emac.d \
./NuttX/nuttx/arch/z80/src/ez80/ez80_i2c.d \
./NuttX/nuttx/arch/z80/src/ez80/ez80_initialstate.d \
./NuttX/nuttx/arch/z80/src/ez80/ez80_irq.d \
./NuttX/nuttx/arch/z80/src/ez80/ez80_lowuart.d \
./NuttX/nuttx/arch/z80/src/ez80/ez80_registerdump.d \
./NuttX/nuttx/arch/z80/src/ez80/ez80_schedulesigaction.d \
./NuttX/nuttx/arch/z80/src/ez80/ez80_serial.d \
./NuttX/nuttx/arch/z80/src/ez80/ez80_sigdeliver.d \
./NuttX/nuttx/arch/z80/src/ez80/ez80_spi.d \
./NuttX/nuttx/arch/z80/src/ez80/ez80_timerisr.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/arch/z80/src/ez80/%.o: ../NuttX/nuttx/arch/z80/src/ez80/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

NuttX/nuttx/arch/z80/src/ez80/%.o: ../NuttX/nuttx/arch/z80/src/ez80/%.asm
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Assembler'
	as  -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


