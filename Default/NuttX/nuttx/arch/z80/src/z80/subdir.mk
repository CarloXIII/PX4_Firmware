################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/arch/z80/src/z80/z80_copystate.c \
../NuttX/nuttx/arch/z80/src/z80/z80_initialstate.c \
../NuttX/nuttx/arch/z80/src/z80/z80_io.c \
../NuttX/nuttx/arch/z80/src/z80/z80_irq.c \
../NuttX/nuttx/arch/z80/src/z80/z80_registerdump.c \
../NuttX/nuttx/arch/z80/src/z80/z80_schedulesigaction.c \
../NuttX/nuttx/arch/z80/src/z80/z80_sigdeliver.c 

ASM_SRCS += \
../NuttX/nuttx/arch/z80/src/z80/z80_head.asm \
../NuttX/nuttx/arch/z80/src/z80/z80_restoreusercontext.asm \
../NuttX/nuttx/arch/z80/src/z80/z80_rom.asm \
../NuttX/nuttx/arch/z80/src/z80/z80_saveusercontext.asm 

OBJS += \
./NuttX/nuttx/arch/z80/src/z80/z80_copystate.o \
./NuttX/nuttx/arch/z80/src/z80/z80_head.o \
./NuttX/nuttx/arch/z80/src/z80/z80_initialstate.o \
./NuttX/nuttx/arch/z80/src/z80/z80_io.o \
./NuttX/nuttx/arch/z80/src/z80/z80_irq.o \
./NuttX/nuttx/arch/z80/src/z80/z80_registerdump.o \
./NuttX/nuttx/arch/z80/src/z80/z80_restoreusercontext.o \
./NuttX/nuttx/arch/z80/src/z80/z80_rom.o \
./NuttX/nuttx/arch/z80/src/z80/z80_saveusercontext.o \
./NuttX/nuttx/arch/z80/src/z80/z80_schedulesigaction.o \
./NuttX/nuttx/arch/z80/src/z80/z80_sigdeliver.o 

C_DEPS += \
./NuttX/nuttx/arch/z80/src/z80/z80_copystate.d \
./NuttX/nuttx/arch/z80/src/z80/z80_initialstate.d \
./NuttX/nuttx/arch/z80/src/z80/z80_io.d \
./NuttX/nuttx/arch/z80/src/z80/z80_irq.d \
./NuttX/nuttx/arch/z80/src/z80/z80_registerdump.d \
./NuttX/nuttx/arch/z80/src/z80/z80_schedulesigaction.d \
./NuttX/nuttx/arch/z80/src/z80/z80_sigdeliver.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/arch/z80/src/z80/%.o: ../NuttX/nuttx/arch/z80/src/z80/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

NuttX/nuttx/arch/z80/src/z80/%.o: ../NuttX/nuttx/arch/z80/src/z80/%.asm
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Assembler'
	as  -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


