################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/arch/z80/src/z180/z180_copystate.c \
../NuttX/nuttx/arch/z80/src/z180/z180_initialstate.c \
../NuttX/nuttx/arch/z80/src/z180/z180_io.c \
../NuttX/nuttx/arch/z80/src/z180/z180_irq.c \
../NuttX/nuttx/arch/z80/src/z180/z180_lowscc.c \
../NuttX/nuttx/arch/z80/src/z180/z180_lowserial.c \
../NuttX/nuttx/arch/z80/src/z180/z180_lowuart.c \
../NuttX/nuttx/arch/z80/src/z180/z180_mmu.c \
../NuttX/nuttx/arch/z80/src/z180/z180_modifiyreg8.c \
../NuttX/nuttx/arch/z80/src/z180/z180_registerdump.c \
../NuttX/nuttx/arch/z80/src/z180/z180_scc.c \
../NuttX/nuttx/arch/z80/src/z180/z180_schedulesigaction.c \
../NuttX/nuttx/arch/z80/src/z180/z180_sigdeliver.c \
../NuttX/nuttx/arch/z80/src/z180/z180_timerisr.c 

ASM_SRCS += \
../NuttX/nuttx/arch/z80/src/z180/z180_head.asm \
../NuttX/nuttx/arch/z80/src/z180/z180_restoreusercontext.asm \
../NuttX/nuttx/arch/z80/src/z180/z180_rom.asm \
../NuttX/nuttx/arch/z80/src/z180/z180_romvectors.asm \
../NuttX/nuttx/arch/z80/src/z180/z180_saveusercontext.asm \
../NuttX/nuttx/arch/z80/src/z180/z180_vectcommon.asm \
../NuttX/nuttx/arch/z80/src/z180/z180_vectors.asm 

OBJS += \
./NuttX/nuttx/arch/z80/src/z180/z180_copystate.o \
./NuttX/nuttx/arch/z80/src/z180/z180_head.o \
./NuttX/nuttx/arch/z80/src/z180/z180_initialstate.o \
./NuttX/nuttx/arch/z80/src/z180/z180_io.o \
./NuttX/nuttx/arch/z80/src/z180/z180_irq.o \
./NuttX/nuttx/arch/z80/src/z180/z180_lowscc.o \
./NuttX/nuttx/arch/z80/src/z180/z180_lowserial.o \
./NuttX/nuttx/arch/z80/src/z180/z180_lowuart.o \
./NuttX/nuttx/arch/z80/src/z180/z180_mmu.o \
./NuttX/nuttx/arch/z80/src/z180/z180_modifiyreg8.o \
./NuttX/nuttx/arch/z80/src/z180/z180_registerdump.o \
./NuttX/nuttx/arch/z80/src/z180/z180_restoreusercontext.o \
./NuttX/nuttx/arch/z80/src/z180/z180_rom.o \
./NuttX/nuttx/arch/z80/src/z180/z180_romvectors.o \
./NuttX/nuttx/arch/z80/src/z180/z180_saveusercontext.o \
./NuttX/nuttx/arch/z80/src/z180/z180_scc.o \
./NuttX/nuttx/arch/z80/src/z180/z180_schedulesigaction.o \
./NuttX/nuttx/arch/z80/src/z180/z180_sigdeliver.o \
./NuttX/nuttx/arch/z80/src/z180/z180_timerisr.o \
./NuttX/nuttx/arch/z80/src/z180/z180_vectcommon.o \
./NuttX/nuttx/arch/z80/src/z180/z180_vectors.o 

C_DEPS += \
./NuttX/nuttx/arch/z80/src/z180/z180_copystate.d \
./NuttX/nuttx/arch/z80/src/z180/z180_initialstate.d \
./NuttX/nuttx/arch/z80/src/z180/z180_io.d \
./NuttX/nuttx/arch/z80/src/z180/z180_irq.d \
./NuttX/nuttx/arch/z80/src/z180/z180_lowscc.d \
./NuttX/nuttx/arch/z80/src/z180/z180_lowserial.d \
./NuttX/nuttx/arch/z80/src/z180/z180_lowuart.d \
./NuttX/nuttx/arch/z80/src/z180/z180_mmu.d \
./NuttX/nuttx/arch/z80/src/z180/z180_modifiyreg8.d \
./NuttX/nuttx/arch/z80/src/z180/z180_registerdump.d \
./NuttX/nuttx/arch/z80/src/z180/z180_scc.d \
./NuttX/nuttx/arch/z80/src/z180/z180_schedulesigaction.d \
./NuttX/nuttx/arch/z80/src/z180/z180_sigdeliver.d \
./NuttX/nuttx/arch/z80/src/z180/z180_timerisr.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/arch/z80/src/z180/%.o: ../NuttX/nuttx/arch/z80/src/z180/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

NuttX/nuttx/arch/z80/src/z180/%.o: ../NuttX/nuttx/arch/z80/src/z180/%.asm
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Assembler'
	as  -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


