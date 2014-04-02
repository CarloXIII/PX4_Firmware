################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/arch/8051/src/up_allocateheap.c \
../NuttX/nuttx/arch/8051/src/up_assert.c \
../NuttX/nuttx/arch/8051/src/up_blocktask.c \
../NuttX/nuttx/arch/8051/src/up_debug.c \
../NuttX/nuttx/arch/8051/src/up_delay.c \
../NuttX/nuttx/arch/8051/src/up_exit.c \
../NuttX/nuttx/arch/8051/src/up_idle.c \
../NuttX/nuttx/arch/8051/src/up_initialize.c \
../NuttX/nuttx/arch/8051/src/up_initialstate.c \
../NuttX/nuttx/arch/8051/src/up_interruptcontext.c \
../NuttX/nuttx/arch/8051/src/up_irq.c \
../NuttX/nuttx/arch/8051/src/up_irqtest.c \
../NuttX/nuttx/arch/8051/src/up_putc.c \
../NuttX/nuttx/arch/8051/src/up_releasepending.c \
../NuttX/nuttx/arch/8051/src/up_reprioritizertr.c \
../NuttX/nuttx/arch/8051/src/up_restorecontext.c \
../NuttX/nuttx/arch/8051/src/up_savecontext.c \
../NuttX/nuttx/arch/8051/src/up_timerisr.c \
../NuttX/nuttx/arch/8051/src/up_unblocktask.c 

S_UPPER_SRCS += \
../NuttX/nuttx/arch/8051/src/up_head.S 

OBJS += \
./NuttX/nuttx/arch/8051/src/up_allocateheap.o \
./NuttX/nuttx/arch/8051/src/up_assert.o \
./NuttX/nuttx/arch/8051/src/up_blocktask.o \
./NuttX/nuttx/arch/8051/src/up_debug.o \
./NuttX/nuttx/arch/8051/src/up_delay.o \
./NuttX/nuttx/arch/8051/src/up_exit.o \
./NuttX/nuttx/arch/8051/src/up_head.o \
./NuttX/nuttx/arch/8051/src/up_idle.o \
./NuttX/nuttx/arch/8051/src/up_initialize.o \
./NuttX/nuttx/arch/8051/src/up_initialstate.o \
./NuttX/nuttx/arch/8051/src/up_interruptcontext.o \
./NuttX/nuttx/arch/8051/src/up_irq.o \
./NuttX/nuttx/arch/8051/src/up_irqtest.o \
./NuttX/nuttx/arch/8051/src/up_putc.o \
./NuttX/nuttx/arch/8051/src/up_releasepending.o \
./NuttX/nuttx/arch/8051/src/up_reprioritizertr.o \
./NuttX/nuttx/arch/8051/src/up_restorecontext.o \
./NuttX/nuttx/arch/8051/src/up_savecontext.o \
./NuttX/nuttx/arch/8051/src/up_timerisr.o \
./NuttX/nuttx/arch/8051/src/up_unblocktask.o 

C_DEPS += \
./NuttX/nuttx/arch/8051/src/up_allocateheap.d \
./NuttX/nuttx/arch/8051/src/up_assert.d \
./NuttX/nuttx/arch/8051/src/up_blocktask.d \
./NuttX/nuttx/arch/8051/src/up_debug.d \
./NuttX/nuttx/arch/8051/src/up_delay.d \
./NuttX/nuttx/arch/8051/src/up_exit.d \
./NuttX/nuttx/arch/8051/src/up_idle.d \
./NuttX/nuttx/arch/8051/src/up_initialize.d \
./NuttX/nuttx/arch/8051/src/up_initialstate.d \
./NuttX/nuttx/arch/8051/src/up_interruptcontext.d \
./NuttX/nuttx/arch/8051/src/up_irq.d \
./NuttX/nuttx/arch/8051/src/up_irqtest.d \
./NuttX/nuttx/arch/8051/src/up_putc.d \
./NuttX/nuttx/arch/8051/src/up_releasepending.d \
./NuttX/nuttx/arch/8051/src/up_reprioritizertr.d \
./NuttX/nuttx/arch/8051/src/up_restorecontext.d \
./NuttX/nuttx/arch/8051/src/up_savecontext.d \
./NuttX/nuttx/arch/8051/src/up_timerisr.d \
./NuttX/nuttx/arch/8051/src/up_unblocktask.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/arch/8051/src/%.o: ../NuttX/nuttx/arch/8051/src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

NuttX/nuttx/arch/8051/src/%.o: ../NuttX/nuttx/arch/8051/src/%.S
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Assembler'
	as  -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


