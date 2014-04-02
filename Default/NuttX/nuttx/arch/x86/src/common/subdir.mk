################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/arch/x86/src/common/up_allocateheap.c \
../NuttX/nuttx/arch/x86/src/common/up_assert.c \
../NuttX/nuttx/arch/x86/src/common/up_blocktask.c \
../NuttX/nuttx/arch/x86/src/common/up_copystate.c \
../NuttX/nuttx/arch/x86/src/common/up_elf.c \
../NuttX/nuttx/arch/x86/src/common/up_exit.c \
../NuttX/nuttx/arch/x86/src/common/up_initialize.c \
../NuttX/nuttx/arch/x86/src/common/up_interruptcontext.c \
../NuttX/nuttx/arch/x86/src/common/up_lowputs.c \
../NuttX/nuttx/arch/x86/src/common/up_mdelay.c \
../NuttX/nuttx/arch/x86/src/common/up_modifyreg16.c \
../NuttX/nuttx/arch/x86/src/common/up_modifyreg32.c \
../NuttX/nuttx/arch/x86/src/common/up_modifyreg8.c \
../NuttX/nuttx/arch/x86/src/common/up_puts.c \
../NuttX/nuttx/arch/x86/src/common/up_releasepending.c \
../NuttX/nuttx/arch/x86/src/common/up_reprioritizertr.c \
../NuttX/nuttx/arch/x86/src/common/up_udelay.c \
../NuttX/nuttx/arch/x86/src/common/up_unblocktask.c 

OBJS += \
./NuttX/nuttx/arch/x86/src/common/up_allocateheap.o \
./NuttX/nuttx/arch/x86/src/common/up_assert.o \
./NuttX/nuttx/arch/x86/src/common/up_blocktask.o \
./NuttX/nuttx/arch/x86/src/common/up_copystate.o \
./NuttX/nuttx/arch/x86/src/common/up_elf.o \
./NuttX/nuttx/arch/x86/src/common/up_exit.o \
./NuttX/nuttx/arch/x86/src/common/up_initialize.o \
./NuttX/nuttx/arch/x86/src/common/up_interruptcontext.o \
./NuttX/nuttx/arch/x86/src/common/up_lowputs.o \
./NuttX/nuttx/arch/x86/src/common/up_mdelay.o \
./NuttX/nuttx/arch/x86/src/common/up_modifyreg16.o \
./NuttX/nuttx/arch/x86/src/common/up_modifyreg32.o \
./NuttX/nuttx/arch/x86/src/common/up_modifyreg8.o \
./NuttX/nuttx/arch/x86/src/common/up_puts.o \
./NuttX/nuttx/arch/x86/src/common/up_releasepending.o \
./NuttX/nuttx/arch/x86/src/common/up_reprioritizertr.o \
./NuttX/nuttx/arch/x86/src/common/up_udelay.o \
./NuttX/nuttx/arch/x86/src/common/up_unblocktask.o 

C_DEPS += \
./NuttX/nuttx/arch/x86/src/common/up_allocateheap.d \
./NuttX/nuttx/arch/x86/src/common/up_assert.d \
./NuttX/nuttx/arch/x86/src/common/up_blocktask.d \
./NuttX/nuttx/arch/x86/src/common/up_copystate.d \
./NuttX/nuttx/arch/x86/src/common/up_elf.d \
./NuttX/nuttx/arch/x86/src/common/up_exit.d \
./NuttX/nuttx/arch/x86/src/common/up_initialize.d \
./NuttX/nuttx/arch/x86/src/common/up_interruptcontext.d \
./NuttX/nuttx/arch/x86/src/common/up_lowputs.d \
./NuttX/nuttx/arch/x86/src/common/up_mdelay.d \
./NuttX/nuttx/arch/x86/src/common/up_modifyreg16.d \
./NuttX/nuttx/arch/x86/src/common/up_modifyreg32.d \
./NuttX/nuttx/arch/x86/src/common/up_modifyreg8.d \
./NuttX/nuttx/arch/x86/src/common/up_puts.d \
./NuttX/nuttx/arch/x86/src/common/up_releasepending.d \
./NuttX/nuttx/arch/x86/src/common/up_reprioritizertr.d \
./NuttX/nuttx/arch/x86/src/common/up_udelay.d \
./NuttX/nuttx/arch/x86/src/common/up_unblocktask.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/arch/x86/src/common/%.o: ../NuttX/nuttx/arch/x86/src/common/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


