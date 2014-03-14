################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/arch/z16/src/common/up_allocateheap.c \
../NuttX/nuttx/arch/z16/src/common/up_assert.c \
../NuttX/nuttx/arch/z16/src/common/up_blocktask.c \
../NuttX/nuttx/arch/z16/src/common/up_copystate.c \
../NuttX/nuttx/arch/z16/src/common/up_createstack.c \
../NuttX/nuttx/arch/z16/src/common/up_doirq.c \
../NuttX/nuttx/arch/z16/src/common/up_exit.c \
../NuttX/nuttx/arch/z16/src/common/up_idle.c \
../NuttX/nuttx/arch/z16/src/common/up_initialize.c \
../NuttX/nuttx/arch/z16/src/common/up_initialstate.c \
../NuttX/nuttx/arch/z16/src/common/up_interruptcontext.c \
../NuttX/nuttx/arch/z16/src/common/up_mdelay.c \
../NuttX/nuttx/arch/z16/src/common/up_registerdump.c \
../NuttX/nuttx/arch/z16/src/common/up_releasepending.c \
../NuttX/nuttx/arch/z16/src/common/up_releasestack.c \
../NuttX/nuttx/arch/z16/src/common/up_reprioritizertr.c \
../NuttX/nuttx/arch/z16/src/common/up_schedulesigaction.c \
../NuttX/nuttx/arch/z16/src/common/up_sigdeliver.c \
../NuttX/nuttx/arch/z16/src/common/up_stackdump.c \
../NuttX/nuttx/arch/z16/src/common/up_stackframe.c \
../NuttX/nuttx/arch/z16/src/common/up_udelay.c \
../NuttX/nuttx/arch/z16/src/common/up_unblocktask.c \
../NuttX/nuttx/arch/z16/src/common/up_usestack.c 

OBJS += \
./NuttX/nuttx/arch/z16/src/common/up_allocateheap.o \
./NuttX/nuttx/arch/z16/src/common/up_assert.o \
./NuttX/nuttx/arch/z16/src/common/up_blocktask.o \
./NuttX/nuttx/arch/z16/src/common/up_copystate.o \
./NuttX/nuttx/arch/z16/src/common/up_createstack.o \
./NuttX/nuttx/arch/z16/src/common/up_doirq.o \
./NuttX/nuttx/arch/z16/src/common/up_exit.o \
./NuttX/nuttx/arch/z16/src/common/up_idle.o \
./NuttX/nuttx/arch/z16/src/common/up_initialize.o \
./NuttX/nuttx/arch/z16/src/common/up_initialstate.o \
./NuttX/nuttx/arch/z16/src/common/up_interruptcontext.o \
./NuttX/nuttx/arch/z16/src/common/up_mdelay.o \
./NuttX/nuttx/arch/z16/src/common/up_registerdump.o \
./NuttX/nuttx/arch/z16/src/common/up_releasepending.o \
./NuttX/nuttx/arch/z16/src/common/up_releasestack.o \
./NuttX/nuttx/arch/z16/src/common/up_reprioritizertr.o \
./NuttX/nuttx/arch/z16/src/common/up_schedulesigaction.o \
./NuttX/nuttx/arch/z16/src/common/up_sigdeliver.o \
./NuttX/nuttx/arch/z16/src/common/up_stackdump.o \
./NuttX/nuttx/arch/z16/src/common/up_stackframe.o \
./NuttX/nuttx/arch/z16/src/common/up_udelay.o \
./NuttX/nuttx/arch/z16/src/common/up_unblocktask.o \
./NuttX/nuttx/arch/z16/src/common/up_usestack.o 

C_DEPS += \
./NuttX/nuttx/arch/z16/src/common/up_allocateheap.d \
./NuttX/nuttx/arch/z16/src/common/up_assert.d \
./NuttX/nuttx/arch/z16/src/common/up_blocktask.d \
./NuttX/nuttx/arch/z16/src/common/up_copystate.d \
./NuttX/nuttx/arch/z16/src/common/up_createstack.d \
./NuttX/nuttx/arch/z16/src/common/up_doirq.d \
./NuttX/nuttx/arch/z16/src/common/up_exit.d \
./NuttX/nuttx/arch/z16/src/common/up_idle.d \
./NuttX/nuttx/arch/z16/src/common/up_initialize.d \
./NuttX/nuttx/arch/z16/src/common/up_initialstate.d \
./NuttX/nuttx/arch/z16/src/common/up_interruptcontext.d \
./NuttX/nuttx/arch/z16/src/common/up_mdelay.d \
./NuttX/nuttx/arch/z16/src/common/up_registerdump.d \
./NuttX/nuttx/arch/z16/src/common/up_releasepending.d \
./NuttX/nuttx/arch/z16/src/common/up_releasestack.d \
./NuttX/nuttx/arch/z16/src/common/up_reprioritizertr.d \
./NuttX/nuttx/arch/z16/src/common/up_schedulesigaction.d \
./NuttX/nuttx/arch/z16/src/common/up_sigdeliver.d \
./NuttX/nuttx/arch/z16/src/common/up_stackdump.d \
./NuttX/nuttx/arch/z16/src/common/up_stackframe.d \
./NuttX/nuttx/arch/z16/src/common/up_udelay.d \
./NuttX/nuttx/arch/z16/src/common/up_unblocktask.d \
./NuttX/nuttx/arch/z16/src/common/up_usestack.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/arch/z16/src/common/%.o: ../NuttX/nuttx/arch/z16/src/common/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


