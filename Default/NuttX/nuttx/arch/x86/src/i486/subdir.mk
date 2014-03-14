################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/arch/x86/src/i486/up_createstack.c \
../NuttX/nuttx/arch/x86/src/i486/up_initialstate.c \
../NuttX/nuttx/arch/x86/src/i486/up_irq.c \
../NuttX/nuttx/arch/x86/src/i486/up_regdump.c \
../NuttX/nuttx/arch/x86/src/i486/up_releasestack.c \
../NuttX/nuttx/arch/x86/src/i486/up_savestate.c \
../NuttX/nuttx/arch/x86/src/i486/up_schedulesigaction.c \
../NuttX/nuttx/arch/x86/src/i486/up_sigdeliver.c \
../NuttX/nuttx/arch/x86/src/i486/up_stackframe.c \
../NuttX/nuttx/arch/x86/src/i486/up_usestack.c 

S_UPPER_SRCS += \
../NuttX/nuttx/arch/x86/src/i486/i486_utils.S \
../NuttX/nuttx/arch/x86/src/i486/up_syscall6.S 

OBJS += \
./NuttX/nuttx/arch/x86/src/i486/i486_utils.o \
./NuttX/nuttx/arch/x86/src/i486/up_createstack.o \
./NuttX/nuttx/arch/x86/src/i486/up_initialstate.o \
./NuttX/nuttx/arch/x86/src/i486/up_irq.o \
./NuttX/nuttx/arch/x86/src/i486/up_regdump.o \
./NuttX/nuttx/arch/x86/src/i486/up_releasestack.o \
./NuttX/nuttx/arch/x86/src/i486/up_savestate.o \
./NuttX/nuttx/arch/x86/src/i486/up_schedulesigaction.o \
./NuttX/nuttx/arch/x86/src/i486/up_sigdeliver.o \
./NuttX/nuttx/arch/x86/src/i486/up_stackframe.o \
./NuttX/nuttx/arch/x86/src/i486/up_syscall6.o \
./NuttX/nuttx/arch/x86/src/i486/up_usestack.o 

C_DEPS += \
./NuttX/nuttx/arch/x86/src/i486/up_createstack.d \
./NuttX/nuttx/arch/x86/src/i486/up_initialstate.d \
./NuttX/nuttx/arch/x86/src/i486/up_irq.d \
./NuttX/nuttx/arch/x86/src/i486/up_regdump.d \
./NuttX/nuttx/arch/x86/src/i486/up_releasestack.d \
./NuttX/nuttx/arch/x86/src/i486/up_savestate.d \
./NuttX/nuttx/arch/x86/src/i486/up_schedulesigaction.d \
./NuttX/nuttx/arch/x86/src/i486/up_sigdeliver.d \
./NuttX/nuttx/arch/x86/src/i486/up_stackframe.d \
./NuttX/nuttx/arch/x86/src/i486/up_usestack.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/arch/x86/src/i486/%.o: ../NuttX/nuttx/arch/x86/src/i486/%.S
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Assembler'
	as  -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

NuttX/nuttx/arch/x86/src/i486/%.o: ../NuttX/nuttx/arch/x86/src/i486/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


