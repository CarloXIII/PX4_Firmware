################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/arch/avr/src/avr32/up_blocktask.c \
../NuttX/nuttx/arch/avr/src/avr32/up_copystate.c \
../NuttX/nuttx/arch/avr/src/avr32/up_createstack.c \
../NuttX/nuttx/arch/avr/src/avr32/up_doirq.c \
../NuttX/nuttx/arch/avr/src/avr32/up_dumpstate.c \
../NuttX/nuttx/arch/avr/src/avr32/up_initialstate.c \
../NuttX/nuttx/arch/avr/src/avr32/up_releasepending.c \
../NuttX/nuttx/arch/avr/src/avr32/up_reprioritizertr.c \
../NuttX/nuttx/arch/avr/src/avr32/up_schedulesigaction.c \
../NuttX/nuttx/arch/avr/src/avr32/up_sigdeliver.c \
../NuttX/nuttx/arch/avr/src/avr32/up_stackframe.c \
../NuttX/nuttx/arch/avr/src/avr32/up_unblocktask.c \
../NuttX/nuttx/arch/avr/src/avr32/up_usestack.c 

S_UPPER_SRCS += \
../NuttX/nuttx/arch/avr/src/avr32/up_exceptions.S \
../NuttX/nuttx/arch/avr/src/avr32/up_fullcontextrestore.S \
../NuttX/nuttx/arch/avr/src/avr32/up_nommuhead.S \
../NuttX/nuttx/arch/avr/src/avr32/up_switchcontext.S \
../NuttX/nuttx/arch/avr/src/avr32/up_syscall6.S 

OBJS += \
./NuttX/nuttx/arch/avr/src/avr32/up_blocktask.o \
./NuttX/nuttx/arch/avr/src/avr32/up_copystate.o \
./NuttX/nuttx/arch/avr/src/avr32/up_createstack.o \
./NuttX/nuttx/arch/avr/src/avr32/up_doirq.o \
./NuttX/nuttx/arch/avr/src/avr32/up_dumpstate.o \
./NuttX/nuttx/arch/avr/src/avr32/up_exceptions.o \
./NuttX/nuttx/arch/avr/src/avr32/up_fullcontextrestore.o \
./NuttX/nuttx/arch/avr/src/avr32/up_initialstate.o \
./NuttX/nuttx/arch/avr/src/avr32/up_nommuhead.o \
./NuttX/nuttx/arch/avr/src/avr32/up_releasepending.o \
./NuttX/nuttx/arch/avr/src/avr32/up_reprioritizertr.o \
./NuttX/nuttx/arch/avr/src/avr32/up_schedulesigaction.o \
./NuttX/nuttx/arch/avr/src/avr32/up_sigdeliver.o \
./NuttX/nuttx/arch/avr/src/avr32/up_stackframe.o \
./NuttX/nuttx/arch/avr/src/avr32/up_switchcontext.o \
./NuttX/nuttx/arch/avr/src/avr32/up_syscall6.o \
./NuttX/nuttx/arch/avr/src/avr32/up_unblocktask.o \
./NuttX/nuttx/arch/avr/src/avr32/up_usestack.o 

C_DEPS += \
./NuttX/nuttx/arch/avr/src/avr32/up_blocktask.d \
./NuttX/nuttx/arch/avr/src/avr32/up_copystate.d \
./NuttX/nuttx/arch/avr/src/avr32/up_createstack.d \
./NuttX/nuttx/arch/avr/src/avr32/up_doirq.d \
./NuttX/nuttx/arch/avr/src/avr32/up_dumpstate.d \
./NuttX/nuttx/arch/avr/src/avr32/up_initialstate.d \
./NuttX/nuttx/arch/avr/src/avr32/up_releasepending.d \
./NuttX/nuttx/arch/avr/src/avr32/up_reprioritizertr.d \
./NuttX/nuttx/arch/avr/src/avr32/up_schedulesigaction.d \
./NuttX/nuttx/arch/avr/src/avr32/up_sigdeliver.d \
./NuttX/nuttx/arch/avr/src/avr32/up_stackframe.d \
./NuttX/nuttx/arch/avr/src/avr32/up_unblocktask.d \
./NuttX/nuttx/arch/avr/src/avr32/up_usestack.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/arch/avr/src/avr32/%.o: ../NuttX/nuttx/arch/avr/src/avr32/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

NuttX/nuttx/arch/avr/src/avr32/%.o: ../NuttX/nuttx/arch/avr/src/avr32/%.S
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Assembler'
	as  -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


