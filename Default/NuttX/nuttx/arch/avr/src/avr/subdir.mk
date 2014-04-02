################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/arch/avr/src/avr/up_blocktask.c \
../NuttX/nuttx/arch/avr/src/avr/up_checkstack.c \
../NuttX/nuttx/arch/avr/src/avr/up_copystate.c \
../NuttX/nuttx/arch/avr/src/avr/up_createstack.c \
../NuttX/nuttx/arch/avr/src/avr/up_doirq.c \
../NuttX/nuttx/arch/avr/src/avr/up_dumpstate.c \
../NuttX/nuttx/arch/avr/src/avr/up_initialstate.c \
../NuttX/nuttx/arch/avr/src/avr/up_irq.c \
../NuttX/nuttx/arch/avr/src/avr/up_releasepending.c \
../NuttX/nuttx/arch/avr/src/avr/up_reprioritizertr.c \
../NuttX/nuttx/arch/avr/src/avr/up_romgetc.c \
../NuttX/nuttx/arch/avr/src/avr/up_schedulesigaction.c \
../NuttX/nuttx/arch/avr/src/avr/up_sigdeliver.c \
../NuttX/nuttx/arch/avr/src/avr/up_spi.c \
../NuttX/nuttx/arch/avr/src/avr/up_stackframe.c \
../NuttX/nuttx/arch/avr/src/avr/up_unblocktask.c \
../NuttX/nuttx/arch/avr/src/avr/up_usestack.c 

S_UPPER_SRCS += \
../NuttX/nuttx/arch/avr/src/avr/up_switchcontext.S 

OBJS += \
./NuttX/nuttx/arch/avr/src/avr/up_blocktask.o \
./NuttX/nuttx/arch/avr/src/avr/up_checkstack.o \
./NuttX/nuttx/arch/avr/src/avr/up_copystate.o \
./NuttX/nuttx/arch/avr/src/avr/up_createstack.o \
./NuttX/nuttx/arch/avr/src/avr/up_doirq.o \
./NuttX/nuttx/arch/avr/src/avr/up_dumpstate.o \
./NuttX/nuttx/arch/avr/src/avr/up_initialstate.o \
./NuttX/nuttx/arch/avr/src/avr/up_irq.o \
./NuttX/nuttx/arch/avr/src/avr/up_releasepending.o \
./NuttX/nuttx/arch/avr/src/avr/up_reprioritizertr.o \
./NuttX/nuttx/arch/avr/src/avr/up_romgetc.o \
./NuttX/nuttx/arch/avr/src/avr/up_schedulesigaction.o \
./NuttX/nuttx/arch/avr/src/avr/up_sigdeliver.o \
./NuttX/nuttx/arch/avr/src/avr/up_spi.o \
./NuttX/nuttx/arch/avr/src/avr/up_stackframe.o \
./NuttX/nuttx/arch/avr/src/avr/up_switchcontext.o \
./NuttX/nuttx/arch/avr/src/avr/up_unblocktask.o \
./NuttX/nuttx/arch/avr/src/avr/up_usestack.o 

C_DEPS += \
./NuttX/nuttx/arch/avr/src/avr/up_blocktask.d \
./NuttX/nuttx/arch/avr/src/avr/up_checkstack.d \
./NuttX/nuttx/arch/avr/src/avr/up_copystate.d \
./NuttX/nuttx/arch/avr/src/avr/up_createstack.d \
./NuttX/nuttx/arch/avr/src/avr/up_doirq.d \
./NuttX/nuttx/arch/avr/src/avr/up_dumpstate.d \
./NuttX/nuttx/arch/avr/src/avr/up_initialstate.d \
./NuttX/nuttx/arch/avr/src/avr/up_irq.d \
./NuttX/nuttx/arch/avr/src/avr/up_releasepending.d \
./NuttX/nuttx/arch/avr/src/avr/up_reprioritizertr.d \
./NuttX/nuttx/arch/avr/src/avr/up_romgetc.d \
./NuttX/nuttx/arch/avr/src/avr/up_schedulesigaction.d \
./NuttX/nuttx/arch/avr/src/avr/up_sigdeliver.d \
./NuttX/nuttx/arch/avr/src/avr/up_spi.d \
./NuttX/nuttx/arch/avr/src/avr/up_stackframe.d \
./NuttX/nuttx/arch/avr/src/avr/up_unblocktask.d \
./NuttX/nuttx/arch/avr/src/avr/up_usestack.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/arch/avr/src/avr/%.o: ../NuttX/nuttx/arch/avr/src/avr/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

NuttX/nuttx/arch/avr/src/avr/%.o: ../NuttX/nuttx/arch/avr/src/avr/%.S
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Assembler'
	as  -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


