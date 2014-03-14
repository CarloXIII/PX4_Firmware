################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/arch/z16/src/z16f/z16f_clkinit.c \
../NuttX/nuttx/arch/z16/src/z16f/z16f_irq.c \
../NuttX/nuttx/arch/z16/src/z16f/z16f_serial.c \
../NuttX/nuttx/arch/z16/src/z16f/z16f_sysexec.c \
../NuttX/nuttx/arch/z16/src/z16f/z16f_timerisr.c 

S_UPPER_SRCS += \
../NuttX/nuttx/arch/z16/src/z16f/z16f_head.S \
../NuttX/nuttx/arch/z16/src/z16f/z16f_lowuart.S \
../NuttX/nuttx/arch/z16/src/z16f/z16f_restoreusercontext.S \
../NuttX/nuttx/arch/z16/src/z16f/z16f_saveusercontext.S 

OBJS += \
./NuttX/nuttx/arch/z16/src/z16f/z16f_clkinit.o \
./NuttX/nuttx/arch/z16/src/z16f/z16f_head.o \
./NuttX/nuttx/arch/z16/src/z16f/z16f_irq.o \
./NuttX/nuttx/arch/z16/src/z16f/z16f_lowuart.o \
./NuttX/nuttx/arch/z16/src/z16f/z16f_restoreusercontext.o \
./NuttX/nuttx/arch/z16/src/z16f/z16f_saveusercontext.o \
./NuttX/nuttx/arch/z16/src/z16f/z16f_serial.o \
./NuttX/nuttx/arch/z16/src/z16f/z16f_sysexec.o \
./NuttX/nuttx/arch/z16/src/z16f/z16f_timerisr.o 

C_DEPS += \
./NuttX/nuttx/arch/z16/src/z16f/z16f_clkinit.d \
./NuttX/nuttx/arch/z16/src/z16f/z16f_irq.d \
./NuttX/nuttx/arch/z16/src/z16f/z16f_serial.d \
./NuttX/nuttx/arch/z16/src/z16f/z16f_sysexec.d \
./NuttX/nuttx/arch/z16/src/z16f/z16f_timerisr.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/arch/z16/src/z16f/%.o: ../NuttX/nuttx/arch/z16/src/z16f/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

NuttX/nuttx/arch/z16/src/z16f/%.o: ../NuttX/nuttx/arch/z16/src/z16f/%.S
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Assembler'
	as  -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


