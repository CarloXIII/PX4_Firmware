################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/configs/xtrs/src/xtr_irq.c \
../NuttX/nuttx/configs/xtrs/src/xtr_lowputc.c \
../NuttX/nuttx/configs/xtrs/src/xtr_serial.c \
../NuttX/nuttx/configs/xtrs/src/xtr_timerisr.c 

ASM_SRCS += \
../NuttX/nuttx/configs/xtrs/src/xtrs_head.asm 

OBJS += \
./NuttX/nuttx/configs/xtrs/src/xtr_irq.o \
./NuttX/nuttx/configs/xtrs/src/xtr_lowputc.o \
./NuttX/nuttx/configs/xtrs/src/xtr_serial.o \
./NuttX/nuttx/configs/xtrs/src/xtr_timerisr.o \
./NuttX/nuttx/configs/xtrs/src/xtrs_head.o 

C_DEPS += \
./NuttX/nuttx/configs/xtrs/src/xtr_irq.d \
./NuttX/nuttx/configs/xtrs/src/xtr_lowputc.d \
./NuttX/nuttx/configs/xtrs/src/xtr_serial.d \
./NuttX/nuttx/configs/xtrs/src/xtr_timerisr.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/configs/xtrs/src/%.o: ../NuttX/nuttx/configs/xtrs/src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

NuttX/nuttx/configs/xtrs/src/%.o: ../NuttX/nuttx/configs/xtrs/src/%.asm
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Assembler'
	as  -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


