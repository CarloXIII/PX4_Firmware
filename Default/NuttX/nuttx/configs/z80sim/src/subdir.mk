################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/configs/z80sim/src/z80_irq.c \
../NuttX/nuttx/configs/z80sim/src/z80_lowputc.c \
../NuttX/nuttx/configs/z80sim/src/z80_serial.c \
../NuttX/nuttx/configs/z80sim/src/z80_timerisr.c 

OBJS += \
./NuttX/nuttx/configs/z80sim/src/z80_irq.o \
./NuttX/nuttx/configs/z80sim/src/z80_lowputc.o \
./NuttX/nuttx/configs/z80sim/src/z80_serial.o \
./NuttX/nuttx/configs/z80sim/src/z80_timerisr.o 

C_DEPS += \
./NuttX/nuttx/configs/z80sim/src/z80_irq.d \
./NuttX/nuttx/configs/z80sim/src/z80_lowputc.d \
./NuttX/nuttx/configs/z80sim/src/z80_serial.d \
./NuttX/nuttx/configs/z80sim/src/z80_timerisr.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/configs/z80sim/src/%.o: ../NuttX/nuttx/configs/z80sim/src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


