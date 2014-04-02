################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/arch/avr/src/atmega/atmega_lowconsole.c \
../NuttX/nuttx/arch/avr/src/atmega/atmega_lowinit.c \
../NuttX/nuttx/arch/avr/src/atmega/atmega_serial.c \
../NuttX/nuttx/arch/avr/src/atmega/atmega_timerisr.c 

S_UPPER_SRCS += \
../NuttX/nuttx/arch/avr/src/atmega/atmega_exceptions.S \
../NuttX/nuttx/arch/avr/src/atmega/atmega_head.S 

OBJS += \
./NuttX/nuttx/arch/avr/src/atmega/atmega_exceptions.o \
./NuttX/nuttx/arch/avr/src/atmega/atmega_head.o \
./NuttX/nuttx/arch/avr/src/atmega/atmega_lowconsole.o \
./NuttX/nuttx/arch/avr/src/atmega/atmega_lowinit.o \
./NuttX/nuttx/arch/avr/src/atmega/atmega_serial.o \
./NuttX/nuttx/arch/avr/src/atmega/atmega_timerisr.o 

C_DEPS += \
./NuttX/nuttx/arch/avr/src/atmega/atmega_lowconsole.d \
./NuttX/nuttx/arch/avr/src/atmega/atmega_lowinit.d \
./NuttX/nuttx/arch/avr/src/atmega/atmega_serial.d \
./NuttX/nuttx/arch/avr/src/atmega/atmega_timerisr.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/arch/avr/src/atmega/%.o: ../NuttX/nuttx/arch/avr/src/atmega/%.S
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Assembler'
	as  -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

NuttX/nuttx/arch/avr/src/atmega/%.o: ../NuttX/nuttx/arch/avr/src/atmega/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


