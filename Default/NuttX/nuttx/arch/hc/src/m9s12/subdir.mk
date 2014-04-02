################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/arch/hc/src/m9s12/m9s12_assert.c \
../NuttX/nuttx/arch/hc/src/m9s12/m9s12_dumpgpio.c \
../NuttX/nuttx/arch/hc/src/m9s12/m9s12_ethernet.c \
../NuttX/nuttx/arch/hc/src/m9s12/m9s12_gpio.c \
../NuttX/nuttx/arch/hc/src/m9s12/m9s12_gpioirq.c \
../NuttX/nuttx/arch/hc/src/m9s12/m9s12_initialstate.c \
../NuttX/nuttx/arch/hc/src/m9s12/m9s12_irq.c \
../NuttX/nuttx/arch/hc/src/m9s12/m9s12_serial.c \
../NuttX/nuttx/arch/hc/src/m9s12/m9s12_timerisr.c 

S_UPPER_SRCS += \
../NuttX/nuttx/arch/hc/src/m9s12/m9s12_lowputc.S \
../NuttX/nuttx/arch/hc/src/m9s12/m9s12_saveusercontext.S \
../NuttX/nuttx/arch/hc/src/m9s12/m9s12_start.S \
../NuttX/nuttx/arch/hc/src/m9s12/m9s12_vectors.S 

OBJS += \
./NuttX/nuttx/arch/hc/src/m9s12/m9s12_assert.o \
./NuttX/nuttx/arch/hc/src/m9s12/m9s12_dumpgpio.o \
./NuttX/nuttx/arch/hc/src/m9s12/m9s12_ethernet.o \
./NuttX/nuttx/arch/hc/src/m9s12/m9s12_gpio.o \
./NuttX/nuttx/arch/hc/src/m9s12/m9s12_gpioirq.o \
./NuttX/nuttx/arch/hc/src/m9s12/m9s12_initialstate.o \
./NuttX/nuttx/arch/hc/src/m9s12/m9s12_irq.o \
./NuttX/nuttx/arch/hc/src/m9s12/m9s12_lowputc.o \
./NuttX/nuttx/arch/hc/src/m9s12/m9s12_saveusercontext.o \
./NuttX/nuttx/arch/hc/src/m9s12/m9s12_serial.o \
./NuttX/nuttx/arch/hc/src/m9s12/m9s12_start.o \
./NuttX/nuttx/arch/hc/src/m9s12/m9s12_timerisr.o \
./NuttX/nuttx/arch/hc/src/m9s12/m9s12_vectors.o 

C_DEPS += \
./NuttX/nuttx/arch/hc/src/m9s12/m9s12_assert.d \
./NuttX/nuttx/arch/hc/src/m9s12/m9s12_dumpgpio.d \
./NuttX/nuttx/arch/hc/src/m9s12/m9s12_ethernet.d \
./NuttX/nuttx/arch/hc/src/m9s12/m9s12_gpio.d \
./NuttX/nuttx/arch/hc/src/m9s12/m9s12_gpioirq.d \
./NuttX/nuttx/arch/hc/src/m9s12/m9s12_initialstate.d \
./NuttX/nuttx/arch/hc/src/m9s12/m9s12_irq.d \
./NuttX/nuttx/arch/hc/src/m9s12/m9s12_serial.d \
./NuttX/nuttx/arch/hc/src/m9s12/m9s12_timerisr.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/arch/hc/src/m9s12/%.o: ../NuttX/nuttx/arch/hc/src/m9s12/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

NuttX/nuttx/arch/hc/src/m9s12/%.o: ../NuttX/nuttx/arch/hc/src/m9s12/%.S
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Assembler'
	as  -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


