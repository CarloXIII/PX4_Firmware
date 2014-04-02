################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/arch/avr/src/at32uc3/at32uc3_clkinit.c \
../NuttX/nuttx/arch/avr/src/at32uc3/at32uc3_gpio.c \
../NuttX/nuttx/arch/avr/src/at32uc3/at32uc3_gpioirq.c \
../NuttX/nuttx/arch/avr/src/at32uc3/at32uc3_irq.c \
../NuttX/nuttx/arch/avr/src/at32uc3/at32uc3_lowconsole.c \
../NuttX/nuttx/arch/avr/src/at32uc3/at32uc3_lowinit.c \
../NuttX/nuttx/arch/avr/src/at32uc3/at32uc3_serial.c \
../NuttX/nuttx/arch/avr/src/at32uc3/at32uc3_timerisr.c 

OBJS += \
./NuttX/nuttx/arch/avr/src/at32uc3/at32uc3_clkinit.o \
./NuttX/nuttx/arch/avr/src/at32uc3/at32uc3_gpio.o \
./NuttX/nuttx/arch/avr/src/at32uc3/at32uc3_gpioirq.o \
./NuttX/nuttx/arch/avr/src/at32uc3/at32uc3_irq.o \
./NuttX/nuttx/arch/avr/src/at32uc3/at32uc3_lowconsole.o \
./NuttX/nuttx/arch/avr/src/at32uc3/at32uc3_lowinit.o \
./NuttX/nuttx/arch/avr/src/at32uc3/at32uc3_serial.o \
./NuttX/nuttx/arch/avr/src/at32uc3/at32uc3_timerisr.o 

C_DEPS += \
./NuttX/nuttx/arch/avr/src/at32uc3/at32uc3_clkinit.d \
./NuttX/nuttx/arch/avr/src/at32uc3/at32uc3_gpio.d \
./NuttX/nuttx/arch/avr/src/at32uc3/at32uc3_gpioirq.d \
./NuttX/nuttx/arch/avr/src/at32uc3/at32uc3_irq.d \
./NuttX/nuttx/arch/avr/src/at32uc3/at32uc3_lowconsole.d \
./NuttX/nuttx/arch/avr/src/at32uc3/at32uc3_lowinit.d \
./NuttX/nuttx/arch/avr/src/at32uc3/at32uc3_serial.d \
./NuttX/nuttx/arch/avr/src/at32uc3/at32uc3_timerisr.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/arch/avr/src/at32uc3/%.o: ../NuttX/nuttx/arch/avr/src/at32uc3/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


