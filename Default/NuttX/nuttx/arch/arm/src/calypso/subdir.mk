################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/arch/arm/src/calypso/calypso_armio.c \
../NuttX/nuttx/arch/arm/src/calypso/calypso_heap.c \
../NuttX/nuttx/arch/arm/src/calypso/calypso_irq.c \
../NuttX/nuttx/arch/arm/src/calypso/calypso_keypad.c \
../NuttX/nuttx/arch/arm/src/calypso/calypso_power.c \
../NuttX/nuttx/arch/arm/src/calypso/calypso_serial.c \
../NuttX/nuttx/arch/arm/src/calypso/calypso_spi.c \
../NuttX/nuttx/arch/arm/src/calypso/calypso_timer.c \
../NuttX/nuttx/arch/arm/src/calypso/calypso_uwire.c \
../NuttX/nuttx/arch/arm/src/calypso/clock.c 

S_UPPER_SRCS += \
../NuttX/nuttx/arch/arm/src/calypso/calypso_head.S \
../NuttX/nuttx/arch/arm/src/calypso/calypso_lowputc.S 

OBJS += \
./NuttX/nuttx/arch/arm/src/calypso/calypso_armio.o \
./NuttX/nuttx/arch/arm/src/calypso/calypso_head.o \
./NuttX/nuttx/arch/arm/src/calypso/calypso_heap.o \
./NuttX/nuttx/arch/arm/src/calypso/calypso_irq.o \
./NuttX/nuttx/arch/arm/src/calypso/calypso_keypad.o \
./NuttX/nuttx/arch/arm/src/calypso/calypso_lowputc.o \
./NuttX/nuttx/arch/arm/src/calypso/calypso_power.o \
./NuttX/nuttx/arch/arm/src/calypso/calypso_serial.o \
./NuttX/nuttx/arch/arm/src/calypso/calypso_spi.o \
./NuttX/nuttx/arch/arm/src/calypso/calypso_timer.o \
./NuttX/nuttx/arch/arm/src/calypso/calypso_uwire.o \
./NuttX/nuttx/arch/arm/src/calypso/clock.o 

C_DEPS += \
./NuttX/nuttx/arch/arm/src/calypso/calypso_armio.d \
./NuttX/nuttx/arch/arm/src/calypso/calypso_heap.d \
./NuttX/nuttx/arch/arm/src/calypso/calypso_irq.d \
./NuttX/nuttx/arch/arm/src/calypso/calypso_keypad.d \
./NuttX/nuttx/arch/arm/src/calypso/calypso_power.d \
./NuttX/nuttx/arch/arm/src/calypso/calypso_serial.d \
./NuttX/nuttx/arch/arm/src/calypso/calypso_spi.d \
./NuttX/nuttx/arch/arm/src/calypso/calypso_timer.d \
./NuttX/nuttx/arch/arm/src/calypso/calypso_uwire.d \
./NuttX/nuttx/arch/arm/src/calypso/clock.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/arch/arm/src/calypso/%.o: ../NuttX/nuttx/arch/arm/src/calypso/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

NuttX/nuttx/arch/arm/src/calypso/%.o: ../NuttX/nuttx/arch/arm/src/calypso/%.S
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Assembler'
	as  -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


