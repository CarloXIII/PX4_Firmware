################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/arch/arm/src/str71x/str71x_decodeirq.c \
../NuttX/nuttx/arch/arm/src/str71x/str71x_irq.c \
../NuttX/nuttx/arch/arm/src/str71x/str71x_lowputc.c \
../NuttX/nuttx/arch/arm/src/str71x/str71x_prccu.c \
../NuttX/nuttx/arch/arm/src/str71x/str71x_serial.c \
../NuttX/nuttx/arch/arm/src/str71x/str71x_timerisr.c \
../NuttX/nuttx/arch/arm/src/str71x/str71x_xti.c 

S_UPPER_SRCS += \
../NuttX/nuttx/arch/arm/src/str71x/str71x_head.S 

OBJS += \
./NuttX/nuttx/arch/arm/src/str71x/str71x_decodeirq.o \
./NuttX/nuttx/arch/arm/src/str71x/str71x_head.o \
./NuttX/nuttx/arch/arm/src/str71x/str71x_irq.o \
./NuttX/nuttx/arch/arm/src/str71x/str71x_lowputc.o \
./NuttX/nuttx/arch/arm/src/str71x/str71x_prccu.o \
./NuttX/nuttx/arch/arm/src/str71x/str71x_serial.o \
./NuttX/nuttx/arch/arm/src/str71x/str71x_timerisr.o \
./NuttX/nuttx/arch/arm/src/str71x/str71x_xti.o 

C_DEPS += \
./NuttX/nuttx/arch/arm/src/str71x/str71x_decodeirq.d \
./NuttX/nuttx/arch/arm/src/str71x/str71x_irq.d \
./NuttX/nuttx/arch/arm/src/str71x/str71x_lowputc.d \
./NuttX/nuttx/arch/arm/src/str71x/str71x_prccu.d \
./NuttX/nuttx/arch/arm/src/str71x/str71x_serial.d \
./NuttX/nuttx/arch/arm/src/str71x/str71x_timerisr.d \
./NuttX/nuttx/arch/arm/src/str71x/str71x_xti.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/arch/arm/src/str71x/%.o: ../NuttX/nuttx/arch/arm/src/str71x/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

NuttX/nuttx/arch/arm/src/str71x/%.o: ../NuttX/nuttx/arch/arm/src/str71x/%.S
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Assembler'
	as  -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


