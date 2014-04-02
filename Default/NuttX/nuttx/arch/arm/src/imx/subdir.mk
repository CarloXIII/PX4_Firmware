################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/arch/arm/src/imx/imx_allocateheap.c \
../NuttX/nuttx/arch/arm/src/imx/imx_boot.c \
../NuttX/nuttx/arch/arm/src/imx/imx_decodeirq.c \
../NuttX/nuttx/arch/arm/src/imx/imx_gpio.c \
../NuttX/nuttx/arch/arm/src/imx/imx_irq.c \
../NuttX/nuttx/arch/arm/src/imx/imx_serial.c \
../NuttX/nuttx/arch/arm/src/imx/imx_spi.c \
../NuttX/nuttx/arch/arm/src/imx/imx_timerisr.c 

S_UPPER_SRCS += \
../NuttX/nuttx/arch/arm/src/imx/imx_lowputc.S 

OBJS += \
./NuttX/nuttx/arch/arm/src/imx/imx_allocateheap.o \
./NuttX/nuttx/arch/arm/src/imx/imx_boot.o \
./NuttX/nuttx/arch/arm/src/imx/imx_decodeirq.o \
./NuttX/nuttx/arch/arm/src/imx/imx_gpio.o \
./NuttX/nuttx/arch/arm/src/imx/imx_irq.o \
./NuttX/nuttx/arch/arm/src/imx/imx_lowputc.o \
./NuttX/nuttx/arch/arm/src/imx/imx_serial.o \
./NuttX/nuttx/arch/arm/src/imx/imx_spi.o \
./NuttX/nuttx/arch/arm/src/imx/imx_timerisr.o 

C_DEPS += \
./NuttX/nuttx/arch/arm/src/imx/imx_allocateheap.d \
./NuttX/nuttx/arch/arm/src/imx/imx_boot.d \
./NuttX/nuttx/arch/arm/src/imx/imx_decodeirq.d \
./NuttX/nuttx/arch/arm/src/imx/imx_gpio.d \
./NuttX/nuttx/arch/arm/src/imx/imx_irq.d \
./NuttX/nuttx/arch/arm/src/imx/imx_serial.d \
./NuttX/nuttx/arch/arm/src/imx/imx_spi.d \
./NuttX/nuttx/arch/arm/src/imx/imx_timerisr.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/arch/arm/src/imx/%.o: ../NuttX/nuttx/arch/arm/src/imx/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

NuttX/nuttx/arch/arm/src/imx/%.o: ../NuttX/nuttx/arch/arm/src/imx/%.S
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Assembler'
	as  -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


