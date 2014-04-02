################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/arch/arm/src/sam34/sam3u_clockconfig.c \
../NuttX/nuttx/arch/arm/src/sam34/sam3u_gpio.c \
../NuttX/nuttx/arch/arm/src/sam34/sam4l_clockconfig.c \
../NuttX/nuttx/arch/arm/src/sam34/sam4l_gpio.c \
../NuttX/nuttx/arch/arm/src/sam34/sam4l_periphclks.c \
../NuttX/nuttx/arch/arm/src/sam34/sam_allocateheap.c \
../NuttX/nuttx/arch/arm/src/sam34/sam_dmac.c \
../NuttX/nuttx/arch/arm/src/sam34/sam_gpioirq.c \
../NuttX/nuttx/arch/arm/src/sam34/sam_hsmci.c \
../NuttX/nuttx/arch/arm/src/sam34/sam_irq.c \
../NuttX/nuttx/arch/arm/src/sam34/sam_lowputc.c \
../NuttX/nuttx/arch/arm/src/sam34/sam_mpuinit.c \
../NuttX/nuttx/arch/arm/src/sam34/sam_serial.c \
../NuttX/nuttx/arch/arm/src/sam34/sam_spi.c \
../NuttX/nuttx/arch/arm/src/sam34/sam_start.c \
../NuttX/nuttx/arch/arm/src/sam34/sam_timerisr.c \
../NuttX/nuttx/arch/arm/src/sam34/sam_userspace.c 

S_UPPER_SRCS += \
../NuttX/nuttx/arch/arm/src/sam34/sam_vectors.S 

OBJS += \
./NuttX/nuttx/arch/arm/src/sam34/sam3u_clockconfig.o \
./NuttX/nuttx/arch/arm/src/sam34/sam3u_gpio.o \
./NuttX/nuttx/arch/arm/src/sam34/sam4l_clockconfig.o \
./NuttX/nuttx/arch/arm/src/sam34/sam4l_gpio.o \
./NuttX/nuttx/arch/arm/src/sam34/sam4l_periphclks.o \
./NuttX/nuttx/arch/arm/src/sam34/sam_allocateheap.o \
./NuttX/nuttx/arch/arm/src/sam34/sam_dmac.o \
./NuttX/nuttx/arch/arm/src/sam34/sam_gpioirq.o \
./NuttX/nuttx/arch/arm/src/sam34/sam_hsmci.o \
./NuttX/nuttx/arch/arm/src/sam34/sam_irq.o \
./NuttX/nuttx/arch/arm/src/sam34/sam_lowputc.o \
./NuttX/nuttx/arch/arm/src/sam34/sam_mpuinit.o \
./NuttX/nuttx/arch/arm/src/sam34/sam_serial.o \
./NuttX/nuttx/arch/arm/src/sam34/sam_spi.o \
./NuttX/nuttx/arch/arm/src/sam34/sam_start.o \
./NuttX/nuttx/arch/arm/src/sam34/sam_timerisr.o \
./NuttX/nuttx/arch/arm/src/sam34/sam_userspace.o \
./NuttX/nuttx/arch/arm/src/sam34/sam_vectors.o 

C_DEPS += \
./NuttX/nuttx/arch/arm/src/sam34/sam3u_clockconfig.d \
./NuttX/nuttx/arch/arm/src/sam34/sam3u_gpio.d \
./NuttX/nuttx/arch/arm/src/sam34/sam4l_clockconfig.d \
./NuttX/nuttx/arch/arm/src/sam34/sam4l_gpio.d \
./NuttX/nuttx/arch/arm/src/sam34/sam4l_periphclks.d \
./NuttX/nuttx/arch/arm/src/sam34/sam_allocateheap.d \
./NuttX/nuttx/arch/arm/src/sam34/sam_dmac.d \
./NuttX/nuttx/arch/arm/src/sam34/sam_gpioirq.d \
./NuttX/nuttx/arch/arm/src/sam34/sam_hsmci.d \
./NuttX/nuttx/arch/arm/src/sam34/sam_irq.d \
./NuttX/nuttx/arch/arm/src/sam34/sam_lowputc.d \
./NuttX/nuttx/arch/arm/src/sam34/sam_mpuinit.d \
./NuttX/nuttx/arch/arm/src/sam34/sam_serial.d \
./NuttX/nuttx/arch/arm/src/sam34/sam_spi.d \
./NuttX/nuttx/arch/arm/src/sam34/sam_start.d \
./NuttX/nuttx/arch/arm/src/sam34/sam_timerisr.d \
./NuttX/nuttx/arch/arm/src/sam34/sam_userspace.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/arch/arm/src/sam34/%.o: ../NuttX/nuttx/arch/arm/src/sam34/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

NuttX/nuttx/arch/arm/src/sam34/%.o: ../NuttX/nuttx/arch/arm/src/sam34/%.S
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Assembler'
	as  -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


