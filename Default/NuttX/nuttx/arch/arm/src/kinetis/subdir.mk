################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/arch/arm/src/kinetis/kinetis_allocateheap.c \
../NuttX/nuttx/arch/arm/src/kinetis/kinetis_clockconfig.c \
../NuttX/nuttx/arch/arm/src/kinetis/kinetis_clrpend.c \
../NuttX/nuttx/arch/arm/src/kinetis/kinetis_enet.c \
../NuttX/nuttx/arch/arm/src/kinetis/kinetis_idle.c \
../NuttX/nuttx/arch/arm/src/kinetis/kinetis_irq.c \
../NuttX/nuttx/arch/arm/src/kinetis/kinetis_lowputc.c \
../NuttX/nuttx/arch/arm/src/kinetis/kinetis_mpuinit.c \
../NuttX/nuttx/arch/arm/src/kinetis/kinetis_pin.c \
../NuttX/nuttx/arch/arm/src/kinetis/kinetis_pindma.c \
../NuttX/nuttx/arch/arm/src/kinetis/kinetis_pingpio.c \
../NuttX/nuttx/arch/arm/src/kinetis/kinetis_pinirq.c \
../NuttX/nuttx/arch/arm/src/kinetis/kinetis_sdhc.c \
../NuttX/nuttx/arch/arm/src/kinetis/kinetis_serial.c \
../NuttX/nuttx/arch/arm/src/kinetis/kinetis_start.c \
../NuttX/nuttx/arch/arm/src/kinetis/kinetis_timerisr.c \
../NuttX/nuttx/arch/arm/src/kinetis/kinetis_userspace.c \
../NuttX/nuttx/arch/arm/src/kinetis/kinetis_wdog.c 

S_UPPER_SRCS += \
../NuttX/nuttx/arch/arm/src/kinetis/kinetis_vectors.S 

OBJS += \
./NuttX/nuttx/arch/arm/src/kinetis/kinetis_allocateheap.o \
./NuttX/nuttx/arch/arm/src/kinetis/kinetis_clockconfig.o \
./NuttX/nuttx/arch/arm/src/kinetis/kinetis_clrpend.o \
./NuttX/nuttx/arch/arm/src/kinetis/kinetis_enet.o \
./NuttX/nuttx/arch/arm/src/kinetis/kinetis_idle.o \
./NuttX/nuttx/arch/arm/src/kinetis/kinetis_irq.o \
./NuttX/nuttx/arch/arm/src/kinetis/kinetis_lowputc.o \
./NuttX/nuttx/arch/arm/src/kinetis/kinetis_mpuinit.o \
./NuttX/nuttx/arch/arm/src/kinetis/kinetis_pin.o \
./NuttX/nuttx/arch/arm/src/kinetis/kinetis_pindma.o \
./NuttX/nuttx/arch/arm/src/kinetis/kinetis_pingpio.o \
./NuttX/nuttx/arch/arm/src/kinetis/kinetis_pinirq.o \
./NuttX/nuttx/arch/arm/src/kinetis/kinetis_sdhc.o \
./NuttX/nuttx/arch/arm/src/kinetis/kinetis_serial.o \
./NuttX/nuttx/arch/arm/src/kinetis/kinetis_start.o \
./NuttX/nuttx/arch/arm/src/kinetis/kinetis_timerisr.o \
./NuttX/nuttx/arch/arm/src/kinetis/kinetis_userspace.o \
./NuttX/nuttx/arch/arm/src/kinetis/kinetis_vectors.o \
./NuttX/nuttx/arch/arm/src/kinetis/kinetis_wdog.o 

C_DEPS += \
./NuttX/nuttx/arch/arm/src/kinetis/kinetis_allocateheap.d \
./NuttX/nuttx/arch/arm/src/kinetis/kinetis_clockconfig.d \
./NuttX/nuttx/arch/arm/src/kinetis/kinetis_clrpend.d \
./NuttX/nuttx/arch/arm/src/kinetis/kinetis_enet.d \
./NuttX/nuttx/arch/arm/src/kinetis/kinetis_idle.d \
./NuttX/nuttx/arch/arm/src/kinetis/kinetis_irq.d \
./NuttX/nuttx/arch/arm/src/kinetis/kinetis_lowputc.d \
./NuttX/nuttx/arch/arm/src/kinetis/kinetis_mpuinit.d \
./NuttX/nuttx/arch/arm/src/kinetis/kinetis_pin.d \
./NuttX/nuttx/arch/arm/src/kinetis/kinetis_pindma.d \
./NuttX/nuttx/arch/arm/src/kinetis/kinetis_pingpio.d \
./NuttX/nuttx/arch/arm/src/kinetis/kinetis_pinirq.d \
./NuttX/nuttx/arch/arm/src/kinetis/kinetis_sdhc.d \
./NuttX/nuttx/arch/arm/src/kinetis/kinetis_serial.d \
./NuttX/nuttx/arch/arm/src/kinetis/kinetis_start.d \
./NuttX/nuttx/arch/arm/src/kinetis/kinetis_timerisr.d \
./NuttX/nuttx/arch/arm/src/kinetis/kinetis_userspace.d \
./NuttX/nuttx/arch/arm/src/kinetis/kinetis_wdog.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/arch/arm/src/kinetis/%.o: ../NuttX/nuttx/arch/arm/src/kinetis/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

NuttX/nuttx/arch/arm/src/kinetis/%.o: ../NuttX/nuttx/arch/arm/src/kinetis/%.S
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Assembler'
	as  -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


