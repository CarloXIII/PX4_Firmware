################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/arch/x86/src/qemu/qemu_handlers.c \
../NuttX/nuttx/arch/x86/src/qemu/qemu_idle.c \
../NuttX/nuttx/arch/x86/src/qemu/qemu_lowputc.c \
../NuttX/nuttx/arch/x86/src/qemu/qemu_lowsetup.c \
../NuttX/nuttx/arch/x86/src/qemu/qemu_serial.c \
../NuttX/nuttx/arch/x86/src/qemu/qemu_timerisr.c 

S_UPPER_SRCS += \
../NuttX/nuttx/arch/x86/src/qemu/qemu_fullcontextrestore.S \
../NuttX/nuttx/arch/x86/src/qemu/qemu_head.S \
../NuttX/nuttx/arch/x86/src/qemu/qemu_saveusercontext.S \
../NuttX/nuttx/arch/x86/src/qemu/qemu_vectors.S 

OBJS += \
./NuttX/nuttx/arch/x86/src/qemu/qemu_fullcontextrestore.o \
./NuttX/nuttx/arch/x86/src/qemu/qemu_handlers.o \
./NuttX/nuttx/arch/x86/src/qemu/qemu_head.o \
./NuttX/nuttx/arch/x86/src/qemu/qemu_idle.o \
./NuttX/nuttx/arch/x86/src/qemu/qemu_lowputc.o \
./NuttX/nuttx/arch/x86/src/qemu/qemu_lowsetup.o \
./NuttX/nuttx/arch/x86/src/qemu/qemu_saveusercontext.o \
./NuttX/nuttx/arch/x86/src/qemu/qemu_serial.o \
./NuttX/nuttx/arch/x86/src/qemu/qemu_timerisr.o \
./NuttX/nuttx/arch/x86/src/qemu/qemu_vectors.o 

C_DEPS += \
./NuttX/nuttx/arch/x86/src/qemu/qemu_handlers.d \
./NuttX/nuttx/arch/x86/src/qemu/qemu_idle.d \
./NuttX/nuttx/arch/x86/src/qemu/qemu_lowputc.d \
./NuttX/nuttx/arch/x86/src/qemu/qemu_lowsetup.d \
./NuttX/nuttx/arch/x86/src/qemu/qemu_serial.d \
./NuttX/nuttx/arch/x86/src/qemu/qemu_timerisr.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/arch/x86/src/qemu/%.o: ../NuttX/nuttx/arch/x86/src/qemu/%.S
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Assembler'
	as  -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

NuttX/nuttx/arch/x86/src/qemu/%.o: ../NuttX/nuttx/arch/x86/src/qemu/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


