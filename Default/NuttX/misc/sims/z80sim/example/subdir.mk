################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
ASM_SRCS += \
../NuttX/misc/sims/z80sim/example/example.asm 

OBJS += \
./NuttX/misc/sims/z80sim/example/example.o 


# Each subdirectory must supply rules for building sources it contributes
NuttX/misc/sims/z80sim/example/%.o: ../NuttX/misc/sims/z80sim/example/%.asm
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Assembler'
	as  -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


