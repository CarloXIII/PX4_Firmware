################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/misc/pascal/insn16/prun/pdbg.c \
../NuttX/misc/pascal/insn16/prun/pexec.c \
../NuttX/misc/pascal/insn16/prun/pload.c \
../NuttX/misc/pascal/insn16/prun/prun.c 

OBJS += \
./NuttX/misc/pascal/insn16/prun/pdbg.o \
./NuttX/misc/pascal/insn16/prun/pexec.o \
./NuttX/misc/pascal/insn16/prun/pload.o \
./NuttX/misc/pascal/insn16/prun/prun.o 

C_DEPS += \
./NuttX/misc/pascal/insn16/prun/pdbg.d \
./NuttX/misc/pascal/insn16/prun/pexec.d \
./NuttX/misc/pascal/insn16/prun/pload.d \
./NuttX/misc/pascal/insn16/prun/prun.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/misc/pascal/insn16/prun/%.o: ../NuttX/misc/pascal/insn16/prun/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


