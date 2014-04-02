################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/misc/pascal/insn32/regm/regm.c \
../NuttX/misc/pascal/insn32/regm/regm_pass1.c \
../NuttX/misc/pascal/insn32/regm/regm_pass2.c \
../NuttX/misc/pascal/insn32/regm/regm_registers2.c \
../NuttX/misc/pascal/insn32/regm/regm_tree.c 

OBJS += \
./NuttX/misc/pascal/insn32/regm/regm.o \
./NuttX/misc/pascal/insn32/regm/regm_pass1.o \
./NuttX/misc/pascal/insn32/regm/regm_pass2.o \
./NuttX/misc/pascal/insn32/regm/regm_registers2.o \
./NuttX/misc/pascal/insn32/regm/regm_tree.o 

C_DEPS += \
./NuttX/misc/pascal/insn32/regm/regm.d \
./NuttX/misc/pascal/insn32/regm/regm_pass1.d \
./NuttX/misc/pascal/insn32/regm/regm_pass2.d \
./NuttX/misc/pascal/insn32/regm/regm_registers2.d \
./NuttX/misc/pascal/insn32/regm/regm_tree.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/misc/pascal/insn32/regm/%.o: ../NuttX/misc/pascal/insn32/regm/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


