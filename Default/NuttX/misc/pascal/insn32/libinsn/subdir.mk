################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/misc/pascal/insn32/libinsn/paddopcode.c \
../NuttX/misc/pascal/insn32/libinsn/paddtmpopcode.c \
../NuttX/misc/pascal/insn32/libinsn/pdasm.c \
../NuttX/misc/pascal/insn32/libinsn/pgen.c \
../NuttX/misc/pascal/insn32/libinsn/pgetopcode.c \
../NuttX/misc/pascal/insn32/libinsn/preloc.c \
../NuttX/misc/pascal/insn32/libinsn/presettmpopcodewrite.c 

OBJS += \
./NuttX/misc/pascal/insn32/libinsn/paddopcode.o \
./NuttX/misc/pascal/insn32/libinsn/paddtmpopcode.o \
./NuttX/misc/pascal/insn32/libinsn/pdasm.o \
./NuttX/misc/pascal/insn32/libinsn/pgen.o \
./NuttX/misc/pascal/insn32/libinsn/pgetopcode.o \
./NuttX/misc/pascal/insn32/libinsn/preloc.o \
./NuttX/misc/pascal/insn32/libinsn/presettmpopcodewrite.o 

C_DEPS += \
./NuttX/misc/pascal/insn32/libinsn/paddopcode.d \
./NuttX/misc/pascal/insn32/libinsn/paddtmpopcode.d \
./NuttX/misc/pascal/insn32/libinsn/pdasm.d \
./NuttX/misc/pascal/insn32/libinsn/pgen.d \
./NuttX/misc/pascal/insn32/libinsn/pgetopcode.d \
./NuttX/misc/pascal/insn32/libinsn/preloc.d \
./NuttX/misc/pascal/insn32/libinsn/presettmpopcodewrite.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/misc/pascal/insn32/libinsn/%.o: ../NuttX/misc/pascal/insn32/libinsn/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


