################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/misc/pascal/insn16/libinsn/paddopcode.c \
../NuttX/misc/pascal/insn16/libinsn/paddtmpopcode.c \
../NuttX/misc/pascal/insn16/libinsn/pdasm.c \
../NuttX/misc/pascal/insn16/libinsn/pgen.c \
../NuttX/misc/pascal/insn16/libinsn/pgetopcode.c \
../NuttX/misc/pascal/insn16/libinsn/preloc.c 

OBJS += \
./NuttX/misc/pascal/insn16/libinsn/paddopcode.o \
./NuttX/misc/pascal/insn16/libinsn/paddtmpopcode.o \
./NuttX/misc/pascal/insn16/libinsn/pdasm.o \
./NuttX/misc/pascal/insn16/libinsn/pgen.o \
./NuttX/misc/pascal/insn16/libinsn/pgetopcode.o \
./NuttX/misc/pascal/insn16/libinsn/preloc.o 

C_DEPS += \
./NuttX/misc/pascal/insn16/libinsn/paddopcode.d \
./NuttX/misc/pascal/insn16/libinsn/paddtmpopcode.d \
./NuttX/misc/pascal/insn16/libinsn/pdasm.d \
./NuttX/misc/pascal/insn16/libinsn/pgen.d \
./NuttX/misc/pascal/insn16/libinsn/pgetopcode.d \
./NuttX/misc/pascal/insn16/libinsn/preloc.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/misc/pascal/insn16/libinsn/%.o: ../NuttX/misc/pascal/insn16/libinsn/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


