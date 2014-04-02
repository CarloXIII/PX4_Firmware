################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/misc/pascal/insn16/popt/pcopt.c \
../NuttX/misc/pascal/insn16/popt/pfopt.c \
../NuttX/misc/pascal/insn16/popt/pjopt.c \
../NuttX/misc/pascal/insn16/popt/plopt.c \
../NuttX/misc/pascal/insn16/popt/polocal.c \
../NuttX/misc/pascal/insn16/popt/popt.c \
../NuttX/misc/pascal/insn16/popt/psopt.c 

OBJS += \
./NuttX/misc/pascal/insn16/popt/pcopt.o \
./NuttX/misc/pascal/insn16/popt/pfopt.o \
./NuttX/misc/pascal/insn16/popt/pjopt.o \
./NuttX/misc/pascal/insn16/popt/plopt.o \
./NuttX/misc/pascal/insn16/popt/polocal.o \
./NuttX/misc/pascal/insn16/popt/popt.o \
./NuttX/misc/pascal/insn16/popt/psopt.o 

C_DEPS += \
./NuttX/misc/pascal/insn16/popt/pcopt.d \
./NuttX/misc/pascal/insn16/popt/pfopt.d \
./NuttX/misc/pascal/insn16/popt/pjopt.d \
./NuttX/misc/pascal/insn16/popt/plopt.d \
./NuttX/misc/pascal/insn16/popt/polocal.d \
./NuttX/misc/pascal/insn16/popt/popt.d \
./NuttX/misc/pascal/insn16/popt/psopt.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/misc/pascal/insn16/popt/%.o: ../NuttX/misc/pascal/insn16/popt/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


