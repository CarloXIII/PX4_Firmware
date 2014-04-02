################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/arch/rgmp/src/x86/arch_nuttx.c \
../NuttX/nuttx/arch/rgmp/src/x86/com.c 

S_UPPER_SRCS += \
../NuttX/nuttx/arch/rgmp/src/x86/sigentry.S 

OBJS += \
./NuttX/nuttx/arch/rgmp/src/x86/arch_nuttx.o \
./NuttX/nuttx/arch/rgmp/src/x86/com.o \
./NuttX/nuttx/arch/rgmp/src/x86/sigentry.o 

C_DEPS += \
./NuttX/nuttx/arch/rgmp/src/x86/arch_nuttx.d \
./NuttX/nuttx/arch/rgmp/src/x86/com.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/arch/rgmp/src/x86/%.o: ../NuttX/nuttx/arch/rgmp/src/x86/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

NuttX/nuttx/arch/rgmp/src/x86/%.o: ../NuttX/nuttx/arch/rgmp/src/x86/%.S
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Assembler'
	as  -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


