################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/arch/rgmp/src/arm/arch_nuttx.c 

S_UPPER_SRCS += \
../NuttX/nuttx/arch/rgmp/src/arm/sigentry.S 

OBJS += \
./NuttX/nuttx/arch/rgmp/src/arm/arch_nuttx.o \
./NuttX/nuttx/arch/rgmp/src/arm/sigentry.o 

C_DEPS += \
./NuttX/nuttx/arch/rgmp/src/arm/arch_nuttx.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/arch/rgmp/src/arm/%.o: ../NuttX/nuttx/arch/rgmp/src/arm/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

NuttX/nuttx/arch/rgmp/src/arm/%.o: ../NuttX/nuttx/arch/rgmp/src/arm/%.S
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Assembler'
	as  -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


