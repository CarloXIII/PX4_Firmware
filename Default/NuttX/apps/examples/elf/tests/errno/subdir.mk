################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/apps/examples/elf/tests/errno/errno.c 

OBJS += \
./NuttX/apps/examples/elf/tests/errno/errno.o 

C_DEPS += \
./NuttX/apps/examples/elf/tests/errno/errno.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/apps/examples/elf/tests/errno/%.o: ../NuttX/apps/examples/elf/tests/errno/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


