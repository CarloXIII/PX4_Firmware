################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/apps/interpreters/ficl/src/nuttx.c 

OBJS += \
./NuttX/apps/interpreters/ficl/src/nuttx.o 

C_DEPS += \
./NuttX/apps/interpreters/ficl/src/nuttx.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/apps/interpreters/ficl/src/%.o: ../NuttX/apps/interpreters/ficl/src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


