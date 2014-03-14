################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/arch/arm/src/board/empty.c 

OBJS += \
./NuttX/nuttx/arch/arm/src/board/empty.o 

C_DEPS += \
./NuttX/nuttx/arch/arm/src/board/empty.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/arch/arm/src/board/%.o: ../NuttX/nuttx/arch/arm/src/board/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


