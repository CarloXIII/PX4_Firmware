################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/configs/qemu-i486/src/up_boot.c 

OBJS += \
./NuttX/nuttx/configs/qemu-i486/src/up_boot.o 

C_DEPS += \
./NuttX/nuttx/configs/qemu-i486/src/up_boot.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/configs/qemu-i486/src/%.o: ../NuttX/nuttx/configs/qemu-i486/src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


