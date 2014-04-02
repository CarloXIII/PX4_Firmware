################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/configs/open1788/kernel/up_userspace.c 

OBJS += \
./NuttX/nuttx/configs/open1788/kernel/up_userspace.o 

C_DEPS += \
./NuttX/nuttx/configs/open1788/kernel/up_userspace.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/configs/open1788/kernel/%.o: ../NuttX/nuttx/configs/open1788/kernel/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


