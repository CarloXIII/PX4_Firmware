################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/apps/system/free/free.c 

OBJS += \
./NuttX/apps/system/free/free.o 

C_DEPS += \
./NuttX/apps/system/free/free.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/apps/system/free/%.o: ../NuttX/apps/system/free/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


