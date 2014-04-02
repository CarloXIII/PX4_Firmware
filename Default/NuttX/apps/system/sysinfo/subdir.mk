################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/apps/system/sysinfo/sysinfo.c 

OBJS += \
./NuttX/apps/system/sysinfo/sysinfo.o 

C_DEPS += \
./NuttX/apps/system/sysinfo/sysinfo.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/apps/system/sysinfo/%.o: ../NuttX/apps/system/sysinfo/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


