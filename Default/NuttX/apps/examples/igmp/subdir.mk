################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/apps/examples/igmp/igmp.c 

OBJS += \
./NuttX/apps/examples/igmp/igmp.o 

C_DEPS += \
./NuttX/apps/examples/igmp/igmp.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/apps/examples/igmp/%.o: ../NuttX/apps/examples/igmp/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


