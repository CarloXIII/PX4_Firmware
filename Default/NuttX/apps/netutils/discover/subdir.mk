################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/apps/netutils/discover/discover.c 

OBJS += \
./NuttX/apps/netutils/discover/discover.o 

C_DEPS += \
./NuttX/apps/netutils/discover/discover.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/apps/netutils/discover/%.o: ../NuttX/apps/netutils/discover/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


