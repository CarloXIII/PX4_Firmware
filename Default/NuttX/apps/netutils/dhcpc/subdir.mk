################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/apps/netutils/dhcpc/dhcpc.c 

OBJS += \
./NuttX/apps/netutils/dhcpc/dhcpc.o 

C_DEPS += \
./NuttX/apps/netutils/dhcpc/dhcpc.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/apps/netutils/dhcpc/%.o: ../NuttX/apps/netutils/dhcpc/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


