################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/apps/netutils/dhcpd/dhcpd.c 

OBJS += \
./NuttX/apps/netutils/dhcpd/dhcpd.o 

C_DEPS += \
./NuttX/apps/netutils/dhcpd/dhcpd.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/apps/netutils/dhcpd/%.o: ../NuttX/apps/netutils/dhcpd/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


