################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/apps/examples/dhcpd/host.c \
../NuttX/apps/examples/dhcpd/target.c 

OBJS += \
./NuttX/apps/examples/dhcpd/host.o \
./NuttX/apps/examples/dhcpd/target.o 

C_DEPS += \
./NuttX/apps/examples/dhcpd/host.d \
./NuttX/apps/examples/dhcpd/target.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/apps/examples/dhcpd/%.o: ../NuttX/apps/examples/dhcpd/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


