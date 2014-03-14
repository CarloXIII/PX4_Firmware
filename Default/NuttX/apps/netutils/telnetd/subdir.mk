################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/apps/netutils/telnetd/telnetd_daemon.c \
../NuttX/apps/netutils/telnetd/telnetd_driver.c 

OBJS += \
./NuttX/apps/netutils/telnetd/telnetd_daemon.o \
./NuttX/apps/netutils/telnetd/telnetd_driver.o 

C_DEPS += \
./NuttX/apps/netutils/telnetd/telnetd_daemon.d \
./NuttX/apps/netutils/telnetd/telnetd_driver.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/apps/netutils/telnetd/%.o: ../NuttX/apps/netutils/telnetd/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


