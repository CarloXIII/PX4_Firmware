################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/drivers/syslog/ramlog.c 

OBJS += \
./NuttX/nuttx/drivers/syslog/ramlog.o 

C_DEPS += \
./NuttX/nuttx/drivers/syslog/ramlog.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/drivers/syslog/%.o: ../NuttX/nuttx/drivers/syslog/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


