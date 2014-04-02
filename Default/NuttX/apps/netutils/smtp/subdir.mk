################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/apps/netutils/smtp/smtp.c 

OBJS += \
./NuttX/apps/netutils/smtp/smtp.o 

C_DEPS += \
./NuttX/apps/netutils/smtp/smtp.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/apps/netutils/smtp/%.o: ../NuttX/apps/netutils/smtp/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


