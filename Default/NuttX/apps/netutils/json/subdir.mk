################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/apps/netutils/json/cJSON.c 

OBJS += \
./NuttX/apps/netutils/json/cJSON.o 

C_DEPS += \
./NuttX/apps/netutils/json/cJSON.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/apps/netutils/json/%.o: ../NuttX/apps/netutils/json/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


