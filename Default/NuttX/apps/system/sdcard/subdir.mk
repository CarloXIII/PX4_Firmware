################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/apps/system/sdcard/sdcard.c 

OBJS += \
./NuttX/apps/system/sdcard/sdcard.o 

C_DEPS += \
./NuttX/apps/system/sdcard/sdcard.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/apps/system/sdcard/%.o: ../NuttX/apps/system/sdcard/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


