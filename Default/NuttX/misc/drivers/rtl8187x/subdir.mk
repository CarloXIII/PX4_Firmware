################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/misc/drivers/rtl8187x/rtl8187x.c 

OBJS += \
./NuttX/misc/drivers/rtl8187x/rtl8187x.o 

C_DEPS += \
./NuttX/misc/drivers/rtl8187x/rtl8187x.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/misc/drivers/rtl8187x/%.o: ../NuttX/misc/drivers/rtl8187x/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


