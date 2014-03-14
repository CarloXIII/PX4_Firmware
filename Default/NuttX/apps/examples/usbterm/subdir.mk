################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/apps/examples/usbterm/usbterm_main.c 

OBJS += \
./NuttX/apps/examples/usbterm/usbterm_main.o 

C_DEPS += \
./NuttX/apps/examples/usbterm/usbterm_main.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/apps/examples/usbterm/%.o: ../NuttX/apps/examples/usbterm/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


