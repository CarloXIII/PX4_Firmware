################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/apps/examples/pashello/device.c \
../NuttX/apps/examples/pashello/pashello.c 

OBJS += \
./NuttX/apps/examples/pashello/device.o \
./NuttX/apps/examples/pashello/pashello.o 

C_DEPS += \
./NuttX/apps/examples/pashello/device.d \
./NuttX/apps/examples/pashello/pashello.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/apps/examples/pashello/%.o: ../NuttX/apps/examples/pashello/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


