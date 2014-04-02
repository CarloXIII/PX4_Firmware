################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/drivers/sercomm/console.c \
../NuttX/nuttx/drivers/sercomm/uart.c 

OBJS += \
./NuttX/nuttx/drivers/sercomm/console.o \
./NuttX/nuttx/drivers/sercomm/uart.o 

C_DEPS += \
./NuttX/nuttx/drivers/sercomm/console.d \
./NuttX/nuttx/drivers/sercomm/uart.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/drivers/sercomm/%.o: ../NuttX/nuttx/drivers/sercomm/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


