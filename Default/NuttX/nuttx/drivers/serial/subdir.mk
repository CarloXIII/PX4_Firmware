################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/drivers/serial/lowconsole.c \
../NuttX/nuttx/drivers/serial/serial.c \
../NuttX/nuttx/drivers/serial/serialirq.c \
../NuttX/nuttx/drivers/serial/uart_16550.c 

OBJS += \
./NuttX/nuttx/drivers/serial/lowconsole.o \
./NuttX/nuttx/drivers/serial/serial.o \
./NuttX/nuttx/drivers/serial/serialirq.o \
./NuttX/nuttx/drivers/serial/uart_16550.o 

C_DEPS += \
./NuttX/nuttx/drivers/serial/lowconsole.d \
./NuttX/nuttx/drivers/serial/serial.d \
./NuttX/nuttx/drivers/serial/serialirq.d \
./NuttX/nuttx/drivers/serial/uart_16550.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/drivers/serial/%.o: ../NuttX/nuttx/drivers/serial/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


