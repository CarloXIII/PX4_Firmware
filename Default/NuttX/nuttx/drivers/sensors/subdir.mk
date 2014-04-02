################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/drivers/sensors/lis331dl.c \
../NuttX/nuttx/drivers/sensors/lm75.c \
../NuttX/nuttx/drivers/sensors/qencoder.c 

OBJS += \
./NuttX/nuttx/drivers/sensors/lis331dl.o \
./NuttX/nuttx/drivers/sensors/lm75.o \
./NuttX/nuttx/drivers/sensors/qencoder.o 

C_DEPS += \
./NuttX/nuttx/drivers/sensors/lis331dl.d \
./NuttX/nuttx/drivers/sensors/lm75.d \
./NuttX/nuttx/drivers/sensors/qencoder.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/drivers/sensors/%.o: ../NuttX/nuttx/drivers/sensors/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


