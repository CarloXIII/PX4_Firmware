################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/apps/modbus/nuttx/portevent.c \
../NuttX/apps/modbus/nuttx/portother.c \
../NuttX/apps/modbus/nuttx/portserial.c \
../NuttX/apps/modbus/nuttx/porttimer.c 

OBJS += \
./NuttX/apps/modbus/nuttx/portevent.o \
./NuttX/apps/modbus/nuttx/portother.o \
./NuttX/apps/modbus/nuttx/portserial.o \
./NuttX/apps/modbus/nuttx/porttimer.o 

C_DEPS += \
./NuttX/apps/modbus/nuttx/portevent.d \
./NuttX/apps/modbus/nuttx/portother.d \
./NuttX/apps/modbus/nuttx/portserial.d \
./NuttX/apps/modbus/nuttx/porttimer.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/apps/modbus/nuttx/%.o: ../NuttX/apps/modbus/nuttx/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


