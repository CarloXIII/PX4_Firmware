################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/apps/modbus/functions/mbfunccoils.c \
../NuttX/apps/modbus/functions/mbfuncdiag.c \
../NuttX/apps/modbus/functions/mbfuncdisc.c \
../NuttX/apps/modbus/functions/mbfuncholding.c \
../NuttX/apps/modbus/functions/mbfuncinput.c \
../NuttX/apps/modbus/functions/mbfuncother.c \
../NuttX/apps/modbus/functions/mbutils.c 

OBJS += \
./NuttX/apps/modbus/functions/mbfunccoils.o \
./NuttX/apps/modbus/functions/mbfuncdiag.o \
./NuttX/apps/modbus/functions/mbfuncdisc.o \
./NuttX/apps/modbus/functions/mbfuncholding.o \
./NuttX/apps/modbus/functions/mbfuncinput.o \
./NuttX/apps/modbus/functions/mbfuncother.o \
./NuttX/apps/modbus/functions/mbutils.o 

C_DEPS += \
./NuttX/apps/modbus/functions/mbfunccoils.d \
./NuttX/apps/modbus/functions/mbfuncdiag.d \
./NuttX/apps/modbus/functions/mbfuncdisc.d \
./NuttX/apps/modbus/functions/mbfuncholding.d \
./NuttX/apps/modbus/functions/mbfuncinput.d \
./NuttX/apps/modbus/functions/mbfuncother.d \
./NuttX/apps/modbus/functions/mbutils.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/apps/modbus/functions/%.o: ../NuttX/apps/modbus/functions/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


